"""绿、黄小球识别模块，适配 OpenMV 5.0。

巡线和识球共用同一帧。检测阶段不在原图上绘制，避免调试图形被下一步颜色检测
误识别；全部检测结束后再由 DrawResults() 统一显示结果。红色只作为巡线路线，
不参与小球计数，从源头避免红线被误认为小球。
"""

import sensor
import AnoMessage as Message


# LAB 颜色阈值，现场光照、球体材质不同，应使用 OpenMV IDE 阈值编辑器校准
# RED_THRESHOLD 仅保留给旧调试脚本兼容，本任务不会用它搜索小球
RED_THRESHOLD = (35, 100, 25, 127, -25, 80)
GREEN_THRESHOLD = (30, 100, -80, -15, -20, 70)
YELLOW_THRESHOLD = (50, 100, -20, 25, 25, 127)

# 小球几何约束：同时使用面积、长宽比、填充率、圆度和紧致度排除色带/反光
PIXELS_THRESHOLD = 35
AREA_THRESHOLD = 55
MIN_SIZE = 6
MAX_SIZE = 60
MIN_ASPECT = 0.58
MAX_ASPECT = 1.72
MIN_DENSITY = 0.36
MIN_ROUNDNESS = 0.34
MIN_COMPACTNESS = 0.22

# 两帧确认可滤除瞬时反光；允许丢失一帧，避免计数和主目标坐标闪烁
TRACK_CONFIRM_HITS = 2
# 轨迹保留时间长于画面显示保持时间：短暂漏检后仍关联到原球，避免重复累计
TRACK_MAX_MISSES = 8
DISPLAY_MAX_MISSES = 1
TRACK_MIN_GATE = 14


class ColorResult(object):
    def __init__(self):
        self.clear()

    def clear(self):
        self.flag = 0
        self.red_count = 0
        self.green_count = 0
        self.yellow_count = 0
        # 坐标以图像中心为原点：x 向右为正，y 向下为正（与 crossy 同一框架）
        self.red_positions = []
        self.green_positions = []
        self.yellow_positions = []

        # 每种颜色面积最大的小球位置，供固定长度通信协议使用
        self.red_x = 0
        self.red_y = 0
        self.green_x = 0
        self.green_y = 0
        self.yellow_x = 0
        self.yellow_y = 0

        # 当前帧中已确认目标的调试绘图数据
        self.items = []


Result = ColorResult()

# 每种颜色独立跟踪，元素均为小字典，QQVGA 下内存占用很小
_tracks = {1: [], 2: [], 3: []}


def ResetTracking():
    """清空跨帧目标轨迹；开始新任务时调用。"""
    global _tracks
    _tracks = {1: [], 2: [], 3: []}


def _value(obj, name):
    """兼容 OpenMV 不同固件中 blob.w 与 blob.w() 两种接口。"""
    value = getattr(obj, name)
    return value() if callable(value) else value


def _clamp_count(value):
    if value < 0:
        return 0
    if value > 255:
        return 255
    return value


def _is_ball(blob):
    """依据多种几何特征排除细长色带、噪点和大面积背景。"""
    width = _value(blob, 'w')
    height = _value(blob, 'h')
    density = _value(blob, 'density')
    roundness = _value(blob, 'roundness')
    compactness = _value(blob, 'compactness')

    if width < MIN_SIZE or height < MIN_SIZE:
        return False
    if width > MAX_SIZE or height > MAX_SIZE:
        return False

    aspect = width / float(height)
    if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
        return False
    if density < MIN_DENSITY:
        return False

    # 被轻微遮挡的球可能某一项偏低，因此仅在两项都差时剔除
    if roundness < MIN_ROUNDNESS and compactness < MIN_COMPACTNESS:
        return False
    return True


def _color_confidence(img, blob, threshold, color_id):
    """用候选区域内通过阈值的 LAB 均值解决绿/黄阈值边界重叠。"""
    stats = img.get_statistics(thresholds=[threshold], roi=_value(blob, 'rect'))
    a_value = _value(stats, 'a_mean')
    b_value = _value(stats, 'b_mean')

    if color_id == 1:
        # 红色由正 A 通道主导；较大的正 B 值更像黄/橙色
        return a_value - max(0, b_value) * 0.15
    if color_id == 2:
        # 绿色由负 A 通道主导
        return -a_value - max(0, b_value) * 0.05
    # 黄色由正 B 通道主导，A 通道应靠近中性
    return b_value - abs(a_value) * 0.30


def _find_candidates(img, threshold, color_id):
    blobs = img.find_blobs(
        [threshold],
        pixels_threshold=PIXELS_THRESHOLD,
        area_threshold=AREA_THRESHOLD,
        merge=False,
    )

    candidates = []
    for blob in blobs:
        if not _is_ball(blob):
            continue

        x = _value(blob, 'x')
        y = _value(blob, 'y')
        width = _value(blob, 'w')
        height = _value(blob, 'h')
        center_x = _value(blob, 'cx')
        center_y = _value(blob, 'cy')
        pixels = _value(blob, 'pixels')
        rect = _value(blob, 'rect')
        density = _value(blob, 'density')
        roundness = _value(blob, 'roundness')
        compactness = _value(blob, 'compactness')
        color_score = _color_confidence(img, blob, threshold, color_id)
        shape_score = (
            density * 24.0
            + roundness * 28.0
            + compactness * 16.0
        )
        score = shape_score + color_score * 1.5 + pixels * 0.20
        candidates.append({
            'color': color_id,
            'x': x,
            'y': y,
            'w': width,
            'h': height,
            'cx': center_x,
            'cy': center_y,
            'pixels': pixels,
            'rect': rect,
            'score': score,
        })
    return candidates


def _same_object(first, second):
    """判断两个不同颜色阈值得到的候选是否实际来自同一个色块。"""
    left = max(first['x'], second['x'])
    top = max(first['y'], second['y'])
    right = min(first['x'] + first['w'], second['x'] + second['w'])
    bottom = min(first['y'] + first['h'], second['y'] + second['h'])
    if right <= left or bottom <= top:
        return False

    overlap = (right - left) * (bottom - top)
    min_area = min(first['w'] * first['h'], second['w'] * second['h'])
    if overlap < min_area * 0.35:
        return False

    dx = first['cx'] - second['cx']
    dy = first['cy'] - second['cy']
    gate = min(max(first['w'], first['h']), max(second['w'], second['h'])) * 0.45
    if gate < 4:
        gate = 4
    return (dx * dx + dy * dy) <= (gate * gate)


def _resolve_color_conflicts(candidates):
    """同一物体只保留颜色置信度最高的候选，防止重复计数。"""
    candidates.sort(key=lambda item: item['score'], reverse=True)
    accepted = []
    for candidate in candidates:
        duplicated = False
        for kept in accepted:
            if candidate['color'] != kept['color'] and _same_object(candidate, kept):
                duplicated = True
                break
        if not duplicated:
            accepted.append(candidate)
    return accepted


def _new_track(candidate):
    return {
        'x': candidate['cx'],
        'y': candidate['cy'],
        'w': candidate['w'],
        'h': candidate['h'],
        'pixels': candidate['pixels'],
        'rect': candidate['rect'],
        'hits': 1,
        'misses': 0,
        'visible': True,
    }


def _update_tracks(color_id, candidates, center_x, center_y):
    """最近邻关联并进行坐标平滑，返回已确认目标。"""
    tracks = _tracks[color_id]
    used = [False] * len(candidates)

    for track in tracks:
        best_index = -1
        best_distance = 0x7FFFFFFF
        for index in range(len(candidates)):
            if used[index]:
                continue
            candidate = candidates[index]
            dx = candidate['cx'] - track['x']
            dy = candidate['cy'] - track['y']
            distance = dx * dx + dy * dy
            gate = max(TRACK_MIN_GATE, max(candidate['w'], candidate['h']) * 0.75)
            if distance <= gate * gate and distance < best_distance:
                best_distance = distance
                best_index = index

        if best_index >= 0:
            candidate = candidates[best_index]
            used[best_index] = True
            # 2/3 历史值 + 1/3 当前值，抑制像素级跳动
            track['x'] = int((track['x'] * 2 + candidate['cx']) / 3)
            track['y'] = int((track['y'] * 2 + candidate['cy']) / 3)
            track['w'] = candidate['w']
            track['h'] = candidate['h']
            track['pixels'] = candidate['pixels']
            track['rect'] = candidate['rect']
            track['hits'] = min(track['hits'] + 1, 255)
            track['misses'] = 0
            track['visible'] = True
        else:
            track['misses'] += 1
            track['visible'] = False

    for index in range(len(candidates)):
        if not used[index]:
            tracks.append(_new_track(candidates[index]))

    active_tracks = []
    for track in tracks:
        if track['misses'] <= TRACK_MAX_MISSES:
            active_tracks.append(track)
    _tracks[color_id] = active_tracks

    positions = []
    items = []
    largest_pixels = -1
    largest_x = 0
    largest_y = 0
    for track in active_tracks:
        if track['hits'] < TRACK_CONFIRM_HITS:
            continue

        # 轨迹可保留更久用于重识别；当前帧数量只短暂保持一次漏检
        if track['misses'] > DISPLAY_MAX_MISSES:
            continue

        relative_x = track['x'] - center_x
        relative_y = track['y'] - center_y
        positions.append((relative_x, relative_y))
        if track['visible']:
            items.append((color_id, track['rect'], track['x'], track['y'], track['pixels']))

        if track['pixels'] > largest_pixels:
            largest_pixels = track['pixels']
            largest_x = relative_x
            largest_y = relative_y

    return positions, items, largest_x, largest_y


def _draw_results(img):
    for item in Result.items:
        color_id, rect, cx, cy, _ = item
        if color_id == 1:
            label = 'R'
            draw_color = (255, 0, 0)
        elif color_id == 2:
            label = 'G'
            draw_color = (0, 255, 0)
        else:
            label = 'Y'
            draw_color = (255, 255, 0)

        img.draw_rectangle(rect, color=draw_color, thickness=2)
        img.draw_cross(cx, cy, color=draw_color, size=5)
        text_y = max(0, rect[1] - 10)
        img.draw_string(rect[0], text_y, label, color=draw_color)

    text_y = img.height() - 10
    img.draw_string(
        0, text_y,
        'G%d Y%d' % (Result.green_count, Result.yellow_count),
        color=(255, 255, 255),
    )


def DrawResults(img):
    """全部视觉检测结束后调用，避免绘图污染待识别图像。"""
    _draw_results(img)


def Color(img=None, send=True, draw=True):
    """识别一帧中的绿、黄小球并返回 Result。send 参数保留兼容，实际不发送（统一用 FusionDataPack）。"""
    if img is None:
        img = sensor.snapshot()

    Result.clear()
    center_x = img.width() // 2
    center_y = img.height() // 2

    all_candidates = []
    # 不搜索红色小球：本任务的红色目标是正方形巡线路线
    all_candidates += _find_candidates(img, GREEN_THRESHOLD, 2)
    all_candidates += _find_candidates(img, YELLOW_THRESHOLD, 3)
    accepted = _resolve_color_conflicts(all_candidates)

    by_color = {1: [], 2: [], 3: []}
    for candidate in accepted:
        by_color[candidate['color']].append(candidate)

    red_positions, red_items, red_x, red_y = [], [], 0, 0
    green_positions, green_items, green_x, green_y = _update_tracks(
        2, by_color[2], center_x, center_y
    )
    yellow_positions, yellow_items, yellow_x, yellow_y = _update_tracks(
        3, by_color[3], center_x, center_y
    )

    Result.red_positions = red_positions
    Result.green_positions = green_positions
    Result.yellow_positions = yellow_positions
    Result.red_count = _clamp_count(len(red_positions))
    Result.green_count = _clamp_count(len(green_positions))
    Result.yellow_count = _clamp_count(len(yellow_positions))
    Result.red_x = red_x
    Result.red_y = red_y
    Result.green_x = green_x
    Result.green_y = green_y
    Result.yellow_x = yellow_x
    Result.yellow_y = yellow_y
    Result.items = red_items + green_items + yellow_items
    Result.flag = 1 if (
        Result.red_count or Result.green_count or Result.yellow_count
    ) else 0
    if draw:
        DrawResults(img)

    return Result
