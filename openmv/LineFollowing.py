#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
"""巡线、转角和小球融合识别。

核心流程：拍照 → 五区域色块检测 → 来线密集采样+稳健拟合 → 交点检测(双中线/行走法)
→ 状态判定与防抖 → angle/distance 低通 → 交点卡尔曼跟踪 → 识球 → 拐角累计 → 打包发送。

交点检测采用双中线拟合法（来线中心线 × 横臂中心线解联立方程）与行走法
（沿来线逐行测横向展宽定位拐点）双路径，倾斜天然免疫；交点输出用恒速卡尔曼滤波
跟踪，量测按方法质量加权，野值门控剔除，丢失期用速度状态外推。
"""
import sensor, math, time
import AnoMessage as Message
import Colortracking
import CornerStatistics


# 巡线路线颜色阈值（LAB），实地测试标定；黑线只需替换此常量
LINE_THRESHOLD = (0, 100, 21, 127, -128, 127)

RAD_TO_ANGLE = 57.2958
IMG_WIDTH = 160
IMG_HEIGHT = 120
IMG_CENTER_X = IMG_WIDTH // 2
IMG_CENTER_Y = IMG_HEIGHT // 2

# 三条横向 ROI 用于拟合主线，两条边缘 ROI 仅用于判断水平分支/转角
ROIS = {
    'down':   (0, 98, 160, 22),
    'middle': (0, 50, 160, 20),
    'up':     (0, 0, 160, 20),
    'left':   (0, 8, 20, 104),
    'right':  (140, 8, 20, 104),
}

HORIZONTAL_ROIS = ('down', 'middle', 'up')
LINE_CONFIRM_FRAMES = 2
LINE_HOLD_LOST_FRAMES = 3  # 正上方时 down 易丢线，保持 3 帧防止状态抖动
MAX_LINE_ANGLE = 60
MAX_LINE_OFFSET = 79


class LineData(object):
    """巡线识别输出，供 AnoMessage.py 打包发送。"""
    flag = 0
    angle = 0
    distance = 0
    cross_x = 0
    cross_y = 0
    cross_flag = 0


class _AxisKF(object):
    """单轴恒速卡尔曼滤波：状态 [位置, 速度]，2x2 协方差解析展开。

    x/y 两轴完全解耦（恒速模型无交叉项），两个实例等效 4 状态 KF，
    纯标量浮点运算，适配 OpenMV 算力。
    """
    __slots__ = ('valid', 's0', 's1', 'p00', 'p01', 'p11')

    def __init__(self):
        self.valid = False
        self.s0 = 0.0
        self.s1 = 0.0
        self.p00 = 0.0
        self.p01 = 0.0
        self.p11 = 0.0

    def reseed(self, z, r, v_unc):
        """重新播种：位置=量测，速度=0，位置方差=量测噪声，速度方差=先验。"""
        self.valid = True
        self.s0 = float(z)
        self.s1 = 0.0
        self.p00 = float(r)
        self.p01 = 0.0
        self.p11 = float(v_unc)

    def predict(self, dt, q):
        """恒速外推：F=[[1,dt],[0,1]]，Q=q*[[dt^3/3,dt^2/2],[dt^2/2,dt]]。"""
        if not self.valid:
            return
        self.s0 += self.s1 * dt
        p00 = (self.p00 + dt * (self.p01 + self.p01 + dt * self.p11)
               + q * dt * dt * dt / 3.0)
        p01 = self.p01 + dt * self.p11 + q * dt * dt / 2.0
        p11 = self.p11 + dt * q
        self.p00 = p00
        self.p01 = p01
        self.p11 = p11

    def gate_reject(self, z, r, sigma):
        """innovation 门控：|z-预测| > sigma*sqrt(P00+R) 判为野值。"""
        s = self.p00 + r
        return abs(z - self.s0) > sigma * math.sqrt(s)

    def update(self, z, r):
        if not self.valid:
            return
        s = self.p00 + r
        innov = z - self.s0
        k0 = self.p00 / s
        k1 = self.p01 / s
        self.s0 += k0 * innov
        self.s1 += k1 * innov
        b = self.p01  # 对称更新需旧 P01，先保存
        self.p00 -= k0 * self.p00
        self.p01 -= k0 * self.p01
        self.p11 -= k1 * b

    def clamp_speed(self, max_v):
        if self.s1 > max_v:
            self.s1 = max_v
        elif self.s1 < -max_v:
            self.s1 = -max_v


Line = LineData()

_last_roi_x = {'down': IMG_CENTER_X, 'middle': IMG_CENTER_X, 'up': IMG_CENTER_X}
_stable_flag = 0
_pending_flag = 0
_pending_count = 0
_lost_count = 0
_filtered_distance = 0
_filtered_angle = 0
_filter_valid = False
_debug_count = 0

# 交点跟踪：恒速卡尔曼滤波（x/y 两轴解耦）
#   检测有效 → predict+update（按检测方法质量给 R），连续 N 次收敛后输出
#   检测丢失/门控拒绝 → 仅 predict，用速度状态外推（代替旧"冻结保持"），
#   正上方丢失期输出仍沿真实趋势逼近拐点，不再滞后
_cross_kf_x = _AxisKF()
_cross_kf_y = _AxisKF()
_cross_lost_count = 0
_cross_update_streak = 0
_cross_confirmed = False
_cross_last_tick = 0
CROSS_HOLD_FRAMES = 6       # 丢失后纯外推保持帧数（约 200ms），超过则清除跟踪
CROSS_CONFIRM_FRAMES = 2    # 连续 N 帧成功更新才输出 cross_flag（滤单帧毛刺）
CROSS_Q = 200.0             # 过程噪声（加速度 PSD，px^2/s^3）
CROSS_INIT_V_UNC = 3600.0   # 播种时速度方差 (px/s)^2，60px/s 的不确定度
CROSS_MAX_SPEED = 200.0     # 速度状态限幅（px/s），旋转 27°/s 时拐点画面移动 ~120px/s
CROSS_GATE_SIGMA = 3.0      # innovation 门控倍数
CROSS_GATE_MIN_PX = 25.0    # 门控下限：innovation 须超此值才判跳变（防 P 收敛后门过紧）
# 量测噪声 R（按检测方法质量，单位 px^2）：(Rx, Ry)
#   quality=0 双中线（来线<=42°）：x/y 都准，横臂中心线多点拟合
#   quality=1 行走法（旋转无关主路径）：y=展宽突增行（准），x=来线外推（小距离）
#   quality=2 双中线降级（来线>42°）：ROI 质心有偏，给大 R 少采信
CROSS_R_TABLE = ((9.0, 9.0), (25.0, 16.0), (49.0, 64.0))

# 双中线法斜率降级：不直接 self-kill，降级给大 R
CROSS_DUAL_MAX_SLOPE = 0.9   # 来线 dx/dy 超此值（约 42°）双中线降级 quality=2
CROSS_DUAL_HARD_SLOPE = 1.4  # 超此值（约 54°）放弃双中线，行走法接管
CROSS_LINE_DIST_MAX = 8.0    # 交点到两条拟合中心线的最大允许距离（px）

# 行走法参数（旋转无关主路径，不依赖侧 ROI）
WALKER_STEP = 3              # 逐行扫描步长（px）
WALKER_Y_MIN = 6             # 扫描最远到画面上缘此行为止
WALKER_MAX_SLOPE = 1.6       # 来线拟合超此斜率（约 58°）则预测 x 发散，放弃
WALKER_SPAN_DELTA = 16       # 横向展宽超出扫描最小基线此值判为拐点行
WALKER_SPAN_MIN = 26         # 拐点行绝对展宽下限（防小目标误触发）
WALKER_ARM_MIN = 14          # 判定转向时横臂单侧延伸的最小像素数
WALKER_BLOB_DIST = 6         # 预测 x 与 blob 边缘的最大允许距离（px）

# 双中线法调试可视化（横臂采样点与拟合线）
_last_branch_pts = []
_last_branch_fit = None


def ResetMission():
    """开始新一轮正方形任务，清空巡线滤波、球轨迹和四拐角累计值。"""
    global _last_roi_x, _stable_flag, _pending_flag, _pending_count
    global _lost_count, _filtered_distance, _filtered_angle, _filter_valid
    global _debug_count
    global _cross_lost_count, _cross_update_streak, _cross_confirmed
    global _last_branch_pts, _last_branch_fit

    _last_roi_x = {
        'down': IMG_CENTER_X,
        'middle': IMG_CENTER_X,
        'up': IMG_CENTER_X,
    }
    _stable_flag = 0
    _pending_flag = 0
    _pending_count = 0
    _lost_count = 0
    _filtered_distance = 0
    _filtered_angle = 0
    _filter_valid = False
    _debug_count = 0
    _cross_kf_x.__init__()
    _cross_kf_y.__init__()
    _cross_lost_count = 0
    _cross_update_streak = 0
    _cross_confirmed = False
    _last_branch_pts = []
    _last_branch_fit = None

    Line.flag = 0
    Line.angle = 0
    Line.distance = 0
    Line.cross_flag = 0
    Line.cross_x = 0
    Line.cross_y = 0

    Colortracking.ResetTracking()
    CornerStatistics.Statistics.reset()


def _blob_value(blob, name):
    """兼容 OpenMV 不同固件中属性式和方法式 blob 接口。"""
    value = getattr(blob, name)
    return value() if callable(value) else value


def _empty_roi_result():
    return {
        'cx': -1,
        'cy': -1,
        'blob_flag': False,
        'rect': None,
        'pixels': 0,
    }


def _blob_geometry_ok(roi_name, blob):
    """按 ROI 类型做几何过滤：横 ROI 排除宽大色块（横臂干扰），侧 ROI 排除高瘦色块（来线干扰）。"""
    width = _blob_value(blob, 'w')
    height = _blob_value(blob, 'h')
    if width < 3 or height < 3:
        return False

    if roi_name in HORIZONTAL_ROIS:
        # 横 ROI 内的主线应近似竖向；允许较大倾角，但排除宽大色块
        if width > 58 or height < 5:
            return False
        if height < width * 0.55:
            return False
    else:
        # 边缘 ROI 中的转弯分支（横臂）：倾斜时横臂在 20px 宽侧条内竖向展开，
        # 高度上限放宽到 32，宽扁偏好由 _select_blob 评分实现
        if height > 32 or width < 6:
            return False
    return True


def _select_blob(img, roi_name, expected_position=None):
    """在指定 ROI 中按评分选择最佳 blob：像素数（饱和防大色块）+ 形状比例 + 位置连续性。"""
    roi = ROIS[roi_name]
    blobs = img.find_blobs(
        [LINE_THRESHOLD],
        roi=roi,
        pixels_threshold=12,
        area_threshold=12,
        merge=True,
        margin=1,
    )

    best_blob = None
    best_score = -1000000
    for blob in blobs:
        if not _blob_geometry_ok(roi_name, blob):
            continue

        width = _blob_value(blob, 'w')
        height = _blob_value(blob, 'h')
        pixels = _blob_value(blob, 'pixels')
        center_x = _blob_value(blob, 'cx')

        score = min(pixels, 140) * 0.5
        if roi_name in HORIZONTAL_ROIS:
            score += min(height / float(max(1, width)), 4.0) * 10.0
            if expected_position is not None:
                score -= abs(center_x - expected_position) * 1.5
        else:
            score += min(width / float(max(1, height)), 5.0) * 10.0

        if score > best_score:
            best_score = score
            best_blob = blob

    result = _empty_roi_result()
    if best_blob is not None:
        result['cx'] = _blob_value(best_blob, 'cx')
        result['cy'] = _blob_value(best_blob, 'cy')
        result['blob_flag'] = True
        result['rect'] = _blob_value(best_blob, 'rect')
        result['pixels'] = _blob_value(best_blob, 'pixels')
    return result


def _find_blobs_in_rois(img):
    """按从近到远的顺序选择主线，使同一帧内也能利用轨迹连续性。"""
    results = {}

    down_expected = _last_roi_x['down']
    results['down'] = _select_blob(img, 'down', down_expected)

    if results['down']['blob_flag']:
        middle_expected = results['down']['cx']
    else:
        middle_expected = _last_roi_x['middle']
    # 历史值与本帧下方位置共同约束，减少小球抢占中间 ROI
    middle_expected = int((middle_expected + _last_roi_x['middle']) / 2)
    results['middle'] = _select_blob(img, 'middle', middle_expected)

    if results['middle']['blob_flag'] and results['down']['blob_flag']:
        up_expected = 2 * results['middle']['cx'] - results['down']['cx']
    elif results['middle']['blob_flag']:
        up_expected = results['middle']['cx']
    else:
        up_expected = _last_roi_x['up']
    up_expected = max(0, min(IMG_WIDTH - 1, up_expected))
    results['up'] = _select_blob(img, 'up', up_expected)

    results['left'] = _select_blob(img, 'left')
    results['right'] = _select_blob(img, 'right')

    for roi_name in HORIZONTAL_ROIS:
        if results[roi_name]['blob_flag']:
            # 历史中心轻量平滑，仅用于下一帧候选选择，不直接作为控制量
            _last_roi_x[roi_name] = int(
                (_last_roi_x[roi_name] * 2 + results[roi_name]['cx']) / 3
            )
    return results


def _raw_line_flag(results, cross_result):
    """状态判定：交点优先（独立计算），其次用 ROI 逻辑回退。

    cross_result 是 _detect_cross_precise 的返回值，独立于状态判定计算，
    打破"状态需要交点、交点需要状态"的鸡生蛋死锁。
    """
    # 优先级 1：交点有效则直接用交点的转向
    if cross_result is not None:
        return cross_result[0]  # flag=2 左转，3 右转

    # 优先级 2：ROI 逻辑（交点无效时回退）
    up_found = results['up']['blob_flag']
    middle_found = results['middle']['blob_flag']
    down_found = results['down']['blob_flag']
    left_found = results['left']['blob_flag']
    right_found = results['right']['blob_flag']
    side_xor = left_found != right_found
    has_near_line = down_found or middle_found

    # 转角判定：up 无 + 仅一侧横臂 + 近处有线
    if side_xor and not up_found and has_near_line:
        if left_found:
            return 2
        return 3

    horizontal_count = int(up_found) + int(middle_found) + int(down_found)
    if horizontal_count >= 2:
        return 1
    return 0


def _stabilize_flag(raw_flag):
    """连续两帧确认状态改变，并短暂保持一次丢线。"""
    global _stable_flag, _pending_flag, _pending_count, _lost_count

    if raw_flag == _stable_flag:
        _pending_flag = raw_flag
        _pending_count = 0
        _lost_count = 0
        return _stable_flag

    if raw_flag == 0:
        _lost_count += 1
        _pending_flag = 0
        _pending_count = 0
        if _stable_flag != 0 and _lost_count <= LINE_HOLD_LOST_FRAMES:
            return _stable_flag
        _stable_flag = 0
        return 0

    _lost_count = 0
    if raw_flag != _pending_flag:
        _pending_flag = raw_flag
        _pending_count = 1
    else:
        _pending_count += 1

    if _pending_count >= LINE_CONFIRM_FRAMES:
        _stable_flag = raw_flag
        _pending_count = 0
    return _stable_flag


def _lowpass(old, new, new_weight=2):
    """低通滤波：new_weight=2 表示新值占 2/5，旧值占 3/5。"""
    return int((old * (5 - new_weight) + new * new_weight) / 5)


def _update_measurements(raw_flag, stable_flag, raw_distance, raw_angle, has_points):
    global _filtered_distance, _filtered_angle, _filter_valid

    if raw_flag != 0 and has_points:
        if not _filter_valid:
            _filtered_distance = raw_distance
            _filtered_angle = raw_angle
            _filter_valid = True
        else:
            _filtered_distance = _lowpass(_filtered_distance, raw_distance)
            _filtered_angle = _lowpass(_filtered_angle, raw_angle)
    elif stable_flag == 0:
        _filter_valid = False
        _filtered_distance = 0
        _filtered_angle = 0

    if stable_flag == 0 or not _filter_valid:
        Line.distance = 0
        Line.angle = 0
    else:
        Line.distance = _filtered_distance
        Line.angle = _filtered_angle


def _sample_branch_points(img, flag, expected_y=None):
    """横臂逐列采样：在横臂一侧布若干 4px 宽纵向细条，每条取最上方的矮胖红色 blob 质心。

    原理：透视变换保持直线性，机身再倾斜横臂在画面中仍是直线，采样点始终落在该直线上。
    来线近似竖直，在细条内的 blob 通常较高被排除；expected_y（侧 ROI 横臂质心 cy）
    先验再排除远离横臂高度的来线碎片。
    返回 [(x, cy), ...] 全图坐标。
    """
    pts = []
    if flag == 2:  # 左转，横臂从画面左缘伸向拐点
        xs = (6, 18, 30, 42, 54, 66, 78)
    else:          # 右转，镜像
        xs = (IMG_WIDTH - 7, IMG_WIDTH - 19, IMG_WIDTH - 31,
              IMG_WIDTH - 43, IMG_WIDTH - 55, IMG_WIDTH - 67,
              IMG_WIDTH - 79)
    for x in xs:
        try:
            blobs = img.find_blobs(
                [LINE_THRESHOLD],
                roi=(x, 0, 4, 95),
                pixels_threshold=5,
                area_threshold=5,
                merge=True,
                margin=1,
            )
        except Exception:
            continue
        best_cy = -1
        for blob in blobs:
            bh = _blob_value(blob, 'h')
            bw = _blob_value(blob, 'w')
            cy = _blob_value(blob, 'cy')
            # 横臂段：矮胖（高度有限），且不能偏离侧 ROI 横臂高度先验太远；
            # 同一条内取最上方候选（横臂在来线上方）
            if not (2 <= bw and bh <= 40):
                continue
            if expected_y is not None and abs(cy - expected_y) > 40:
                continue
            if best_cy < 0 or cy < best_cy:
                best_cy = cy
        if best_cy >= 0:
            pts.append((x + 2, best_cy))
    return pts


def _fit_line_robust(pts):
    """稳健最小二乘拟合 y = c + s*x：先标准拟合，残差过大点剔除后重拟合一次。"""
    if len(pts) < 2:
        return None

    def _ls(data):
        n = float(len(data))
        sx = 0.0
        sy = 0.0
        sxx = 0.0
        sxy = 0.0
        for p in data:
            sx += p[0]
            sy += p[1]
            sxx += p[0] * p[0]
            sxy += p[0] * p[1]
        den = n * sxx - sx * sx
        if abs(den) < 1e-6:
            return None
        s = (n * sxy - sx * sy) / den
        c = (sy - s * sx) / n
        return (c, s)

    fit = _ls(pts)
    if fit is None:
        return None
    if len(pts) < 3:
        return fit
    c, s = fit
    residuals = [abs(p[1] - (c + s * p[0])) for p in pts]
    worst = 0
    for i in range(1, len(residuals)):
        if residuals[i] > residuals[worst]:
            worst = i
    if residuals[worst] > 8.0:
        trimmed = [pts[i] for i in range(len(pts)) if i != worst]
        if len(trimmed) >= 2:
            refit = _ls(trimmed)
            if refit is not None:
                return refit
    return fit


def _sample_approach_points(img, min_y=25, start_x=None):
    """来线密集采样：从画面底部沿来线向上布横向细条，遇展宽突增（横臂）停止。

    复用 _row_red_extent 测每行会线展宽：直线段展宽≈线宽（小），横臂横穿该行使展宽突增，
    检测到突增即停止采样（上方是横臂区域，避免污染拟合）。
    从下往上采样保证先采到来线部分，横臂区域不会混入。
    返回 [(cx, cy), ...] 全图坐标。
    """
    pts = []
    n_slices = 9
    max_y = IMG_HEIGHT - 6
    if min_y >= max_y:
        return pts
    step = (max_y - min_y) / float(n_slices - 1)
    last_cx = start_x if start_x is not None else IMG_CENTER_X
    spans = []

    for i in range(n_slices):
        y_center = int(max_y - i * step)
        if y_center < min_y:
            break
        ext = _row_red_extent(img, last_cx, y_center)
        if ext is None:
            break  # 来线断开或偏离预测过远
        span, left_ext, right_ext = ext
        # blob 实际中心 = 预测 x + (右延伸-左延伸)/2
        cx = int(round(last_cx + (right_ext - left_ext) / 2.0))
        pts.append((cx, y_center))
        spans.append(span)
        last_cx = cx

        # 前 3 个点建立基线（直线段最小展宽），之后遇展宽突增停止（横臂区域）
        if len(spans) >= 3:
            baseline = min(spans[:3])
            if span > max(baseline * 2.0, baseline + 25):
                break

    return pts


def _fit_line_robust_x(points):
    """迭代稳健最小二乘拟合 x = c + s*y（来线方向），反复拟合剔除离群点。

    返回 (distance, angle, slope, mean_x, mean_y, trimmed_points)。
    """
    if len(points) < 2:
        if len(points) == 1:
            cx, cy = points[0]
            distance = int(cx - IMG_CENTER_X)
            distance = max(-MAX_LINE_OFFSET, min(MAX_LINE_OFFSET, distance))
            return distance, 0, 0.0, float(cx), float(cy), points
        return 0, 0, 0.0, IMG_CENTER_X, IMG_CENTER_Y, points

    def _ls(data):
        n = float(len(data))
        sx = sum(p[0] for p in data)
        sy = sum(p[1] for p in data)
        syy = sum(p[1] * p[1] for p in data)
        sxy = sum(p[0] * p[1] for p in data)
        den = n * syy - sy * sy
        if abs(den) < 1e-6:
            return None
        s = (n * sxy - sx * sy) / den  # dx/dy
        c = (sx - s * sy) / n
        return (c, s)

    trimmed = list(points)
    for _ in range(3):  # 最多迭代剔除 3 个离群点
        if len(trimmed) < 3:
            break
        fit = _ls(trimmed)
        if fit is None:
            break
        c, s = fit
        residuals = [abs(p[0] - (c + s * p[1])) for p in trimmed]
        worst = 0
        for i in range(1, len(residuals)):
            if residuals[i] > residuals[worst]:
                worst = i
        if residuals[worst] > 8.0:
            trimmed.pop(worst)
        else:
            break

    fit = _ls(trimmed)
    if fit is None:
        return 0, 0, 0.0, IMG_CENTER_X, IMG_CENTER_Y, points
    c, s = fit

    n = float(len(trimmed))
    mean_x = sum(p[0] for p in trimmed) / n
    mean_y = sum(p[1] for p in trimmed) / n
    slope = s
    center_x = mean_x + slope * (IMG_CENTER_Y - mean_y)
    distance = int(center_x - IMG_CENTER_X)
    angle = int(math.atan(slope) * RAD_TO_ANGLE)
    distance = max(-MAX_LINE_OFFSET, min(MAX_LINE_OFFSET, distance))
    angle = max(-MAX_LINE_ANGLE, min(MAX_LINE_ANGLE, angle))
    return distance, angle, slope, mean_x, mean_y, trimmed


def _point_to_line_dist(px, py, A, B, C):
    """点 (px,py) 到直线 Ax+By+C=0 的真实距离。"""
    denom = math.sqrt(A * A + B * B)
    if denom < 1e-6:
        return 0.0
    return abs(A * px + B * py + C) / denom


def _verify_pixel_on_track(img, cx, cy):
    """在轨像素硬校验：交点邻域 7x7 必须有真实红像素，防止几何方法给出漂到赛道外的坐标。"""
    ix = int(cx)
    iy = int(cy)
    if ix < 4 or ix > IMG_WIDTH - 4 or iy < 4 or iy > IMG_HEIGHT - 4:
        return False
    try:
        blobs = img.find_blobs(
            [LINE_THRESHOLD],
            roi=(ix - 3, iy - 3, 7, 7),
            pixels_threshold=4, area_threshold=4,
        )
    except Exception:
        return False
    return len(blobs) > 0


def _row_red_extent(img, pred_x, y):
    """在像素行 y 上，以预测来线位置 pred_x 为中心测量红色横向展宽。

    返回 (span, left_ext, right_ext) 或 None。直线段：左右延伸都≈半个线宽（小）；
    拐角行：横臂单侧突然变长。
    """
    if pred_x < 0 or pred_x >= IMG_WIDTH:
        return None
    win = 40
    rx = max(0, pred_x - win)
    rw = min(IMG_WIDTH - rx, 2 * win)
    try:
        blobs = img.find_blobs(
            [LINE_THRESHOLD],
            roi=(rx, y, rw, 2),
            pixels_threshold=3, area_threshold=3,
            merge=True, margin=1,
        )
    except Exception:
        return None
    best = None
    best_dist = 1 << 30
    for blob in blobs:
        rect = _blob_value(blob, 'rect')
        x0 = rect[0]
        x1 = rect[0] + rect[2]
        if x0 <= pred_x <= x1:
            dist = 0
        else:
            dist = min(abs(pred_x - x0), abs(pred_x - x1))
        if dist < best_dist:
            best_dist = dist
            best = rect
    if best is None or best_dist > WALKER_BLOB_DIST:
        return None
    x0 = best[0]
    x1 = best[0] + best[2]
    left_ext = pred_x - x0
    right_ext = x1 - pred_x
    return (x1 - x0, left_ext, right_ext)


def _detect_cross_walker(img, slope, mean_x, mean_y):
    """行走法交点检测（旋转无关主路径，quality=1）：沿已拟合的来线逐行向上扫描，
    用"横向展宽突增"定位拐点行，完全不依赖侧 ROI 横臂。

    动机：旧方法都以 left XOR right 侧 ROI 检到横臂为前置门槛，机身大倾斜时横臂滑出
    20px 侧条 → 两侧皆 False → 所有方法同时失效。而来线拟合 x=f(y) 在倾斜时依然可靠
    （透视把直线映成直线），沿这条线走：直线段每行红宽≈线宽，到拐角横臂横穿该行使
    红宽突增，该行即拐点。对任意旋转角成立。

    返回 (flag, cross_x_abs, cross_y_abs, quality=1) 或 None。
    """
    if abs(slope) > WALKER_MAX_SLOPE:
        return None

    # 从画面下方（近处）沿来线向上（远处）扫描
    y_start = IMG_HEIGHT - 8
    spans = []
    extents = []
    ys = []
    y = y_start
    while y >= WALKER_Y_MIN:
        px = int(round(mean_x + slope * (y - mean_y)))
        ext = _row_red_extent(img, px, y)
        if ext is None:
            break  # 来线在此行断开（已越过顶端），停止
        spans.append(ext[0])
        extents.append(ext)
        ys.append(y)
        y -= WALKER_STEP

    if len(spans) < 3:
        return None

    # 基线取扫描到的最小展宽（直线段行），抗个别噪声行
    baseline = min(spans)
    threshold = max(baseline + WALKER_SPAN_DELTA, WALKER_SPAN_MIN)

    # 从近到远找第一个展宽突增行 = 拐点行
    for idx in range(len(spans)):
        if spans[idx] >= threshold:
            corner_y = ys[idx]
            left_ext = extents[idx][1]
            right_ext = extents[idx][2]
            # 横臂向哪侧延伸即为转向方向；单侧不明显则无法定方向
            if left_ext >= WALKER_ARM_MIN and left_ext > right_ext:
                flag = 2
            elif right_ext >= WALKER_ARM_MIN and right_ext > left_ext:
                flag = 3
            else:
                return None
            cross_x = int(round(mean_x + slope * (corner_y - mean_y)))
            if cross_x < 0 or cross_x >= IMG_WIDTH:
                return None
            if not _verify_pixel_on_track(img, cross_x, corner_y):
                return None
            return (flag, cross_x, int(corner_y), 1)
    return None


def _detect_cross_dual_line(img, results, slope, mean_x, mean_y, points):
    """双中线交点法（quality=0/2）：来线中心线 x=f(y) 与横臂中心线 y=g(x) 解联立方程。

    不依赖"横臂质心 cy≈拐点 y"的假设（机身倾斜时该假设误差可达 19px），
    两条线都用多点最小二乘拟合，单点噪声被平均，倾斜天然免疫。
    来线 |slope|<=0.9（约 42°）时 quality=0，>0.9 降级 quality=2（KF 给大 R 少采信），
    >1.4（约 54°）放弃，行走法接管。
    新增在轨验证 + 点到线距离双重校验，防交点飞出赛道或远交。

    返回 (flag, cross_x_abs, cross_y_abs, quality) 或 None。
    """
    global _last_branch_pts, _last_branch_fit
    _last_branch_pts = []
    _last_branch_fit = None

    left_found = results['left']['blob_flag']
    right_found = results['right']['blob_flag']
    if left_found and not right_found:
        flag = 2
        branch_cy = results['left']['cy']
    elif right_found and not left_found:
        flag = 3
        branch_cy = results['right']['cy']
    else:
        return None

    # 来线需要至少 2 个拟合点；过斜时 ROI 质心失真严重，放弃→行走法接管
    if len(points) < 2 or abs(slope) > CROSS_DUAL_HARD_SLOPE:
        return None
    quality = 0 if abs(slope) <= CROSS_DUAL_MAX_SLOPE else 2

    branch_pts = _sample_branch_points(img, flag, branch_cy)
    _last_branch_pts = branch_pts
    if len(branch_pts) < 2:
        return None
    fit = _fit_line_robust(branch_pts)
    if fit is None:
        return None
    c2, s2 = fit
    _last_branch_fit = fit

    # 解联立：x = mean_x + slope*(y - mean_y)，y = c2 + s2*x
    # 代入得：x*(1 - slope*s2) = mean_x + slope*(c2 - mean_y)
    den = 1.0 - slope * s2
    if abs(den) < 0.3:  # 两线近平行，交点病态（非 L 型拐角）
        return None
    cross_x = (mean_x + slope * (c2 - mean_y)) / den
    cross_y = c2 + s2 * cross_x

    if cross_x < 2 or cross_x > IMG_WIDTH - 3:
        return None
    if cross_y < 2 or cross_y > IMG_HEIGHT - 3:
        return None

    # 交点必须落在横臂采样范围附近，防止沿直线无限外推爆掉
    xs = [p[0] for p in branch_pts]
    min_sx = min(xs)
    max_sx = max(xs)
    if flag == 2:
        # 左转：拐点在横臂右端，允许超出最大采样列一段距离
        if cross_x < min_sx - 12 or cross_x > max_sx + 32:
            return None
    else:
        if cross_x > max_sx + 12 or cross_x < min_sx - 32:
            return None

    # 拐点是来线顶端：交点 y 不应明显低于最靠上的来线采样点
    ys = [p[1] for p in points]
    if cross_y > min(ys) + 12:
        return None

    # 在轨验证（7x7 邻域）
    if not _verify_pixel_on_track(img, cross_x, cross_y):
        return None

    # 点到线距离校验：交点到两条拟合中心线的真实距离均须 < 阈值，
    # 确保是线的真实端点相接而非无限延长线的远处交叉
    dist_appr = _point_to_line_dist(
        cross_x, cross_y,
        1.0, -slope, slope * mean_y - mean_x
    )
    dist_branch = _point_to_line_dist(
        cross_x, cross_y,
        s2, -1.0, c2
    )
    if dist_appr > CROSS_LINE_DIST_MAX or dist_branch > CROSS_LINE_DIST_MAX:
        return None

    return (flag, int(cross_x), int(cross_y), quality)


def _detect_cross_precise(img, results, slope=0, mean_x=80, mean_y=60, points=None):
    """交点检测调度：双中线（精度） + 行走法（旋转无关主路径）。

    1. 双中线拟合法：仅当侧 ROI 恰好检到横臂时启用，精度最高（横臂中心线多点拟合），
       带在轨+距离验证；来线过斜时降级 quality=2；侧 ROI 丢失时自动跳过。
    2. 行走法：不依赖侧 ROI，沿来线逐行测横向展宽定位拐点，任意旋转角可用，
       是倾斜/转弯中段的主路径。直线段（三横向 ROI 全检到）预门控跳过省算力。

    返回 (flag, cross_x_abs, cross_y_abs, quality)，quality=0 双中线/
    1 行走法/2 双中线降级，供交点 KF 按方法质量取量测噪声 R。
    """
    if points is None:
        points = []

    # 方法 0/2：双中线拟合法（侧 ROI 可用时的高精度增强）
    dual_result = _detect_cross_dual_line(
        img, results, slope, mean_x, mean_y, points
    )
    if dual_result is not None:
        return dual_result

    # 方法 1：行走法（旋转无关主路径，不依赖侧 ROI）
    # 廉价预门控：三条横向 ROI 全部检到 = 干净的直线巡航段，拐角不在视野内，
    # 跳过行走法避免每帧数十次逐行 find_blobs 拖垮帧率
    horiz_found = (int(results['up']['blob_flag'])
                   + int(results['middle']['blob_flag'])
                   + int(results['down']['blob_flag']))
    if horiz_found < 3:
        walker_result = _detect_cross_walker(img, slope, mean_x, mean_y)
        if walker_result is not None:
            return walker_result

    return None


def _draw_line_results(img, results, slope, mean_x, mean_y, points):
    for roi_name in ROIS.keys():
        result = results[roi_name]
        if result['blob_flag']:
            img.draw_rectangle(result['rect'], color=(0, 255, 255))

    # 双中线法可视化：横臂采样点（蓝）与拟合线（蓝）、来线拟合（绿）
    if _last_branch_pts:
        for pt in _last_branch_pts:
            img.draw_cross(pt[0], pt[1], color=(0, 0, 255), size=3)
    if _last_branch_fit is not None and len(_last_branch_pts) >= 2:
        bc, bs = _last_branch_fit
        xs = [p[0] for p in _last_branch_pts]
        x0 = max(0, min(xs) - 6)
        x1 = min(IMG_WIDTH - 1, max(xs) + 6)
        img.draw_line(
            int(x0), int(bc + bs * x0), int(x1), int(bc + bs * x1),
            color=(0, 0, 255), thickness=1,
        )

    if len(points) >= 2:
        x_top = int(mean_x + slope * (0 - mean_y))
        x_bottom = int(mean_x + slope * ((IMG_HEIGHT - 1) - mean_y))
        x_top = max(0, min(IMG_WIDTH - 1, x_top))
        x_bottom = max(0, min(IMG_WIDTH - 1, x_bottom))
        img.draw_line(x_top, 0, x_bottom, IMG_HEIGHT - 1, color=(0, 255, 0), thickness=2)

    if Line.cross_flag:
        # Line.cross_x/cross_y 是中心为原点的坐标，转回全图坐标绘图
        cx_abs = Line.cross_x + IMG_CENTER_X
        cy_abs = Line.cross_y + IMG_CENTER_Y
        img.draw_cross(cx_abs, cy_abs, color=(255, 0, 0), size=5)
        # 交点旁绘制坐标文字（与内部计算/串口打印/飞控接收完全一致，下为正）
        coord_str = '(%d,%d)' % (Line.cross_x, Line.cross_y)
        tx = min(cx_abs + 6, IMG_WIDTH - len(coord_str) * 8 - 2)
        ty = max(cy_abs - 10, 0)
        img.draw_string(tx, ty, coord_str, color=(255, 0, 0))

    turn_type = 'N'
    if Line.flag == 1:
        turn_type = 'S'
    elif Line.flag == 2:
        turn_type = 'L'
    elif Line.flag == 3:
        turn_type = 'R'
    cross_mark = ' X' if Line.cross_flag else ''
    img.draw_string(
        0, 0,
        '%s D%d A%d%s' % (turn_type, Line.distance, Line.angle, cross_mark),
        color=(255, 255, 255),
    )

    green_total, yellow_total = CornerStatistics.Statistics.totals()
    img.draw_string(
        0, 10,
        'C%d/%d G%d Y%d'
        % (
            CornerStatistics.Statistics.corner_count,
            CornerStatistics.TARGET_CORNERS,
            green_total,
            yellow_total,
        ),
        color=(0, 255, 255),
    )


def LineCheck():
    """拍摄一次，完成巡线和识球，并按下位机的 29 字节定长帧发送。

    交点检测用双中线拟合法（来线中心线 × 横臂中心线解交点），倾斜天然免疫；
    交点输出用恒速卡尔曼滤波跟踪：量测按方法质量加权，野值门控剔除，
    丢失期用速度状态外推（代替旧冻结保持），保证 crossx/crossy 稳定且滞后小。
    """
    global img, _debug_count, _cross_lost_count, _cross_update_streak
    global _cross_confirmed, _cross_last_tick
    img = sensor.snapshot()

    # 实测帧间隔：OpenMV 帧率 15~30FPS 不均，KF 的 dt 必须用真实值
    now_ms = time.ticks_ms()
    if _cross_last_tick:
        dt = time.ticks_diff(now_ms, _cross_last_tick) / 1000.0
    else:
        dt = 0.05
    dt = max(0.02, min(0.2, dt))
    _cross_last_tick = now_ms

    # 检测阶段严禁绘图：绘制内容会真实改变 RGB565 像素
    results = _find_blobs_in_rois(img)

    # 来线密集采样 + 迭代稳健拟合：从画面底部沿来线向上采样，
    # 复用 _row_red_extent 测展宽，遇展宽突增（横臂）自动停止，只采来线部分的点
    branch_cy = None
    if results['left']['blob_flag'] != results['right']['blob_flag']:
        branch_cy = results['left']['cy'] if results['left']['blob_flag'] else results['right']['cy']
    approach_min_y = (branch_cy + 15) if branch_cy is not None else 25
    start_x = results['down']['cx'] if results['down']['blob_flag'] else None
    approach_pts = _sample_approach_points(img, approach_min_y, start_x)
    raw_distance, raw_angle, slope, mean_x, mean_y, points = _fit_line_robust_x(approach_pts)

    # 交点独立计算：优先用拟合方程，多级兜底
    cross_result = _detect_cross_precise(img, results, slope, mean_x, mean_y, points)

    # 状态判定：交点优先，ROI 逻辑回退
    raw_flag = _raw_line_flag(results, cross_result)
    Line.flag = _stabilize_flag(raw_flag)

    _update_measurements(
        raw_flag, Line.flag, raw_distance, raw_angle, len(points) > 0
    )

    # ===== 交点 KF 跟踪 =====
    # 有量测 → predict+update（质量差的方法给大 R，自动少采信）；
    # innovation 超门控（跳变/方法切换）→ 本帧量测弃用，等效丢失只外推；
    # 连续 N 帧成功更新才确认输出（滤毛刺）；
    # 丢失/被拒 → 仅 predict 沿速度外推，超 CROSS_HOLD_FRAMES 清除跟踪
    kf_ok = False
    predicted = False  # 本帧是否已 predict 过（门控拒绝时避免丢失分支二次外推）
    if cross_result is not None:
        _, cx_abs, cy_abs, quality = cross_result
        rx, ry = CROSS_R_TABLE[quality]
        if not _cross_kf_x.valid:
            # 首帧播种：位置=量测，速度不确定度给足
            _cross_kf_x.reseed(cx_abs, rx, CROSS_INIT_V_UNC)
            _cross_kf_y.reseed(cy_abs, ry, CROSS_INIT_V_UNC)
            _cross_update_streak = 1
            kf_ok = True
        else:
            _cross_kf_x.predict(dt, CROSS_Q)
            _cross_kf_y.predict(dt, CROSS_Q)
            predicted = True
            # 门控带绝对下限 CROSS_GATE_MIN_PX：防止 P 收敛后门过紧，
            # 把正常接近拐点的大 innovation 误杀
            jump_x = (_cross_kf_x.gate_reject(cx_abs, rx, CROSS_GATE_SIGMA)
                      and abs(cx_abs - _cross_kf_x.s0) > CROSS_GATE_MIN_PX)
            jump_y = (_cross_kf_y.gate_reject(cy_abs, ry, CROSS_GATE_SIGMA)
                      and abs(cy_abs - _cross_kf_y.s0) > CROSS_GATE_MIN_PX)
            if jump_x or jump_y:
                # 跳变：撤销确认，量测弃用，本帧按丢失外推处理（已 predict）；
                # 若目标真的变了，持续跳变会耗尽保持帧数→清除→重新播种
                _cross_confirmed = False
                _cross_update_streak = 0
            else:
                _cross_kf_x.update(cx_abs, rx)
                _cross_kf_y.update(cy_abs, ry)
                _cross_kf_x.clamp_speed(CROSS_MAX_SPEED)
                _cross_kf_y.clamp_speed(CROSS_MAX_SPEED)
                _cross_update_streak += 1
                kf_ok = True

    if kf_ok:
        _cross_lost_count = 0
        if _cross_update_streak >= CROSS_CONFIRM_FRAMES:
            _cross_confirmed = True
    else:
        # 无量测（或被门控拒绝）：已确认跟踪继续纯外推若干帧
        _cross_lost_count += 1
        if _cross_kf_x.valid and _cross_lost_count <= CROSS_HOLD_FRAMES:
            if not predicted:
                _cross_kf_x.predict(dt, CROSS_Q)
                _cross_kf_y.predict(dt, CROSS_Q)
                _cross_kf_x.clamp_speed(CROSS_MAX_SPEED)
                _cross_kf_y.clamp_speed(CROSS_MAX_SPEED)
        else:
            _cross_confirmed = False
            _cross_kf_x.valid = False
            _cross_kf_y.valid = False
            _cross_update_streak = 0

    if _cross_confirmed and _cross_kf_x.valid:
        Line.cross_flag = 1
        # KF 输出是全图绝对坐标，转为以画面中心为原点
        Line.cross_x = int(_cross_kf_x.s0) - IMG_CENTER_X
        Line.cross_y = int(_cross_kf_y.s0) - IMG_CENTER_Y
    else:
        # 确认期内不输出，避免单帧毛刺触发飞控锁存
        Line.cross_flag = 0
        Line.cross_x = 0
        Line.cross_y = 0

    # 复用完全未绘制的同一帧，避免线框/文字成为红绿黄候选
    ball = Colortracking.Color(img=img, send=False, draw=False)

    # 稳定转角状态代表飞机进入拐角。状态机完成观察、去重和累计
    corner_signal = Line.flag in (2, 3) or Line.cross_flag == 1
    CornerStatistics.Statistics.update(
        corner_signal,
        ball.green_count,
        ball.yellow_count,
    )
    total_green, total_yellow = CornerStatistics.Statistics.totals()

    # 所有识别和统计完成后才绘图；非调试模式完全跳过，提升帧率
    if Message.Ctr.IsDebug == 1:
        _draw_line_results(img, results, slope, mean_x, mean_y, points)
        Colortracking.DrawResults(img)
        _debug_count += 1
        if _debug_count >= 10:
            _debug_count = 0
            _status_name = '无'
            if Line.flag == 1:
                _status_name = '直线'
            elif Line.flag == 2:
                _status_name = '左转'
            elif Line.flag == 3:
                _status_name = '右转'
            _cross_name = '有效' if Line.cross_flag == 1 else '无'
            print(
                '状态:%s 角度:%d° 偏移:%dpx | 交点:%s X:%d Y:%d'
                % (
                    _status_name,
                    Line.angle,
                    Line.distance,
                    _cross_name,
                    Line.cross_x,
                    Line.cross_y,  # 与内部计算/屏幕显示/飞控接收完全一致，下为正
                )
            )

    # 红色用于巡线，不参与球计数。主目标优先级：绿 > 黄
    ball_color = 0
    ball_cx = 0
    ball_cy = 0
    if ball.green_count > 0:
        ball_color = 2
        ball_cx = ball.green_x
        ball_cy = ball.green_y
    elif ball.yellow_count > 0:
        ball_color = 3
        ball_cx = ball.yellow_x
        ball_cy = ball.yellow_y

    # ball_status 位域同时携带任务进度与当前球色
    ball_status = CornerStatistics.Statistics.status_byte(ball_color)

    # delta_x = line.distance（飞控端未使用，保留协议兼容），delta_y 固定 0
    delta_x = Line.distance
    delta_y = 0

    Message.UartSendData(Message.FusionDataPack(
        Line.flag, Line.angle, Line.distance,
        Line.cross_flag, Line.cross_x, Line.cross_y,
        # STM32 在每个拐角自行取 5 帧最大值并累加，这里必须发当前帧计数。
        0, ball.green_count, ball.yellow_count,
        ball_status, ball_cx, ball_cy,
        Message.Ctr.T_ms,
        delta_x, delta_y
    ))
    return Line.flag


def SendMissionSummary(repeats=5):
    """飞行结束时重复发送最终绿/黄累计结果，提高上位机收到概率。"""
    green, yellow = CornerStatistics.Statistics.totals()
    green_u8 = max(0, min(255, int(green)))
    yellow_u8 = max(0, min(255, int(yellow)))
    status = CornerStatistics.Statistics.status_byte(0)
    frame = Message.FusionDataPack(
        0, 0, 0,
        0, 0, 0,
        0, green_u8, yellow_u8,
        status, 0, 0,
        Message.Ctr.T_ms,
        0, 0
    )
    for _ in range(max(1, repeats)):
        Message.UartSendData(frame)
        try:
            time.sleep_ms(20)
        except Exception:
            pass
    print(
        'mission summary sent: corners=%d green=%d yellow=%d total=%d'
        % (
            CornerStatistics.Statistics.corner_count,
            green,
            yellow,
            green + yellow,
        )
    )


#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
