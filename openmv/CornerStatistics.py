"""正方形红线巡航任务的四拐角小球累计状态机。

本模块不依赖 OpenMV 图像 API，可在 PC 端直接做状态机测试。每个拐角只入账一次；
离开当前拐角若干帧后，才允许统计下一个拐角。
"""


TARGET_CORNERS = 4

# 连续检测到转角多少帧才进入拐角，过滤偶发误判
ENTER_CONFIRM_FRAMES = 2
# 连续离开转角多少帧才开放下一次统计
LEAVE_CONFIRM_FRAMES = 5
# 同一组球数至少稳定多少帧，才作为可信候选
COUNT_STABLE_FRAMES = 2
# 进入拐角后至少观察多少帧，避免刚看到部分球就立即入账
MIN_OBSERVE_FRAMES = 6
# 可信最大值连续多少帧不再增加后入账
SETTLE_FRAMES = 4
# 防止飞机一直保持转角状态而无法完成当前拐角
MAX_OBSERVE_FRAMES = 24


def _u8(value):
    value = int(value)
    if value < 0:
        return 0
    if value > 255:
        return 255
    return value


class CornerStatistics(object):
    """累计绿球和黄球，并管理四个拐角的去重门闩。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.green_total = 0
        self.yellow_total = 0
        self.corner_count = 0
        self.finished = False
        self.corner_history = []

        self.in_corner = False
        self.corner_committed = False
        self.just_entered = False
        self.just_committed = False
        self.just_left = False

        self._turn_frames = 0
        self._leave_frames = 0
        self._observe_frames = 0
        self._settle_frames = 0
        self._candidate = None
        self._candidate_frames = 0
        self._best_green = 0
        self._best_yellow = 0
        self._raw_green = 0
        self._raw_yellow = 0

    def _start_corner(self):
        self.in_corner = True
        self.corner_committed = False
        self.just_entered = True
        self._leave_frames = 0
        self._observe_frames = 0
        self._settle_frames = 0
        self._candidate = None
        self._candidate_frames = 0
        self._best_green = 0
        self._best_yellow = 0
        self._raw_green = 0
        self._raw_yellow = 0

    def _observe(self, green, yellow):
        green = _u8(green)
        yellow = _u8(yellow)
        self._observe_frames += 1
        self._settle_frames += 1

        # 保留原始逐色最大值，只在飞机很快离开、来不及稳定时作为回退
        if green > self._raw_green:
            self._raw_green = green
        if yellow > self._raw_yellow:
            self._raw_yellow = yellow

        counts = (green, yellow)
        if counts == self._candidate:
            self._candidate_frames += 1
        else:
            self._candidate = counts
            self._candidate_frames = 1

        improved = False
        if self._candidate_frames >= COUNT_STABLE_FRAMES:
            if green > self._best_green:
                self._best_green = green
                improved = True
            if yellow > self._best_yellow:
                self._best_yellow = yellow
                improved = True
        if improved:
            self._settle_frames = 0

    def _has_observation(self):
        return (
            self._best_green + self._best_yellow > 0
            or self._raw_green + self._raw_yellow > 0
        )

    def _commit(self, allow_raw_fallback=False):
        if self.corner_committed or self.finished:
            return False

        green = self._best_green
        yellow = self._best_yellow
        if green + yellow <= 0 and allow_raw_fallback:
            green = self._raw_green
            yellow = self._raw_yellow
        if green + yellow <= 0:
            return False

        self.green_total = _u8(self.green_total + green)
        self.yellow_total = _u8(self.yellow_total + yellow)
        self.corner_count += 1
        self.corner_history.append((green, yellow))
        self.corner_committed = True
        self.just_committed = True

        if self.corner_count >= TARGET_CORNERS:
            self.finished = True
        return True

    def update(self, is_corner, green, yellow):
        """输入当前帧的转角标志和球数，返回本帧是否刚完成一次入账。"""
        self.just_entered = False
        self.just_committed = False
        self.just_left = False

        if self.finished:
            return False

        if not self.in_corner:
            if is_corner:
                self._turn_frames += 1
                if self._turn_frames >= ENTER_CONFIRM_FRAMES:
                    self._start_corner()
                    self._observe(green, yellow)
            else:
                self._turn_frames = 0
            return self.just_committed

        # 已经进入拐角。转角信号短时抖动时仍继续观察当前画面
        self._observe(green, yellow)
        if is_corner:
            self._leave_frames = 0
        else:
            self._leave_frames += 1

        if not self.corner_committed and self._has_observation():
            settled = (
                self._observe_frames >= MIN_OBSERVE_FRAMES
                and self._settle_frames >= SETTLE_FRAMES
            )
            timed_out = self._observe_frames >= MAX_OBSERVE_FRAMES
            if settled or timed_out:
                self._commit(allow_raw_fallback=timed_out)

        if self._leave_frames >= LEAVE_CONFIRM_FRAMES:
            # 飞机很快通过拐角时，在离开沿途用已见到的最佳结果兜底入账
            if not self.corner_committed:
                self._commit(allow_raw_fallback=True)
            self.in_corner = False
            self.corner_committed = False
            self.just_left = True
            self._turn_frames = 0
            self._leave_frames = 0

        return self.just_committed

    def finish(self):
        """收到飞行结束命令时锁定结果，并尽量保存尚未入账的当前拐角。"""
        self.just_committed = False
        if not self.finished and self.in_corner and not self.corner_committed:
            self._commit(allow_raw_fallback=True)
        self.finished = True
        return self.totals()

    def totals(self):
        return self.green_total, self.yellow_total

    def status_byte(self, current_color=0):
        """bit7=结束，bit6..4=已统计拐角数，bit3..0=当前球色。"""
        value = (int(current_color) & 0x0F) | ((self.corner_count & 0x07) << 4)
        if self.finished:
            value |= 0x80
        return value


Statistics = CornerStatistics()
