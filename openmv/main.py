#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
"""OpenMV 主入口：摄像头初始化 + 曝光锁定 + 主循环巡线识别。

功能：红色正方形巡线 + 四拐角绿/黄球累计，结果按下位机 29 字节定长帧发送。
"""
import sensor, time
import AnoMessage as Message
import LineFollowing
import CornerStatistics

# 曝光校准参数：目标 L 通道上四分位越低画面越暗，颜色越不容易被高光冲淡
EXPOSURE_TARGET_L_UQ = 78
EXPOSURE_MIN_US = 800
EXPOSURE_MAX_US = 10000
EXPOSURE_INITIAL_SCALE = 0.45
EXPOSURE_CAL_ROI = (20, 22, 120, 82)
EXPOSURE_CAL_TIMES = 5


def _stat_value(statistics, name):
    """兼容 OpenMV 新版属性接口和旧版方法接口。"""
    value = getattr(statistics, name)
    return value() if callable(value) else value


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _lock_camera_parameters():
    """锁定增益、白平衡，并根据中央区域亮度自动压低、锁定曝光。

    增益和白平衡固定后 LAB 阈值才不会随帧漂移；自动曝光容易被黑边/暗角误导，
    先降到自动值的 45%，再按中央白底迭代收敛。
    """
    try:
        gain_db = sensor.get_gain_db()
        rgb_gain_db = sensor.get_rgb_gain_db()
        auto_exposure_us = sensor.get_exposure_us()

        sensor.set_auto_gain(False, gain_db=gain_db)
        sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gain_db)

        exposure_us = int(auto_exposure_us * EXPOSURE_INITIAL_SCALE)
        exposure_us = _clamp(exposure_us, EXPOSURE_MIN_US, EXPOSURE_MAX_US)
        sensor.set_auto_exposure(False, exposure_us=exposure_us)

        for _ in range(EXPOSURE_CAL_TIMES):
            sensor.skip_frames(time=120)
            calibration_img = sensor.snapshot()
            statistics = calibration_img.get_statistics(roi=EXPOSURE_CAL_ROI)
            light_uq = _stat_value(statistics, 'l_uq')

            if light_uq > EXPOSURE_TARGET_L_UQ + 3:
                # 高亮过曝时快速下降，单次最多降 35% 避免突变
                factor = EXPOSURE_TARGET_L_UQ / float(max(1, light_uq))
                factor = _clamp(factor * 0.92, 0.65, 0.88)
                new_exposure_us = int(exposure_us * factor)
            elif light_uq < EXPOSURE_TARGET_L_UQ - 12:
                # 只缓慢增亮，优先保证红/黄/绿颜色不被冲白
                new_exposure_us = int(exposure_us * 1.12)
            else:
                break

            new_exposure_us = _clamp(new_exposure_us, EXPOSURE_MIN_US, EXPOSURE_MAX_US)
            if new_exposure_us == exposure_us:
                break
            exposure_us = new_exposure_us
            sensor.set_auto_exposure(False, exposure_us=exposure_us)

        return exposure_us
    except Exception:
        # 旧固件回退：固定低增益、白平衡和 5000us 曝光
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        try:
            sensor.set_auto_exposure(False, exposure_us=5000)
        except Exception:
            sensor.set_auto_exposure(False)
        return 5000


# 初始化镜头：先让自动参数收敛，再锁定增益、白平衡和曝光
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)  # 160x120
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.set_auto_whitebal(True)
sensor.skip_frames(time=2500)

_exposure_us = _lock_camera_parameters()
sensor.skip_frames(time=500)
clock = time.clock()

if Message.Ctr.IsDebug == 1:
    print('camera exposure_us', _exposure_us)

# 上电默认进入一次新的四拐角任务，起飞所在拐角可作为第 1 个拐角统计
LineFollowing.ResetMission()

# 主循环
while(True):
    clock.tick()
    # 读取飞控下发的工作模式命令（mode=0 飞行结束，1~5 开始/继续任务）
    Message.UartReadBuffer()

    if Message.Ctr.ModeChanged:
        previous_mode = Message.Ctr.PreviousWorkMode
        current_mode = Message.Ctr.WorkMode
        Message.Ctr.ModeChanged = False

        if current_mode == 0:
            green, yellow = CornerStatistics.Statistics.finish()
            LineFollowing.SendMissionSummary()
            if Message.Ctr.IsDebug == 1:
                print(
                    'flight finished: corners=%d green=%d yellow=%d total=%d'
                    % (
                        CornerStatistics.Statistics.corner_count,
                        green,
                        yellow,
                        green + yellow,
                    )
                )
        elif previous_mode == 0:
            LineFollowing.ResetMission()

    # 结束状态不再拍照识别，避免保存汇总后继续累计
    if Message.Ctr.WorkMode == 0:
        try:
            time.sleep_ms(20)
        except Exception:
            pass
        continue

    # 巡线 + 识球 + 打包发送一帧 STM32 定长数据
    LineFollowing.LineCheck()
    # 每帧更新帧周期（通信需要），不打印 fps/T_ms 避免盖住有效调试信息
    fps = int(clock.fps())
    if fps > 0:
        Message.Ctr.T_ms = min(255, int(1000 / fps))

#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
