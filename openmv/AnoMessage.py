#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
"""OpenMV 与 STM32 下位机的定长帧通信。

帧格式：AA AF 30 17 + 23 字节载荷 + sum1 + sum2（共 29 字节），双校验。
下位机 Drv_Uart.c 的 OpenMV_GetOneByte() 按此格式逐字节解析。
文件特意命名为 AnoMessage.py，避免 OpenMV 固件中的 Message 同名模块被优先导入。
"""
from pyb import UART
uart = UART(3, 500000)  # UART3，波特率 500000


class Receive(object):
    uart_buf = []
    _data_len = 0
    _data_cnt = 0
    state = 0


R = Receive()


class Ctrl(object):
    """工作模式控制：
    WorkMode=0  飞行结束/停止视觉任务（保存并返回累计小球数量）
    WorkMode=1  寻点模式
    WorkMode=2  寻线模式（直线 + 转角，本项目默认）
    WorkMode=3  颜色识别模式
    WorkMode=4  识别二维码模式
    WorkMode=5  拍照模式
    """
    WorkMode = 2
    PreviousWorkMode = 2
    ModeChanged = False
    IsDebug = 1      # 非调试模式关闭图形显示以提高帧率
    T_ms = 0


Ctr = Ctrl()


def UartSendData(Data):
    uart.write(Data)


def ReceiveAnl(data_buf, num):
    """飞控 → OpenMV 命令解析：和校验通过后，0x06 功能字设置工作模式。"""
    sum = 0
    i = 0
    while i < (num - 1):
        sum = sum + data_buf[i]
        i = i + 1
    sum = sum % 256
    if sum != data_buf[num - 1]:
        return
    if data_buf[4] == 0x06:
        new_mode = data_buf[5]
        if new_mode != Ctr.WorkMode:
            Ctr.PreviousWorkMode = Ctr.WorkMode
            Ctr.WorkMode = new_mode
            Ctr.ModeChanged = True


def ReceivePrepare(data):
    """飞控 → OpenMV 帧状态机：AA AF 05 01 06 + 模式字节 + 校验。"""
    if R.state == 0:
        if data == 0xAA:
            R.uart_buf.append(data)
            R.state = 1
        else:
            R.state = 0
    elif R.state == 1:
        if data == 0xAF:
            R.uart_buf.append(data)
            R.state = 2
        else:
            R.state = 0
    elif R.state == 2:
        if data == 0x05:
            R.uart_buf.append(data)
            R.state = 3
        else:
            R.state = 0
    elif R.state == 3:
        if data == 0x01:
            R.state = 4
            R.uart_buf.append(data)
        else:
            R.state = 0
    elif R.state == 4:
        if data == 0x06:
            R.state = 5
            R.uart_buf.append(data)
            R._data_len = data
        else:
            R.state = 0
    elif R.state == 5:
        if 0 <= data <= 5:
            R.uart_buf.append(data)
            R.state = 6
        else:
            R.state = 0
    elif R.state == 6:
        R.state = 0
        R.uart_buf.append(data)
        ReceiveAnl(R.uart_buf, 7)
        R.uart_buf = []
    else:
        R.state = 0


def UartReadBuffer():
    i = 0
    Buffer_size = uart.any()
    while i < Buffer_size:
        ReceivePrepare(uart.readchar())
        i = i + 1


#============================================================
# 23 字节载荷布局（帧头 AA AF 30 17 之后）。
# 下列偏移严格对应 STM32 Drv_Uart.c::OpenMV_DataAnl() 的 data[4..26]：
#   [0]     line_state     u8      0=丢线，1=检测到线
#   [1]     next_direction u8      0=无，2=左转，3=右转
#   [2-4]   reserved       u8[3]   保留
#   [5]     crossflag      u8      交点有效标志
#   [6-7]   angle          s16小端 来线角度
#   [8-13]  reserved       u8[6]   保留
#   [14-15] crossx         s16小端 交点 x，右为正
#   [16-17] crossy         s16小端 交点 y，下为正
#   [18-19] reserved       u8[2]   保留
#   [20]    red            u8      当前帧红球数
#   [21]    blue           u8      当前帧蓝球数（本识别程序将黄球映射到此通道）
#   [22]    green          u8      当前帧绿球数
#============================================================
def FusionDataPack(flag, angle, distance, crossflag, crossx, crossy,
                   red, green, yellow, ball_status, ball_cx, ball_cy,
                   T_ms, delta_x, delta_y):
    payload = bytearray(23)

    # 识别层的 flag=2/3 同时表示“有线”和“转向”；在通信层拆成
    # STM32 控制逻辑需要的两个字段，不改变巡线判定逻辑。
    payload[0] = 1 if flag in (1, 2, 3) else 0
    payload[1] = flag if flag in (2, 3) else 0
    payload[5] = crossflag & 0xFF
    payload[6] = angle & 0xFF
    payload[7] = (angle >> 8) & 0xFF
    payload[14] = crossx & 0xFF
    payload[15] = (crossx >> 8) & 0xFF
    payload[16] = crossy & 0xFF
    payload[17] = (crossy >> 8) & 0xFF

    # 下位机的颜色顺序固定为 red/blue/green。当前识别逻辑检测的是
    # green/yellow，因此仅在通信边界将 yellow 放入下位机 blue 通道。
    payload[20] = red & 0xFF
    payload[21] = yellow & 0xFF
    payload[22] = green & 0xFF

    # 完整帧：AA AF 30 17 + 23 字节载荷 + sum1 + sum2
    frame = bytearray(29)
    frame[0] = 0xAA
    frame[1] = 0xAF
    frame[2] = 0x30
    frame[3] = 0x17
    for i in range(23):
        frame[4 + i] = payload[i]

    # 双校验：sum1 累加 data[0..26]，sum2 累加 sum1
    sum1 = 0
    sum2 = 0
    for i in range(27):
        sum1 = (sum1 + frame[i]) & 0xFF
        sum2 = (sum2 + sum1) & 0xFF
    frame[27] = sum1
    frame[28] = sum2

    return frame

#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
