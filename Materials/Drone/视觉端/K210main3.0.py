# MaixPy (K210) - YOLO2 + UART2（IO_6=RX, IO_8=TX）
import gc
import time
import image
import lcd
import sensor
from maix import KPU
from machine import UART
from fpioa_manager import fm

# ---------------- LCD & Camera ----------------
lcd.init()
lcd.clear(lcd.RED)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=1000)

# ---------------- UART2 映射与初始化 ----------------
# binding UART2 IO:6->RX, 8->TX
fm.register(6, fm.fpioa.UART2_RX)
fm.register(8, fm.fpioa.UART2_TX)

# 115200, 8N1
yb_uart = UART(UART.UART2, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

# ---------------- YOLO2 Model ----------------
labels = ["1", "2", "3", "4", "5"]
# 注意：anchor、模型路径请按你的实际模型调整
anchor = (0.91, 1.05, 1.58, 0.63, 1.29, 0.87, 1.67, 1.69, 2.77, 1.03)

kpu = KPU()
kpu.load_kmodel("/sd/det4.kmodel")  # 修改为你的 .kmodel 路径
kpu.init_yolo2(
    anchor,
    anchor_num=int(len(anchor) / 2),
    img_w=320, img_h=240,
    net_w=320, net_h=240,
    layer_w=10, layer_h=8,
    threshold=0.4, nms_value=0.3,
    classes=len(labels),
)

# ---------------- 数据结构与协议 ----------------
class TargetInfo:
    def __init__(self):
        self.x = 0              # int16_t（占位，不用）
        self.y = 0              # int16_t（占位，不用）
        self.pixel = 0          # uint16_t（占位，不用）
        self.flag = 0           # uint8_t（是否检测到至少一个标签）
        self.state = 0          # uint8_t（保留）
        # 五个字段重定义为标签 1..5 的状态：(exist<<8) | count
        self.angle = 0          # "1"
        self.distance = 0       # "2"
        self.apriltag_id = 0    # "3"
        self.img_width = 0      # "4"
        self.img_height = 0     # "5"

        self.fps = 0            # uint8_t（可填充为平均 fps 或 0）
        self.reserved1 = 0
        self.reserved2 = 0
        self.reserved3 = 0
        self.reserved4 = 0
        self.range_sensor1 = 0  # uint16_t（保留）
        self.range_sensor2 = 0
        self.range_sensor3 = 0
        self.range_sensor4 = 0
        self.camera_id = 0x01   # uint8_t（摄像头ID固定为0x01）
        self.reserved1_u32 = 0  # 4×uint32 保留
        self.reserved2_u32 = 0
        self.reserved3_u32 = 0
        self.reserved4_u32 = 0

target = TargetInfo()

# ======= 关键：mode 始终为 7（0b00000111），不被串口解析修改 =======
MODE_CONST = 0x07
BIT_TASK = 0           # 以 mode 的某一位作为触发开关，这里用 bit0
N_FRAMES = 20          # 触发一次，连续处理 20 帧

def package_data(mode_val=MODE_CONST):
    """FF FC | (0xA0+mode) | len | payload | checksum"""
    data = [
        0xFF, 0xFC,
        (0xA0 + (mode_val & 0xFF)) & 0xFF,
        0x00,  # len 占位

        (target.x >> 8) & 0xFF, target.x & 0xFF,
        (target.y >> 8) & 0xFF, target.y & 0xFF,
        (target.pixel >> 8) & 0xFF, target.pixel & 0xFF,

        target.flag & 0xFF,
        target.state & 0xFF,

        (target.angle >> 8) & 0xFF,      target.angle & 0xFF,        # "1"
        (target.distance >> 8) & 0xFF,   target.distance & 0xFF,     # "2"
        (target.apriltag_id >> 8) & 0xFF, target.apriltag_id & 0xFF, # "3"
        (target.img_width >> 8) & 0xFF,  target.img_width & 0xFF,    # "4"
        (target.img_height >> 8) & 0xFF, target.img_height & 0xFF,   # "5"

        target.fps & 0xFF,
        target.reserved1 & 0xFF,
        target.reserved2 & 0xFF,
        target.reserved3 & 0xFF,
        target.reserved4 & 0xFF,

        (target.range_sensor1 >> 8) & 0xFF, target.range_sensor1 & 0xFF,
        (target.range_sensor2 >> 8) & 0xFF, target.range_sensor2 & 0xFF,
        (target.range_sensor3 >> 8) & 0xFF, target.range_sensor3 & 0xFF,
        (target.range_sensor4 >> 8) & 0xFF, target.range_sensor4 & 0xFF,

        target.camera_id & 0xFF,

        (target.reserved1_u32 >> 24) & 0xFF, (target.reserved1_u32 >> 16) & 0xFF,
        (target.reserved1_u32 >> 8)  & 0xFF,  target.reserved1_u32 & 0xFF,
        (target.reserved2_u32 >> 24) & 0xFF, (target.reserved2_u32 >> 16) & 0xFF,
        (target.reserved2_u32 >> 8)  & 0xFF,  target.reserved2_u32 & 0xFF,
        (target.reserved3_u32 >> 24) & 0xFF, (target.reserved3_u32 >> 16) & 0xFF,
        (target.reserved3_u32 >> 8)  & 0xFF,  target.reserved3_u32 & 0xFF,
        (target.reserved4_u32 >> 24) & 0xFF, (target.reserved4_u32 >> 16) & 0xFF,
        (target.reserved4_u32 >> 8)  & 0xFF,  target.reserved4_u32 & 0xFF,

        0x00  # checksum 占位
    ]
    data[3] = (len(data) - 5) & 0xFF
    data[-1] = (sum(data[:-1]) & 0xFF)
    return data  # list[int]

# ---------------- 串口接收解析（FF FE），不修改 mode ----------------
class RxState:
    def __init__(self):
        self.buf = []
        self.len_left = 0
        self.state = 0

R = RxState()
rx_frame_ok = False  # 成功解析一帧（校验通过）时置 True

def Receive_Anl(data_buf, total_len):
    global rx_frame_ok
    if total_len < 5:
        return
    calc = (sum(data_buf[:total_len-1]) & 0xFF)
    if calc != (data_buf[total_len-1] & 0xFF):
        return
    rx_frame_ok = True  # 仅作为“触发信号”，不更改 mode

def uart_data_prase(b):
    if R.state == 0 and b == 0xFF:
        R.state = 1
        R.buf = [b]
    elif R.state == 1 and b == 0xFE:
        R.state = 2
        R.buf.append(b)
    elif R.state == 2 and b < 0xFF:  # cmd
        R.state = 3
        R.buf.append(b)
    elif R.state == 3 and b < 250:   # 长度上限
        R.state = 4
        R.len_left = b
        R.buf.append(b)
    elif R.state == 4:
        R.buf.append(b)
        R.len_left -= 1
        if R.len_left <= 0:
            R.state = 5
    elif R.state == 5:
        R.buf.append(b)  # checksum
        try:
            Receive_Anl(R.buf, R.buf[3] + 5)
        except:
            pass
        R.state = 0
        R.buf = []
        R.len_left = 0
    else:
        R.state = 0
        R.buf = []
        R.len_left = 0

def uart_data_read():
    n = yb_uart.any()
    if n:
        rx = yb_uart.read(n)  # bytes
        if rx:
            for bb in rx:
                uart_data_prase(bb & 0xFF)

# ---------------- 工具 ----------------
def pack_status(count):
    """(exist<<8) | count，exist = 1 if count>0 else 0"""
    exist = 1 if count > 0 else 0
    return ((exist & 0x01) << 8) | (count & 0xFF)

clock = time.clock()

# ---------------- 单帧检测，返回“信息结果” ----------------
def detect_one_frame():
    """
    返回：
      'flag'      : 0/1，本帧是否至少检测到一个标签
      'packs'     : [p1..p5] 五个 16 位值（高8位exist，低8位count）
      'counts'    : [c1..c5]
      'prob_sum'  : 本帧所有目标的置信度和（用于并列时的择优）
    """
    clock.tick()
    img = sensor.snapshot()

    kpu.run_with_output(img)
    dets = kpu.regionlayer_yolo2()

    counts = [0, 0, 0, 0, 0]
    prob_sum = 0.0
    if dets:
        for d in dets:
            cls = int(d[4])      # 0..4
            if 0 <= cls < 5:
                counts[cls] += 1
                prob_sum += float(d[5])

    packs = [pack_status(c) for c in counts]
    flag = 1 if sum(counts) > 0 else 0

    # 可选：屏幕显示调试
    fps = clock.fps()
    img.draw_string(0, 0, "%2.1ffps" % fps, color=(0, 60, 255), scale=2.0)
    # 叠加各标签计数
    y0 = 12
    for i in range(5):
        msg = "%s:%d" % (labels[i], counts[i])
        img.draw_string(2, y0 + i*12, msg, color=(255, 255, 255), scale=1.0)
    lcd.display(img)

    return {'flag': flag, 'packs': packs, 'counts': counts, 'prob_sum': prob_sum}

# ---------------- 主循环：触发→20帧检测→挑选最可信→发送 ----------------
while True:
    gc.collect()
    rx_frame_ok = False
    uart_data_read()

    # 只有在收到“触发帧”且 mode 的对应位为 1 时，才执行 20 帧检测
    if rx_frame_ok and ((MODE_CONST >> BIT_TASK) & 0x01):
        results = []
        # 采集并检测 20 帧
        for _ in range(N_FRAMES):
            results.append(detect_one_frame())
            time.sleep_ms(2)

        # 若 20 帧中全为“无标签”
        if all(r['flag'] == 0 for r in results):
            target.flag = 0
            target.angle = 0
            target.distance = 0
            target.apriltag_id = 0
            target.img_width = 0
            target.img_height = 0
        else:
            # 以“packs（五元组）”为键统计出现频次
            freq = {}
            for idx, r in enumerate(results):
                key = tuple(r['packs'])
                lst = freq.get(key)
                if lst is None:
                    freq[key] = [(idx, r['prob_sum'])]
                else:
                    lst.append((idx, r['prob_sum']))

            # 选出“出现次数最多”的信息结果（并列时，取 prob_sum 最大的那一帧）
            best_key = None
            best_list = None
            best_cnt = -1
            for key, lst in freq.items():
                if len(lst) > best_cnt:
                    best_cnt = len(lst)
                    best_key = key
                    best_list = lst
                elif len(lst) == best_cnt:
                    if max(v for _, v in lst) > max(v for _, v in best_list):
                        best_key = key
                        best_list = lst

            # 在 best_list 中选置信度和最大的那一帧
            best_idx = max(best_list, key=lambda t: t[1])[0]
            best = results[best_idx]

            # 写入 target（只发送这个“最可信的一次信息结果”）
            target.flag = best['flag']
            target.angle, target.distance, target.apriltag_id, target.img_width, target.img_height = best['packs']

        # 打包并发送（仅这一帧）
        tx = package_data(MODE_CONST)
        try:
            yb_uart.write(bytearray(tx))  # list[int] -> bytearray
        except Exception as e:
            pass

    # 未收到触发帧：空转
    time.sleep_ms(2)
