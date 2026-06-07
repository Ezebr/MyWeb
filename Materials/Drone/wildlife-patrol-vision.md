# 2025 电赛 H 题视觉系统：K210 YOLO 动物检测实现

> 基于 K210 芯片运行 YOLOv2 轻量化目标检测模型，实现 5 类野生动物（象、虎、狼、猴、孔雀）的实时识别，配合 20 帧投票策略提升准确率，通过串口将识别结果回传飞控。

## 一、赛题视觉需求分析

H 题对视觉识别的核心要求：

| 要求 | 内容 | 难点 |
|------|------|------|
| 基本要求(3) | 识别动物种类及数量，将方格代码、动物名称及数量发送到地面站 | 5 类动物形态差异大，需高准确率 |
| 发挥部分(1) | 机载激光笔光斑照射动物体态轮廓 | 需输出目标像素坐标用于激光指向 |
| 说明1(4) | 动物比例 象:虎:狼:猴:孔雀 = 2:1:1:1:1，象长度 30cm | 多尺度检测，大象占多个格子 |
| 说明1(8) | 考虑外界光照、室内照明不均、地图色彩差异 | 模型鲁棒性要求高 |

**5 类动物特征：**

| 动物 | 体型 | 视觉特征 | 检测难度 |
|------|------|---------|---------|
| 象 | 大（30cm） | 长鼻、大耳、灰色 | 较易 |
| 虎 | 中 | 条纹、橙黄色 | 中等 |
| 狼 | 中 | 灰色、体型似犬 | 中等 |
| 猴 | 小 | 棕色、长尾 | 较难 |
| 孔雀 | 小 | 开屏时色彩鲜艳 | 较难 |

---

## 二、硬件平台选型

### 2.1 为什么选择 K210

| 方案 | 优点 | 缺点 |
|------|------|------|
| OpenMV H7 | MicroPython 生态好，开发快 | 算力有限，YOLO 推理慢 |
| 树莓派 + USB 摄像头 | 算力强，模型灵活 | 功耗大、体积大、启动慢 |
| **K210** | 内置 KPU 神经网络加速器，功耗低，体积小 | 模型需转换为 kmodel 格式 |

K210 的 KPU（Knowledge Processing Unit）专门用于神经网络推理加速，YOLOv2 在 320×240 分辨率下可达 15~20 FPS，满足实时检测需求。

### 2.2 硬件连接

```
┌──────────────┐   UART 115200   ┌──────────────┐
│   K210 模块   │ ─────────────→ │   飞控主控    │
│  (MaixPy)    │   IO_6=RX      │  TM4C123     │
│              │   IO_8=TX      │              │
│  摄像头 OV2640│                │              │
└──────────────┘                └──────────────┘
```

---

## 三、模型训练与部署

### 3.1 数据集准备

使用 MaixHub 在线训练平台：

1. **采集样本**：拍摄 5 类动物图片各 100~200 张，包含不同角度、光照、背景
2. **标注**：在 MaixHub 平台上用矩形框标注动物位置，标记类别
3. **训练**：选择 YOLOv2 模型，输入尺寸 320×240，训练 50~100 epochs
4. **导出**：下载 `.kmodel` 格式模型文件（约 571KB）

### 3.2 模型配置

```python
labels = ["elephant", "tiger", "wolf", "monkey", "peacock"]

# YOLOv2 anchor boxes（根据数据集聚类得到）
anchor = (0.91, 1.05,   # anchor 1
          1.58, 0.63,   # anchor 2
          1.29, 0.87,   # anchor 3
          1.67, 1.69,   # anchor 4
          2.77, 1.03)   # anchor 5

kpu = KPU()
kpu.load_kmodel("/sd/det4.kmodel")
kpu.init_yolo2(
    anchor,
    anchor_num=5,          # anchor 数量
    img_w=320, img_h=240,  # 输入图像尺寸
    net_w=320, net_h=240,  # 网络输入尺寸
    layer_w=10, layer_h=8, # 输出特征图尺寸
    threshold=0.4,         # 置信度阈值
    nms_value=0.3,         # NMS 阈值
    classes=5              # 类别数
)
```

### 3.3 模型优化策略

- **anchor 聚类**：根据数据集目标框的宽高比聚类得到 5 组 anchor，提升小目标检测精度
- **阈值调优**：`threshold=0.4` 平衡漏检率和误检率；`nms_value=0.3` 抑制重叠框
- **输入分辨率**：320×240 在 K210 上可达到实时帧率，更高分辨率会明显降低帧率

---

## 四、单帧检测实现

### 4.1 数据结构

定义与飞控端 NCLink 协议兼容的数据结构：

```python
class TargetInfo:
    def __init__(self):
        self.x = 0              # 目标中心 X（像素）
        self.y = 0              # 目标中心 Y（像素）
        self.pixel = 0          # 目标面积（像素）
        self.flag = 0           # 是否检测到目标
        self.state = 0          # 保留
        # 5 个字段重定义为 5 类动物的状态：(exist<<8) | count
        self.angle = 0          # 类别1（象）
        self.distance = 0       # 类别2（虎）
        self.apriltag_id = 0    # 类别3（狼）
        self.img_width = 0      # 类别4（猴）
        self.img_height = 0     # 类别5（孔雀）
        self.fps = 0
        self.camera_id = 0x01   # 摄像头ID

target = TargetInfo()
```

每个动物类别字段编码为 16 位：高 8 位表示是否存在，低 8 位表示数量。

### 4.2 单帧推理

```python
def detect_one_frame():
    """单帧检测，返回各类别计数和置信度总和"""
    clock.tick()
    img = sensor.snapshot()

    # KPU 推理
    kpu.run_with_output(img)
    dets = kpu.regionlayer_yolo2()

    # 统计各类别检测数量
    counts = [0, 0, 0, 0, 0]
    prob_sum = 0.0

    if dets:
        for d in dets:
            cls = int(d[4])       # 类别索引 0~4
            conf = float(d[5])    # 置信度
            if 0 <= cls < 5:
                counts[cls] += 1
                prob_sum += conf

    # 打包为 (exist<<8) | count 格式
    packs = [pack_status(c) for c in counts]
    flag = 1 if sum(counts) > 0 else 0

    # LCD 显示调试信息
    img.draw_string(0, 0, "%2.1ffps" % clock.fps(), color=(0, 60, 255), scale=2.0)
    for i in range(5):
        msg = "%s:%d" % (labels[i], counts[i])
        img.draw_string(2, 12 + i * 12, msg, color=(255, 255, 255), scale=1.0)
    lcd.display(img)

    return {'flag': flag, 'packs': packs, 'counts': counts, 'prob_sum': prob_sum}


def pack_status(count):
    """编码：(exist<<8) | count"""
    exist = 1 if count > 0 else 0
    return ((exist & 0x01) << 8) | (count & 0xFF)
```

### 4.3 检测结果字段映射

| 字段 | 原始含义 | 重定义为 | 说明 |
|------|---------|---------|------|
| angle | 角度 | 象的状态 | (存在<<8) \| 数量 |
| distance | 距离 | 虎的状态 | (存在<<8) \| 数量 |
| apriltag_id | Tag ID | 狼的状态 | (存在<<8) \| 数量 |
| img_width | 图像宽度 | 猴的状态 | (存在<<8) \| 数量 |
| img_height | 图像高度 | 孔雀的状态 | (存在<<8) \| 数量 |

---

## 五、多帧投票策略

### 5.1 为什么需要投票

单帧检测存在的问题：
- 动物图片是平面剪纸，某些角度下特征不明显
- 光照不均导致部分区域颜色失真
- 摄像头运动模糊

单帧准确率约 85%，直接使用会有误检和漏检。

### 5.2 投票算法

采用**连续 20 帧检测 + 频次投票 + 置信度择优**策略：

```python
N_FRAMES = 20
MODE_CONST = 0x07   # 工作模式常量

# 主循环：触发 → 20帧检测 → 投票 → 发送
while True:
    gc.collect()
    rx_frame_ok = False
    uart_data_read()   # 检查是否收到飞控触发指令

    if rx_frame_ok and ((MODE_CONST >> BIT_TASK) & 0x01):
        results = []

        # 阶段1：连续采集 20 帧并检测
        for _ in range(N_FRAMES):
            results.append(detect_one_frame())
            time.sleep_ms(2)

        # 阶段2：投票决策
        if all(r['flag'] == 0 for r in results):
            # 20 帧全部无检测结果 → 确认无动物
            target.flag = 0
            target.angle = target.distance = 0
            target.apriltag_id = target.img_width = target.img_height = 0
        else:
            # 以 packs 五元组为键统计频次
            freq = {}
            for idx, r in enumerate(results):
                key = tuple(r['packs'])
                if key not in freq:
                    freq[key] = [(idx, r['prob_sum'])]
                else:
                    freq[key].append((idx, r['prob_sum']))

            # 选择出现次数最多的结果
            # 并列时取置信度总和最高的帧
            best_key = None
            best_cnt = -1
            best_list = None

            for key, lst in freq.items():
                if len(lst) > best_cnt:
                    best_cnt = len(lst)
                    best_key = key
                    best_list = lst
                elif len(lst) == best_cnt:
                    if max(v for _, v in lst) > max(v for _, v in best_list):
                        best_key = key
                        best_list = lst

            # 在最佳组中选置信度最高的帧
            best_idx = max(best_list, key=lambda t: t[1])[0]
            best = results[best_idx]

            # 写入目标信息
            target.flag = best['flag']
            target.angle = best['packs'][0]         # 象
            target.distance = best['packs'][1]      # 虎
            target.apriltag_id = best['packs'][2]   # 狼
            target.img_width = best['packs'][3]      # 猴
            target.img_height = best['packs'][4]     # 孔雀

        # 阶段3：打包发送
        tx = package_data(MODE_CONST)
        yb_uart.write(bytearray(tx))
```

### 5.3 投票策略示意

```
帧号  检测结果（象,虎,狼,猴,孔雀）  置信度和
 1    (1,0,0,0,0)                  0.85
 2    (1,0,0,0,0)                  0.92
 3    (1,0,0,0,0)                  0.88
 4    (0,0,0,0,0)                  0.00  ← 漏检
 5    (1,0,0,0,0)                  0.79
 ...
 20   (1,0,0,0,0)                  0.91

频次统计：
  (1,0,0,0,0) 出现 18 次 ← 最多
  (0,0,0,0,0) 出现 2 次

最终输出：(1,0,0,0,0)，即检测到 1 只象
```

### 5.4 投票效果

| 指标 | 单帧检测 | 20帧投票 |
|------|---------|---------|
| 准确率 | ~85% | ~95%+ |
| 漏检率 | ~10% | ~2% |
| 误检率 | ~8% | ~3% |
| 响应延迟 | ~60ms | ~1.2s（20帧×60ms） |

1.2 秒的延迟在巡查场景中可接受——无人机每格飞行时间约 3~5 秒。

---

## 六、通信协议

### 6.1 发送帧格式

K210 向飞控发送的检测结果帧：

```
帧头(2B)  功能字(1B)  长度(1B)  数据域(53B)  校验(1B)
 FF FC     0xA0+mode   0x33     payload     checksum
```

数据域结构（53 字节）：

```python
def package_data(mode_val=MODE_CONST):
    data = [
        0xFF, 0xFC,                          # 帧头
        (0xA0 + (mode_val & 0xFF)) & 0xFF,  # 功能字
        0x00,                                 # 长度占位

        (target.x >> 8) & 0xFF, target.x & 0xFF,           # 目标X
        (target.y >> 8) & 0xFF, target.y & 0xFF,           # 目标Y
        (target.pixel >> 8) & 0xFF, target.pixel & 0xFF,   # 面积

        target.flag & 0xFF,                  # 检测标志
        target.state & 0xFF,                 # 保留

        (target.angle >> 8) & 0xFF, target.angle & 0xFF,           # 象
        (target.distance >> 8) & 0xFF, target.distance & 0xFF,     # 虎
        (target.apriltag_id >> 8) & 0xFF, target.apriltag_id & 0xFF, # 狼
        (target.img_width >> 8) & 0xFF, target.img_width & 0xFF,   # 猴
        (target.img_height >> 8) & 0xFF, target.img_height & 0xFF, # 孔雀

        target.fps & 0xFF,                   # 帧率
        # ... 保留字段
        target.camera_id & 0xFF,             # 摄像头ID
        # ... 保留 int32 字段
        0x00                                  # 校验占位
    ]
    data[3] = (len(data) - 5) & 0xFF        # 填充长度
    data[-1] = (sum(data[:-1]) & 0xFF)       # 填充校验和
    return data
```

### 6.2 接收触发帧

飞控通过发送触发帧通知 K210 开始检测：

```python
# 接收状态机
def uart_data_prase(b):
    if R.state == 0 and b == 0xFF:      # 帧头1
        R.state = 1
    elif R.state == 1 and b == 0xFE:    # 帧头2
        R.state = 2
    elif R.state == 2:                   # 功能字
        R.state = 3
    elif R.state == 3:                   # 长度
        R.state = 4
    elif R.state == 4:                   # 数据+校验
        # 校验通过后设置触发标志
        rx_frame_ok = True
```

---

## 七、与飞控端的数据流

### 7.1 完整数据流

```
飞控 (TM4C123)                          K210 视觉模块
    │                                       │
    │  触发帧 (FF FE ...)                    │
    │ ────────────────────────────────────► │
    │                                       │
    │                              20帧YOLO检测
    │                              频次投票决策
    │                                       │
    │  检测结果帧 (FF FC ...)                │
    │ ◄──────────────────────────────────── │
    │                                       │
    ▼                                       │
解析结果：                                  │
  • flag = 1 (检测到动物)                    │
  • 象: 1只                                 │
  • 虎: 0只                                 │
  • 狼: 0只                                 │
  • 猴: 0只                                 │
  • 孔雀: 0只                               │
    │                                       │
    ▼                                       │
NCLink_Send_To_Firetruck(                   │
  count_x, count_y,     ← 当前方格坐标      │
  1, 0, 0, 0, 0,       ← 各类别数量         │
  ...                                       │
)                                           │
    │                                       │
    ▼                                       │
地面站显示：                                │
  方格 A3B5：象 ×1                          │
```

### 7.2 飞控端解析

飞控端 `SDK_Data_Prase_1()` 解析 K210 返回的数据：

```c
// 从 Target_Check 结构体中提取动物数量
uint8_t elephant_count = Opv_Top_View_Target.angle & 0xFF;      // 象
uint8_t tiger_count    = Opv_Top_View_Target.distance & 0xFF;   // 虎
uint8_t wolf_count     = Opv_Top_View_Target.apriltag_id & 0xFF; // 狼
uint8_t monkey_count   = Opv_Top_View_Target.img_width & 0xFF;  // 猴
uint8_t peacock_count  = Opv_Top_View_Target.img_height & 0xFF; // 孔雀
```

---

## 八、发挥部分：激光笔照射

赛题要求发现动物时用激光笔照射动物轮廓。飞控端根据 K210 返回的目标像素坐标，通过舵机控制激光笔指向：

```c
// 激光笔指向控制（简化逻辑）
if (Opv_Top_View_Target.flag == 1) {
    // 目标中心像素坐标
    int cx = Opv_Top_View_Target.x;
    int cy = Opv_Top_View_Target.y;

    // 像素偏差 → 舵机角度
    // 图像中心 (160, 120)，偏差映射为舵机 PWM
    int err_x = cx - 160;
    int err_y = cy - 120;

    // 舵机控制激光笔指向
    Laser_Servo_X += err_x * Kp_laser;
    Laser_Servo_Y += err_y * Kp_laser;
}
```

---

## 九、开发调试经验

### 9.1 模型训练技巧

1. **数据增强**：对训练图片做随机旋转（±15°）、亮度调整（±20%）、添加噪声，提升模型鲁棒性
2. **负样本**：加入不含动物的空格子图片作为负样本，降低误检率
3. **小目标处理**：猴和孔雀体型小，训练时适当提高小目标的 loss 权重
4. **多角度采集**：动物图片由参赛队粘贴方向可变，需覆盖 0°/90°/180°/270° 四个方向

### 9.2 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| 检测框偏移 | 摄像头畸变 | 标定摄像头内参，或限制画面中心区域检测 |
| 小动物漏检 | 特征不明显 | 降低 threshold 到 0.3，或增加训练样本 |
| 光照影响大 | 色彩偏移 | 数据增强中加入色彩抖动 |
| 帧率太低 | 模型过大 | 减少 anchor 数量，或降低输入分辨率 |
| 串口丢包 | 波特率过高 | 增加帧间延时，或使用硬件流控 |

### 9.3 调试命令

```python
# MaixPy 终端调试
import sensor
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.snapshot()  # 预览画面

# 测试单帧检测
kpu.run_with_output(sensor.snapshot())
print(kpu.regionlayer_yolo2())
```

---

## 十、总结

本视觉系统针对 2025 电赛 H 题的动物检测需求，基于 K210 芯片实现了完整的 YOLO 目标检测方案：

1. **模型部署**：YOLOv2 模型在 MaixHub 训练后导出为 kmodel，部署到 K210 的 KPU 加速器
2. **5 类动物检测**：象、虎、狼、猴、孔雀，320×240 分辨率下可达 15~20 FPS
3. **20 帧投票策略**：频次统计 + 置信度择优，将准确率从 85% 提升至 95%+
4. **协议兼容**：检测结果封装为 53 字节帧，通过 UART 发送到飞控，与 NCLink 协议对接
5. **激光笔指向**：利用目标像素坐标控制舵机，实现激光照射动物轮廓

系统实测在室内光照条件下，5 类动物的综合识别准确率达到 95% 以上，单格检测响应时间约 1.2 秒，满足赛题 300 秒巡查时限要求。

---

**项目信息：**
- 竞赛：2025 年全国大学生电子设计竞赛 H 题（野生动物巡查系统）
- 视觉芯片：K210（嘉楠科技）
- 检测模型：YOLOv2（MaixHub 训练，kmodel 格式）
- 模型大小：571KB
- 输入分辨率：320×240
- 检测帧率：15~20 FPS（单帧），约 0.8 FPS（20帧投票）
- 检测类别：象、虎、狼、猴、孔雀（5 类）
