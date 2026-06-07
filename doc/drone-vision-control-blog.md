# 从视觉识别到自主飞行：四旋翼无人机飞控系统开发实战

> 基于 TI TM4C123GH6PM 的 CarryPilot 飞控平台，结合 OpenMV / K210 视觉模块，实现无人机目标检测、自主巡线与定点飞行。

## 一、项目背景

在 2025 年全国大学生电子设计竞赛（H 题·野生动物巡查系统）中，需要无人机在复杂野外环境中完成动物目标识别、航线巡航与坐标回传等任务。本文围绕我在备赛过程中开发的飞控系统展开，重点介绍**视觉识别**与**飞行控制**两大核心模块的设计与实现。

整套系统基于以下硬件平台：

| 模块 | 型号 | 作用 |
|------|------|------|
| 飞控主芯片 | TI TM4C123GH6PM (ARM Cortex-M4F, 80MHz) | 姿态解算、PID 控制、任务调度 |
| 底部视觉 | OpenMV H7 / K210 | 目标检测、巡线、AprilTag 定位 |
| IMU | ICM20689 | 三轴陀螺仪 + 三轴加速度计 (1000Hz 采样) |
| 气压计 | SPL06 | 气压高度 |
| 对地测距 | TOFSense / VL53L1X | 精确对地高度 |
| 光流 | LC306 | 水平速度估计 |
| 地面站 | NCLink V1.0.6 | 参数调试、遥测显示 |

---

## 二、视觉识别系统

### 2.1 系统架构

视觉模块通过 UART（256000 baud）与飞控通信，采用统一的 `Target_Check` 数据结构回传识别结果。飞控端通过 NCLink 协议解析后，将目标坐标用于位置闭环控制。

```
┌─────────────┐     UART      ┌─────────────┐     PID     ┌──────────┐
│  视觉模块    │ ───────────→ │   飞控主控   │ ─────────→ │  电调/电机 │
│ OpenMV/K210 │  256000 baud  │  TM4C123    │   输出PWM   │  ×4      │
└─────────────┘               └─────────────┘             └──────────┘
```

### 2.2 OpenMV 视觉方案

OpenMV 运行 MicroPython，支持多种视觉任务，通过 `work_mode` 变量切换工作模式：

| work_mode | 功能 | 算法 |
|-----------|------|------|
| 0x01 | 色块检测 | LAB 色彩空间 + `find_blobs()` |
| 0x02 | AprilTag 定位 | `find_apriltags()` (TAG36H11) |
| 0x03 | 巡线跟踪 | `get_regression()` 线性回归 |
| 0x07 | 作物识别 | ROI 区域色块检测 |

#### 色块检测实现

核心思路是在 LAB 色彩空间下设定阈值，找到面积最大的色块并返回其中心坐标：

```python
blob_threshold_rgb = [40, 100, 30, 127, 0, 127]  # LAB 阈值

def opv_find_color_blob():
    img = sensor.snapshot()
    pixels_max = 0
    for b in img.find_blobs([blob_threshold_rgb], pixels_threshold=30,
                             merge=True, margin=50):
        if pixels_max < b.pixels():
            pixels_max = b.pixels()
            target.x = b.cx()        # 色块中心 X
            target.y = b.cy()        # 色块中心 Y
            target.pixel = pixels_max # 像素面积
            target.flag = 1           # 检测到标志
```

飞控端收到 `(x, y)` 像素坐标后，通过 SDK 位置控制器将像素偏差转换为期望姿态角，驱动无人机飞向目标。

#### 巡线跟踪实现

巡线使用线性回归算法，输出直线的 `rho`（极径）和 `theta`（极角），飞控根据角度偏差修正偏航：

```python
thresholds = (0, 30, -30, 30, -30, 30)

def found_line():
    img = sensor.snapshot()
    line_state = img.get_regression([thresholds], x_stride=2, y_stride=2,
                                     pixels_threshold=10, robust=True)
    if line_state:
        target.x = abs(line_state.rho())    # 线条偏移量
        target.angle = line_state.theta()   # 线条角度
        target.flag = 1
        img.draw_line(line_state.line(), color=127)
```

#### AprilTag 定位

AprilTag 提供更精确的空间位姿信息，可用于精确悬停或降落对准。代码会筛选距画面中心最近的 Tag：

```python
def opv_find_april_tag():
    img = sensor.snapshot()
    apriltag_dis = IMAGE_DIS_MAX
    for tag in img.find_apriltags():
        # 计算 Tag 到画面中心的距离
        dis_tmp = math.sqrt((tag.cx()-80)**2 + (tag.cy()-60)**2)
        if apriltag_dis > dis_tmp:
            target.x = tag.cx()
            target.y = tag.cy()
            target.apriltag_id = tag.id()
            target.pixel = int(tag.rect()[2]**2)  # 面积
            target.flag = 1
```

### 2.3 K210 YOLO 深度学习方案

针对 2025 年电赛 H 题（野生动物巡查），动物目标种类多、形态差异大，传统色块检测难以胜任。因此引入 K210 芯片运行 YOLOv2 轻量化目标检测模型。

#### 模型训练

使用 MaixHub 在线训练平台，标注 5 类动物目标（标签 "1"~"5"），导出 `.kmodel` 格式模型文件（约 571KB），部署到 K210 的 KPU（神经网络加速器）。

#### 推理流程

```python
# 模型配置
labels = ["1", "2", "3", "4", "5"]
anchor = (0.91, 1.05, 1.58, 0.63, 1.29, 0.87, 1.67, 1.69, 2.77, 1.03)

kpu = KPU()
kpu.load_kmodel("/sd/det4.kmodel")
kpu.init_yolo2(anchor, anchor_num=5, img_w=320, img_h=240,
               net_w=320, net_h=240, layer_w=10, layer_h=8,
               threshold=0.4, nms_value=0.3, classes=5)
```

单帧检测返回每个类别的出现次数与置信度：

```python
def detect_one_frame():
    img = sensor.snapshot()
    kpu.run_with_output(img)
    dets = kpu.regionlayer_yolo2()

    counts = [0, 0, 0, 0, 0]
    prob_sum = 0.0
    if dets:
        for d in dets:
            cls = int(d[4])      # 类别索引 0~4
            counts[cls] += 1
            prob_sum += float(d[5])

    return {'counts': counts, 'prob_sum': prob_sum}
```

#### 多帧投票策略

单帧检测可能存在误检或漏检。为此采用**连续 20 帧检测 + 频次投票**的策略：统计 20 帧中出现次数最多的检测结果，并在并列时选择置信度最高的帧作为最终输出。

```python
N_FRAMES = 20

# 触发后连续采集 20 帧
for _ in range(N_FRAMES):
    results.append(detect_one_frame())

# 统计各检测结果出现频次
freq = {}
for idx, r in enumerate(results):
    key = tuple(r['packs'])
    if key not in freq:
        freq[key] = [(idx, r['prob_sum'])]
    else:
        freq[key].append((idx, r['prob_sum']))

# 选择出现次数最多的结果（并列取置信度最高）
best_key = max(freq.items(), key=lambda x: (len(x[1]), max(v for _, v in x[1])))[0]
```

这种策略有效降低了误检率，在实测中将识别准确率从单帧约 85% 提升至 95% 以上。

### 2.4 通信协议设计

视觉模块与飞控之间采用统一的二进制帧格式：

```
帧头(2B)  功能字(1B)  长度(1B)  数据域(NB)  校验(1B)
 FF FC     0xA0+mode   len      payload    checksum
```

数据域包含 53 字节的 `Target_Check` 结构体：

| 字段 | 类型 | 含义 |
|------|------|------|
| x, y | int16 | 目标中心像素坐标 |
| pixel | uint16 | 目标像素面积 |
| flag | uint8 | 是否检测到目标 |
| angle | int16 | 巡线角度 / 类别状态 |
| apriltag_id | uint16 | AprilTag ID |
| range_sensor1~4 | uint16 | 预留测距数据 |
| camera_id | uint8 | 摄像头标识 (0x01=底部) |

---

## 三、飞行控制系统

### 3.1 整体架构

飞控软件采用**定时器中断驱动**的多任务架构，不同频率处理不同层次的控制：

| 定时器 | 周期 | 频率 | 任务 |
|--------|------|------|------|
| TIMER1A | 1ms | 1000Hz | IMU 数据采集（陀螺仪 + 加速度计） |
| TIMER0 | 5ms | 200Hz | 传感器融合、姿态解算 |
| TIMER2 | 10ms | 100Hz | 位置控制、任务调度 |
| 主循环 | ~20ms | 50Hz | OLED 显示、按键扫描、校准检查 |

### 3.2 级联 PID 控制

控制系统采用经典的**串级 PID** 结构，从内到外分为四层：

```
位置环 (100Hz)          姿态角环 (200Hz)         角速率环 (200Hz)
┌──────────┐          ┌──────────┐           ┌──────────┐
│ 期望位置  │          │ 期望角度  │           │期望角速率 │
│    ↓     │   角度    │    ↓     │   角速率   │    ↓     │
│ 位置PID  │ ──────→ │ 角度PID  │ ───────→ │ 角速率PID│ ──→ PWM
│    ↑     │  设定值   │    ↑     │   设定值   │    ↑     │
│ 反馈位置  │          │ 反馈角度  │           │ 反馈角速率 │
└──────────┘          └──────────┘           └──────────┘
  光流/SLAM              融合角度               陀螺仪
```

PID 控制器结构体包含完整的工程化参数：

```c
typedef struct {
    float Kp, Ki, Kd;           // PID 三参数
    float Expect;               // 期望值
    float FeedBack;             // 反馈值
    float Err;                  // 当前偏差
    float Integrate;            // 积分累加
    float Integrate_Max;        // 积分限幅
    float Control_OutPut;       // 控制输出
    float Control_OutPut_Limit; // 输出限幅
    uint8 Integrate_Separation_Flag; // 积分分离标志
    uint8 Err_Limit_Flag;       // 偏差限幅标志
    // ... 微分滤波、自适应参数等
} PID_Controler;
```

系统共配置 **20 个 PID 控制器**，涵盖姿态、高度、水平位置、光流、SDK 等全通道：

```c
typedef struct {
    PID_Controler Pitch_Angle_Control;   // 俯仰角度环
    PID_Controler Pitch_Gyro_Control;    // 俯仰角速率环
    PID_Controler Roll_Angle_Control;    // 横滚角度环
    PID_Controler Roll_Gyro_Control;     // 横滚角速率环
    PID_Controler Yaw_Angle_Control;     // 偏航角度环
    PID_Controler Yaw_Gyro_Control;      // 偏航角速率环
    PID_Controler Height_Position_Control; // 高度位置环
    PID_Controler Height_Speed_Control;    // 高度速度环
    PID_Controler Longitude_Position_Control; // 经度位置环
    PID_Controler Latitude_Position_Control;  // 纬度位置环
    // ... 光流、SDK、温度控制等
} AllControler;
```

### 3.3 姿态解算

姿态解算基于**捷联惯性导航（SINS）** + **扩展卡尔曼滤波**，融合多传感器数据：

```c
void SINS_Prepare(void) {
    // 1. 陀螺仪数据 → 角速率
    // 2. 加速度计数据 → 重力方向
    // 3. 磁力计数据 → 航向角
    // 4. 气压计/TOF → 高度
}

void Strapdown_INS_High_Kalman(void) {
    // 卡尔曼滤波融合加速度计与高度观测
    // 状态量：[位置, 速度, 加速度偏差]
    // 观测量：气压计高度 / TOF 高度
}
```

IMU 采样率 1000Hz，通过 50Hz 低通滤波去除高频噪声，再送入姿态解算：

```c
#define imu_sampling_hz    1000
#define fc_ctrl_hz         200   // 控制频率
#define gyro_lpf_param_default1  50   // 陀螺仪低通滤波截止频率
#define accel_lpf_param_default1 30   // 加速度计低通滤波截止频率
```

### 3.4 航线设置与自主飞行

飞控支持多种自主飞行模式，通过 SDK（Software Development Kit）接口实现：

#### 导航控制函数

提供"搭积木式"编程接口，用户组合基本动作即可实现复杂航线：

```c
// X/Y 方向位置控制（相对/绝对坐标）
uint8_t move_with_xy_target(float pos_x_target, float pos_y_target,
                             SDK_Status *Status, uint16_t number);

// 速度控制
uint8_t move_with_speed_target(float x_target, float y_target,
                                float delta, SDK_Status *Status, uint16_t number);

// 高度控制
uint8_t move_with_z_target(float z_target, float z_vel,
                            float delta, SDK_Status *Status, uint16_t number);
```

#### 偏航控制模式

偏航支持 6 种控制模式，满足不同任务需求：

```c
enum YAW_CTRL_MODE {
    ROTATE = 0,           // 手动偏航（遥控器控制）
    AZIMUTH = 1,          // 绝对偏航角控制
    CLOCKWISE = 2,        // 顺时针旋转指定角度
    ANTI_CLOCKWISE = 3,   // 逆时针旋转指定角度
    CLOCKWISE_TURN = 4,   // 顺时针角速度控制
    ANTI_CLOCKWISE_TURN = 5 // 逆时针角速度控制
};
```

#### 电赛任务模板

`Subtask_Demo.c` 中预置了多种竞赛任务的参考实现，以"旋转指定角度"为例：

```c
uint8_t adjust_deg_wise(float deg) {
    // 阶段 0：启动偏航控制
    if (flight_subtask_cnt[n] == 0) {
        Flight.yaw_ctrl_mode = CLOCKWISE;   // 顺时针模式
        Flight.yaw_ctrl_start = 1;          // 启动标志
        Flight.yaw_outer_control_output = deg; // 目标角度
        OpticalFlow_Control_Pure(0);        // SLAM 位置保持
        Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL, NUL, NUL); // 高度保持
        flight_subtask_cnt[n] = 1;
        return 0;
    }
    // 阶段 1：等待旋转完成
    else if (flight_subtask_cnt[n] == 1) {
        if (Flight.yaw_ctrl_end == 1)       // 旋转完成标志
            flight_subtask_cnt[n] = 2;
        return 0;
    }
    // 阶段 2：复位，返回完成
    else {
        basic_auto_flight_support();        // 恢复基本飞行支持
        flight_subtask_cnt[n] = 0;
        return 1;  // 任务完成
    }
}
```

用户可以通过按键录入航点坐标，配合激光雷达 SLAM 的全局定位，实现"指哪打哪"的自主飞行。

### 3.5 电调 PWM 输出

电机控制通过 PWM 信号驱动电调，支持两种接线方式：

```c
#define FLIGHT_ESC_PWM  0  // 0: 使用 EPWM (P5~P8), 1: 使用排针 PWM (P1~P4)

// 电机映射（默认 EPWM 模式）
// M1 → PE5 (M0PWM5),  M2 → PE4 (M0PWM4)
// M3 → PB4 (M0PWM2),  M4 → PB5 (M0PWM3)

void PWM_Output(uint16_t width1, uint16_t width2,
                uint16_t width3, uint16_t width4);
```

四电机混控逻辑：

```
       FRONT
     3(CCW)  1(CW)
        *
     2(CW)   4(CCW)

输出 = 油门 ± Roll ± Pitch ± Yaw
```

---

## 四、系统集成与调试

### 4.1 传感器校准

飞控支持板载按键完成全套校准流程，无需连接电脑：

1. **加速度计 6 面校准** — 依次放置六个面，自动采集并计算零偏与比例因子
2. **磁力计椭球校准** — 旋转机体采集数据，拟合椭球补偿硬磁/软磁干扰
3. **遥控器行程校准** — 自动记录各通道最大/最小值
4. **机架水平校准** — 将当前姿态设为水平基准

所有校准参数与 PID 参数存储在 Flash 中，支持一键恢复出厂设置。

### 4.2 地面站调试

通过 UART1 连接 NCLink 地面站（921600 baud），可实时查看：

- 三轴姿态角与角速率
- 传感器原始数据（加速度计、陀螺仪、磁力计）
- 位置与速度融合结果
- PID 参数在线修改
- GPS / SLAM 定位数据

### 4.3 按键参数调整

飞控板载按键支持在 OLED 显示屏上直接调整 PID 参数：

- 上/下翻页切换参数项
- 长按进入编辑模式
- 短按增减值
- 支持恢复出厂设置

---

## 五、总结

本项目从底层驱动到上层应用，实现了一套完整的四旋翼无人机飞控系统。在视觉方面，分别针对传统色块/AprilTag 场景和深度学习目标检测场景，设计了 OpenMV 与 K210 双方案；在控制方面，通过级联 PID + SINS 惯导融合 + SDK 导航函数的架构，实现了从手动飞行到自主航线的完整功能链。

整套系统在 2025 年电赛备赛过程中经过反复调试验证，能够在室内/室外多种环境下稳定飞行，完成了目标识别、航线巡航、定点悬停等竞赛任务。

---

**项目相关资源：**
- 飞控固件：CarryPilot V6.0.2（基于 TI Tiva C Series TM4C123GH6PM）
- 视觉方案：OpenMV H7 (MicroPython) + K210 (MaixPy + YOLOv2)
- 地面站：NCLink V1.0.6
- 开发环境：Keil MDK µVision 5
