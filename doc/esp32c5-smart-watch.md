# ESP32-C5 智能手表：边端云协同的全栈嵌入式开发

> 基于 ESP32-C5 (RISC-V) 芯片，集成 FreeRTOS 多任务调度、LVGL 可视化 UI（SquareLine Studio 设计）、云端 AI 语音交互（ASR+LLM+TTS）、端侧手势识别（TFLite Micro CNN）、AP 配网等功能，构建完整的边端云协同智能穿戴系统。

## 一、项目概述

### 1.1 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        ESP32-C5 智能手表                                │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    FreeRTOS 任务调度层                            │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐           │   │
│  │  │ LVGL UI  │ │ Chat 服务 │ │ 手势识别  │ │ 闹钟/天气 │           │   │
│  │  │ (Core 0) │ │ (Core 0) │ │ (Core 0) │ │ (Core 0) │           │   │
│  │  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘           │   │
│  │       │             │             │             │                 │   │
│  │  ┌────┴─────────────┴─────────────┴─────────────┴─────────────┐  │   │
│  │  │              事件驱动 + 队列 + 信号量                        │  │   │
│  │  └────────────────────────────────────────────────────────────┘  │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│       │             │              │              │                      │
│  ┌────┴────┐   ┌────┴────┐   ┌─────┴────┐   ┌────┴─────┐              │
│  │ BMI270  │   │ADC 麦克风│   │PDM 喇叭  │   │ILI9341   │              │
│  │ 陀螺仪  │   │16kHz    │   │24kHz     │   │284×240   │              │
│  └─────────┘   └─────────┘   └──────────┘   └──────────┘              │
└─────────────────────────────────────────────────────────────────────────┘
       │                              │
       │ 边缘推理                      │ 云端服务
       ▼                              ▼
┌──────────────┐          ┌──────────────────────────────┐
│ TFLite Micro │          │  火山引擎 ASR (WebSocket)    │
│ 1D CNN 模型  │          │  豆包 LLM (HTTPS)            │
│ ~11.6KB      │          │  火山引擎 TTS (WebSocket)    │
│ 7类手势识别   │          │  MQTT Broker (门铃/开门)     │
└──────────────┘          └──────────────────────────────┘
```

### 1.2 边端云分工

| 层级 | 位置 | 功能 | 技术 |
|------|------|------|------|
| **边** | ESP32-C5 本地 | 手势识别、UI 渲染、传感器采集 | TFLite Micro、LVGL、FreeRTOS |
| **端** | ESP32-C5 联网 | WiFi 配网、MQTT 通信、HTTP 请求 | AP Captive Portal、Paho MQTT |
| **云** | 远程服务器 | 语音识别、大模型对话、语音合成、天气 API | 火山引擎 ASR/TTS、豆包 LLM、Open-Meteo |

---

## 二、FreeRTOS 多任务调度

### 2.1 任务拓扑

系统采用 FreeRTOS 多任务架构，各功能模块运行在独立任务中：

```
┌─────────────────────────────────────────────────────────────┐
│                     FreeRTOS 调度器                          │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  LVGL 任务    │  │  Chat 服务    │  │  手势推理     │      │
│  │  Core 0      │  │  Core 0      │  │  Core 0      │      │
│  │  优先级 5     │  │  优先级 4     │  │  优先级 5     │      │
│  │  20KB 栈     │  │  4KB 栈       │  │  6KB 栈      │      │
│  │  5ms 定时器   │  │  状态机驱动   │  │  队列驱动     │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │  录音任务     │  │  ASR 任务     │  │  TTS 播放     │      │
│  │  按需创建     │  │  按需创建     │  │  按需创建     │      │
│  │  优先级 3     │  │  优先级 3     │  │  优先级 4     │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 任务间通信

| 通信方式 | 使用场景 | 说明 |
|---------|---------|------|
| FreeRTOS Queue | 手势采集→推理 | 传递数据指针，避免拷贝 |
| FreeRTOS Semaphore | 推理完成同步 | 通知采集任务释放窗口 |
| FreeRTOS EventGroup | Chat 状态控制 | BIT_RECORDING_START 等事件位 |
| LVGL Lock | UI 线程安全 | `lvgl_port_lock()` 保护多任务访问 |

### 2.3 Chat 服务状态机

```
IDLE ──按下按钮──→ RECORDING ──松开──→ ASR_PROCESSING
                                              │
                                              ▼
TTS_PLAYING ◄────────── LLM_PROCESSING ◄──────┘
     │
     ▼
   IDLE (等待下一轮)
```

### 2.4 手势识别双任务管线

```
┌─────────────────┐     Queue (指针)     ┌──────────────────┐
│  collect_task    │ ──────────────────→ │  inference_task   │
│  5ms 采样一次    │                      │  TFLite Micro    │
│  200 样本窗口    │ ◄── ─ Semaphore ─ ─  │  去抖 + 显示     │
│  滑动步长 30    │                      │                  │
└─────────────────┘                      └──────────────────┘
```

---

## 三、LVGL UI 设计（SquareLine Studio）

### 3.1 SquareLine Studio 工作流

SquareLine Studio 是 LVGL 的可视化 UI 设计工具，导出纯 C 代码，与硬件完全解耦：

```
┌─────────────────────┐     导出      ┌─────────────────────┐
│  SquareLine Studio   │ ───────────→ │  ui_smart_gadget/   │
│  可视化拖拽设计       │   LVGL 9.x   │  screens/           │
│  动画/事件/字体      │   纯 C 代码   │  components/        │
│  模拟器预览          │              │  images/ fonts/     │
└─────────────────────┘              └─────────────────────┘
                                              │
                                              │ ui_init()
                                              ▼
                                    ┌─────────────────────┐
                                    │  display_init()     │
                                    │  LCD + 触摸 + LVGL  │
                                    │  esp_board_manager  │
                                    └─────────────────────┘
```

**核心优势：**
- UI 设计在 PC 端完成，无需反复烧录调试
- 导出的代码是纯 C，不依赖任何 ESP-IDF API
- 修改 UI 只需在 SquareLine 中重新导出，替换文件即可

### 3.2 屏幕结构

| 屏幕 | 功能 | 入口 |
|------|------|------|
| Splash | 启动画面 | 开机自动显示，1.4s 后切换 |
| Clock | 表盘主界面 | 默认主页，显示时间/日期/天气 |
| Chat | AI 语音对话 | 按住麦克风按钮触发 |
| Weather | 天气详情 | 左右滑动切换 |
| Alarm | 闹钟设置 | 10 个槽位 |
| AlarmRing | 闹钟响铃 | 振动 + TTS 语音提醒 |
| HeartDetection | 心率检测 | MAX30102 传感器 |
| WiFiConnection | WiFi 配网 | AP 热点 Captive Portal |
| SearchScreen | 搜索界面 | 手势识别挑战 |

### 3.3 表情动画系统

集成 `esp_lv_eaf_player` 组件，支持 7 种表情动画：

| 表情 | 触发场景 |
|------|---------|
| normal | 默认待机 |
| smile | 识别成功/对话回复 |
| sad | 识别失败 |
| anger | 闹钟超时 |
| amazed | 惊讶（LLM 特殊回复） |
| yes | 确认操作 |
| no | 拒绝操作 |

---

## 四、云端 AI 语音交互

### 4.1 全链路流水线

```
用户按住按钮
    │
    ▼
ADC 麦克风录音 (16kHz, 16bit, 单声道)
    │
    ▼ WebSocket 流式传输
火山引擎 ASR (语音→文字)
    │
    ▼ HTTPS
豆包 LLM doubao-seed-1.6-flash (文字→回复)
    │
    ▼ WebSocket 流式接收
火山引擎 TTS (文字→语音)
    │
    ▼
PDM 喇叭播放 (24kHz)
```

### 4.2 各阶段实现

| 阶段 | 协议 | 端点 | 音频格式 |
|------|------|------|---------|
| ASR | 流式 WebSocket | ai-gateway.vei.volces.com | PCM 16kHz 16bit |
| LLM | HTTPS (OpenAI 兼容) | doubao-seed-1.6-flash | 文本 JSON |
| TTS | 流式 WebSocket | 火山引擎 TTS | PCM 24kHz |

### 4.3 上下文感知

Boot 按钮（GPIO28）的长按行为根据当前页面自动切换：

| 当前页面 | 长按行为 |
|---------|---------|
| Chat | 开始/停止录音 |
| Alarm | 语音设置闹钟 |
| Weather | 语音询问天气 |
| 其他 | 无响应 |

---

## 五、端侧手势识别

### 5.1 模型训练

使用 BMI270 陀螺仪采集手势数据，训练 1D CNN 模型：

**数据集：** 7 类手势（0, 2, 3, 6, 7, 8, Unknown），每类约 50 个样本

**模型架构：**

```
Input (200, 3) — 200个时间步 × 3轴陀螺仪
  ↓ Conv1D(8, kernel=5) + ReLU
  ↓ MaxPool(4)
  ↓ Conv1D(16, kernel=5) + ReLU
  ↓ MaxPool(4)
  ↓ GlobalAveragePooling
  ↓ Dense(32) + Dropout(0.2)
  ↓ Dense(7) + Softmax
Output (7,) — 7类概率
```

**训练结果：**

| 指标 | 值 |
|------|-----|
| 测试准确率 | 94.64% |
| 模型大小 | ~11.6 KB (TFLite float32) |
| 推理时间 | ~20ms (ESP32-C5) |

### 5.2 模型部署

模型通过 TFLite Micro 在 ESP32-C5 上运行：

```
PyTorch/Keras 训练 (.h5)
    │
    ▼ TFLite Converter
TFLite 模型 (.tflite, 11.6KB)
    │
    ▼ xxd 转换
C 字节数组 (model.cpp)
    │
    ▼ TFLite Micro 加载
ESP32-C5 推理 (~20ms/帧)
```

**关键设计：Tensor Arena 动态分配在 PSRAM**

```cpp
// 50KB Tensor Arena 分配在 PSRAM，避免占用内部 SRAM
s_tensor_arena = heap_caps_aligned_alloc(16, 50*1024, MALLOC_CAP_SPIRAM);
```

### 5.3 滑动窗口 + 去抖

```
采集: BMI270 @ 200Hz → 5ms 采样一次
窗口: 200 个样本 (1秒)
步长: 30 个样本 (滑动窗口，重叠 170 个)

去抖:
  连续 3 次识别为同一手势 → 确认输出
  连续 5 次后 → 重置，接受新手势
  置信度 < 85% → 忽略
```

### 5.4 手势开门挑战

将手势识别与 MQTT 远程开门结合，实现"手势密码开门"：

```
屏幕随机显示目标数字 (如 "7")
    │
    ▼
用户在手表上比划手势
    │
    ▼ BMI270 + TFLite Micro
连续 3 次识别为 "7" → 匹配成功
    │
    ▼ MQTT
sml/doorbell/open → ESP32-P4 门锁解锁
```

---

## 六、AP 配网

### 6.1 Captive Portal 配网流程

```
开机 → NVS 有已保存 WiFi?
    │
    ├── 是 → 自动连接 → 成功 → UI 显示 "Connected: XXX"
    │                      → 失败 → UI 显示 "连接失败"
    │
    └── 否 → UI 显示 "No saved WiFi"
              │
              ▼ 用户点击配网按钮
         AP 模式开启 (ESPWatch-XXYY)
         DNS Captive Portal (UDP 53)
         HTTP 服务器 (端口 80)
              │
              ▼ 手机连接 AP
         自动弹出配网页面 (支持 37 种语言)
              │
              ▼ 用户操作
         扫描 → 选择 WiFi → 输入密码 → 提交
              │
              ▼ 连接测试
         成功 → 保存到 NVS → /done.html (3秒倒计时重启)
         失败 → 提示重试
              │
              ▼ 重启
         自动连接已保存的 WiFi
```

### 6.2 组件架构

```
UI 层 (SquareLine C)          桥接层 (C/C++)           组件层 (C++)
ui_WiFiConnection.c           wifi_provision.cc        esp-wifi-connect
  └─ ImgButton2 ──────────►  wifi_btn_click_cb()     WifiConfigurationAp
  └─ 标签 ◄────────────────  update_label_safe()     WifiStation
                                                   SsidManager
                                                   DnsServer
```

### 6.3 NVS 凭据管理

WiFi 凭据保存在 NVS Flash 中，最多存储 10 组 SSID/密码：

- 开机自动读取并尝试连接
- 配网成功后写入新凭据
- 断电不丢失

---

## 七、完整功能矩阵

| 功能 | 边（本地） | 端（联网） | 云（服务） |
|------|-----------|-----------|-----------|
| 手势识别 | TFLite Micro CNN | — | — |
| 表盘显示 | LVGL + SquareLine | NTP 时间同步 | 天气 API |
| AI 对话 | 录音/播放 | WebSocket ASR/TTS | 火山引擎 + 豆包 LLM |
| 闹钟提醒 | 振动 + LVGL | LLM 生成提醒文本 | TTS 语音播报 |
| 远程门铃 | LVGL 告警 UI | MQTT 订阅 + HTTP 照片 | MQTT Broker |
| 手势开门 | BMI270 + 推理 | MQTT 开门指令 | — |
| WiFi 配网 | AP 热点 + DNS | HTTP Captive Portal | — |
| 天气查询 | LVGL 显示 | HTTP GET | Open-Meteo API |

---

## 八、开发经验总结

### 8.1 FreeRTOS 内存管理

ESP32-C5 内部 SRAM 有限（512KB），关键配置：

- LVGL 帧缓冲分配在 PSRAM（80MHz SPI）
- TFLite Tensor Arena 动态分配在 PSRAM（50KB）
- WiFi/LWIP 缓冲区启用 PSRAM 分配
- 大数组避免放在 BSS 段（会导致 WiFi 初始化崩溃）

### 8.2 SquareLine Studio 集成

- 导出的代码是纯 C，通过 `ui_init()` 一行调用即可加载
- 修改 UI 只需替换 `ui_smart_gadget/` 目录下的文件
- LVGL 色深强制 16 位（RGB565），在 `ui.c` 中有编译时检查

### 8.3 端侧推理优化

- Conv1D 在 TFLite Micro 中自动映射为 Conv2D
- 滑动窗口步长 30（重叠 170 样本），确保手势连续性
- 去抖参数：3 次确认 / 5 次重置 / 85% 最低置信度

### 8.4 云端服务集成

- ASR/TTS 使用流式 WebSocket，边录边传/边收边播，降低延迟
- LLM 使用 OpenAI 兼容 API，方便切换模型
- MQTT 用于设备间通信（手表→门锁），QoS 1 保证送达

---

**项目信息：**
- 芯片：ESP32-C5 (RISC-V 单核)
- 框架：ESP-IDF v5.5 + FreeRTOS
- UI：LVGL 9.5.0 + SquareLine Studio
- AI 推理：TFLite Micro ~11.6KB 模型
- 云端服务：火山引擎 ASR/TTS + 豆包 LLM + Open-Meteo
- 通信：MQTT + HTTP + WebSocket + AP Captive Portal
