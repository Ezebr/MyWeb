# ESP32-P4 边缘 AI 智能门锁：从模型训练到 FreeRTOS 多任务调度

> 基于 ESP32-P4 芯片，在端侧实现人脸检测→活体检测→人脸识别的完整流水线，配合 FreeRTOS 任务调度、WiFi 联网、MQTT 智能家居控制、语音交互等功能，构建一套完整的边缘 AI 系统。

## 一、项目概述

### 1.1 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    ESP32-P4 Function EV Board                   │
│                                                                 │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐  │
│  │ MIPI CSI │───→│ 人脸检测  │───→│ 活体检测  │───→│ 人脸识别  │  │
│  │ SC2336   │    │ esp-dl   │    │ LivenessCNN│   │ 特征匹配  │  │
│  └──────────┘    └──────────┘    └──────────┘    └─────┬────┘  │
│                                                       │        │
│  ┌──────────────────────────────────────────────────────┤        │
│  │                    门锁控制逻辑                      │        │
│  └──────────────────────────────────────────────────────┤        │
│                                                       │        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┤        │
│  │ WiFi     │  │ MQTT     │  │ 语音交互  │  │ LVGL UI  │        │
│  │ 联网管理  │  │ 智能家居  │  │ ASR+LLM  │  │ 触摸屏   │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 核心特性

| 功能 | 实现方式 | 模型/算法 |
|------|---------|----------|
| 人脸检测 | esp-dl CNN | 量化 INT8 模型 |
| 活体检测 | MobileNetV1 ×0.5 | FP32→INT8 量化，~160KB |
| 人脸识别 | esp-dl 特征提取 + 余弦匹配 | 128 维特征向量 |
| 语音交互 | WakeNet + 流式 ASR + LLM + TTS | ESP-SR + 火山引擎 |
| 智能家居 | MQTT 协议 | 自建 Broker |
| 远程监控 | JPEG 硬件编码 + HTTP POST | 10 FPS |
| 二维码门禁 | esp-dl QR 码检测 | — |

---

## 二、边缘计算流水线

### 2.1 整体流水线

系统采用**事件驱动的流水线架构**，各阶段通过 FreeRTOS 队列连接：

```
MIPI CSI Camera (SC2336, 1024×600)
    │
    ▼
WhoFetchNode ──→ RingBuf (3帧缓冲) ──→ 下游节点
    │
    ├──→ WhoDetect (人脸检测) ──→ 检测结果队列
    │         │
    │         ├──→ LivenessCNN (活体检测) ──→ 活体结果
    │         │
    │         └──→ WhoRecognitionCore (人脸识别) ──→ 用户ID
    │
    ├──→ WhoFrameLCD (LCD显示)
    │
    └──→ WhoRemoteMonitor (远程监控, JPEG编码+HTTP上传)
```

### 2.2 人脸检测

`WhoDetect` 继承自 `WhoTask`，在独立 FreeRTOS 任务中运行：

```cpp
class WhoDetect : public task::WhoTask {
public:
    typedef struct {
        std::list<dl::detect::result_t> det_res;  // 检测结果列表
        struct timeval timestamp;                   // 时间戳
        dl::image::img_t img;                       // 原始图像
    } result_t;

    WhoDetect(const std::string &name, frame_cap::WhoFrameCapNode *frame_cap_node);

    void set_model(dl::detect::Detect *model);
    void set_detect_result_cb(const std::function<void(const result_t &)> &result_cb);

    bool run(configSTACK_DEPTH_TYPE uxStackDepth, UBaseType_t uxPriority, BaseType_t xCoreID) override;

private:
    void task() override;   // FreeRTOS 任务主循环
    frame_cap::WhoFrameCapNode *m_frame_cap_node;
    dl::detect::Detect *m_model;
    std::function<void(const result_t &)> m_result_cb;  // 检测结果回调
};
```

**工作流程：**
1. 等待 `WhoFetchNode` 发出 `NEW_FRAME` 事件
2. 从 RingBuf 获取最新帧
3. 运行 esp-dl 人脸检测 CNN
4. 通过回调函数将检测结果传递给下游（活体检测、人脸识别）

### 2.3 活体检测（边缘 AI 核心）

活体检测是防止照片/屏幕攻击的关键环节，在检测到人脸后、识别前执行：

```cpp
namespace human_face_liveness {

struct LivenessResult {
    float live_probability;    // 真人概率
    float spoof_probability;   // 攻击概率
    bool is_live(float threshold) const { return live_probability >= threshold; }
};

class LivenessCNN {
public:
    LivenessCNN(const char *model_name);

    // 输入：原始图像 + 人脸边界框
    // 输出：活体检测结果
    LivenessResult classify(const dl::image::img_t &img,
                            const std::vector<int> &crop_area);

private:
    dl::Model *m_model;                         // esp-dl 模型
    dl::image::ImagePreprocessor *m_image_preprocessor;  // 图像预处理
};
```

**推理流程：**

```
原始图像 (1024×600 RGB565)
    │
    ▼ 根据检测框裁剪人脸区域
人脸裁剪图
    │
    ▼ ImagePreprocessor: resize → 96×96 → 归一化 → INT8 量化
96×96×3 INT8 输入
    │
    ▼ MobileNetV1 ×0.5 推理 (esp-dl, ~40ms)
输出: [P(spoof), P(live)]
    │
    ▼ 阈值判断 (threshold=0.5)
活体/攻击
```

### 2.4 人脸识别

活体检测通过后，运行人脸识别提取 128 维特征向量，与数据库余弦匹配：

```cpp
class WhoRecognitionCore : public task::WhoTask {
public:
    void set_recognizer(HumanFaceRecognizer *recognizer);
    void set_recognition_result_cb(const std::function<void(const std::string &)> &result_cb);

private:
    void task() override {
        // 1. 等待检测结果
        // 2. 提取人脸特征 (128维向量)
        // 3. 与已注册用户数据库做余弦相似度匹配
        // 4. 输出用户ID或"未知"
    }
    HumanFaceRecognizer *m_recognizer;
};
```

### 2.5 完整识别流程

```
┌─────────────┐
│  检测到人脸   │
└──────┬──────┘
       ▼
┌─────────────┐     未通过     ┌─────────────┐
│  活体检测    │ ────────────→ │  拒绝开门    │
│ threshold≥0.5│               │  "请使用真人" │
└──────┬──────┘               └─────────────┘
       │ 通过
       ▼
┌─────────────┐     未匹配     ┌─────────────┐
│  人脸识别    │ ────────────→ │  "未注册用户" │
│ 特征余弦匹配 │               │  可选注册     │
└──────┬──────┘               └─────────────┘
       │ 匹配成功
       ▼
┌─────────────┐
│  解锁门锁    │
│  语音播报    │
│  MQTT 上报   │
│  访问日志    │
└─────────────┘
```

---

## 三、活体检测模型训练

### 3.1 模型架构

采用 **MobileNetV1 ×0.5**（宽度乘数 0.5），约 37.6K 参数，INT8 量化后约 160KB：

```
输入 96×96×3
  ↓ Conv 3×3, stride=2        → 48×48×16
  ↓ DepthSepConv s=1           → 48×48×32
  ↓ DepthSepConv s=2           → 24×24×64
  ↓ DepthSepConv s=1           → 24×24×64
  ↓ DepthSepConv s=2           → 12×12×128
  ↓ DepthSepConv s=1           → 12×12×128
  ↓ DepthSepConv s=2           → 6×6×256
  ↓ DepthSepConv s=1           → 6×6×256
  ↓ GlobalAveragePool           → 256
  ↓ Dropout(0.3)
  ↓ FC 256→2                   → [logit_spoof, logit_live]
  ↓ Softmax                    → [P(spoof), P(live)]
```

**深度可分离卷积（Depthwise Separable Convolution）** 是 MobileNet 的核心：

```
普通卷积:  输入 → [Conv 3×3 + BN + ReLU] → 输出
           参数量 = 3×3×C_in×C_out

深度可分离: 输入 → [DW Conv 3×3] → [PW Conv 1×1] → 输出
           参数量 = 3×3×C_in + C_in×C_out
           参数减少 8~10 倍
```

### 3.2 训练数据

| 数据集 | live 样本 | spoof 样本 | 攻击类型 |
|--------|----------|-----------|---------|
| V2 原始数据 | 1,397 | 4,856 | 打印照片 |
| CelebA-Spoof | ~10K | ~10K | 多种攻击 |
| 合并 (Data4) | ~11.4K | ~15.2K | 5 种攻击 |

数据比例 1:1.33，使用 `WeightedRandomSampler` 保证每个 batch 均衡采样。

### 3.3 数据增强策略

增强管线**定向模拟 ESP32-P4 ISP 的实际行为**，而非盲目增强：

```python
transform = transforms.Compose([
    transforms.RandomHorizontalFlip(p=0.5),
    transforms.ColorJitter(brightness=0.25, contrast=0.15),  # AE 活跃
    # R/B 通道增益增强 — 模拟 AWB 漂移（根因）
    RandomChannelGain(channel='R', range=(0.80, 1.50), p=0.35),
    RandomChannelGain(channel='B', range=(0.80, 1.30), p=0.35),
    transforms.GaussianBlur(kernel_size=3, sigma=(0.3, 0.8)),  # 运动/失焦
    AddGaussianNoise(std=0.02),  # 传感器噪声
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
])
```

**关键设计决策：**

| 增强 | 是否使用 | 原因 |
|------|---------|------|
| R 通道增益 | ✅ | AWB R_gain 漂移是确认的根因 |
| B 通道增益 | ✅ | AWB B_gain 同步漂移 |
| Hue 色相旋转 | ❌ | 色相旋转 ≠ AWB 通道增益，机制不同 |
| Saturation 饱和度 | ❌ | ISP ACC 模块不活跃（硬件日志 sat=0） |

### 3.4 量化部署

```
PyTorch FP32 (.pth)
    │
    ▼ ESP-PPQ 1.2.9 量化
INT8 per-tensor (.espdl, ~160KB)
    │
    ▼ ESP-DL 3.2.4 加载运行
ESP32-P4 推理 (~40ms/帧)
```

**预处理对齐**是影响准确率的关键——训练和推理必须使用完全一致的裁剪+归一化方式：

```
训练: 检测框 → crop + margin → resize 96×96 → 亮度均值减法 → INT8
推理: 检测框 → crop           → resize 96×96 → 亮度均值减法 → INT8
              ↑                                 ↑
           完全一致的处理方式
```

**亮度均值减法**（V2.2 方案）：

```cpp
// 每帧计算亮度均值
Y_mean = 0.299 * R̄ + 0.587 * Ḡ + 0.114 * B̄   // ITU-R BT.601

// 所有通道减去同一个亮度偏移
norm = (pixel - Y_mean) / 127.5

// 保留 R/G/B 比例差异（活体检测需要的颜色特征）
```

---

## 四、FreeRTOS 多任务调度

### 4.1 任务框架设计

系统采用统一的任务基类 `WhoTask`，封装 FreeRTOS 任务的创建、生命周期管理、事件同步：

```cpp
class WhoTaskBase {
public:
    // 事件标志位
    static constexpr EventBits_t TASK_STOPPED = 1 << 0;
    static constexpr EventBits_t TASK_PAUSED  = 1 << 1;
    static constexpr EventBits_t TASK_STOP    = 1 << 2;
    static constexpr EventBits_t TASK_PAUSE   = 1 << 3;
    static constexpr EventBits_t TASK_RESUME  = 1 << 4;

    // 生命周期控制
    virtual bool run(configSTACK_DEPTH_TYPE uxStackDepth,
                     UBaseType_t uxPriority, BaseType_t xCoreID);
    virtual bool stop();
    virtual bool pause();
    virtual bool resume();

protected:
    virtual void task() = 0;        // 子类实现的任务主循环
    EventGroupHandle_t m_event_group; // 状态同步事件组
    TaskHandle_t m_task_handle;       // 任务句柄
};

class WhoTask : public WhoTaskBase {
private:
    BaseType_t m_coreid;    // 运行的核心 (Core 0 / Core 1)
    SemaphoreHandle_t m_mutex;
};
```

**任务组（WhoTaskGroup）** 支持批量管理：

```cpp
class WhoTaskGroup {
public:
    void register_task(WhoTask *task);
    void register_task_group(WhoTaskGroup *task_group);
    void stop();      // 停止所有子任务
    void resume();    // 恢复所有子任务
    void pause();     // 暂停所有子任务
private:
    std::vector<WhoTask *> m_tasks;
    std::vector<WhoTaskGroup *> m_task_groups;
};
```

### 4.2 任务分配策略

ESP32-P4 为双核 RISC-V，任务按功能分配到不同核心：

| 任务 | 核心 | 栈深度 | 优先级 | 说明 |
|------|------|--------|--------|------|
| WhoFetchNode | Core 0 | 4KB | 高 | 摄像头帧采集，实时性要求高 |
| WhoDetect | Core 1 | 8KB | 高 | 人脸检测 CNN 推理 |
| WhoRecognitionCore | Core 1 | 8KB | 中 | 人脸识别（检测通过后才运行） |
| WhoFrameLCD | Core 0 | 4KB | 中 | LCD 显示渲染 |
| WhoRemoteMonitor | Core 0 | 8KB | 低 | JPEG 编码 + HTTP 上传 |
| WiFi/MQTT | Core 0 | 4KB | 低 | 网络通信 |
| VoiceCmd | Core 0 | 16KB | 中 | 语音唤醒 + ASR + LLM |

**设计原则：**
- **Core 1** 运行 AI 推理任务（检测+识别），避免被网络/显示任务干扰
- **Core 0** 运行 I/O 密集任务（摄像头、LCD、网络）
- 高优先级任务抢占低优先级任务，确保帧采集不丢帧

### 4.3 任务间通信

```
WhoFetchNode
    │
    ├──→ RingBuf (3帧缓冲) ──→ WhoDetect
    │                              │
    │                              ├──→ FreeRTOS Queue ──→ LivenessCNN
    │                              │
    │                              └──→ FreeRTOS Queue ──→ WhoRecognitionCore
    │
    ├──→ FreeRTOS Queue ──→ WhoFrameLCD
    │
    └──→ FreeRTOS Queue ──→ WhoRemoteMonitor
```

**RingBuf 三帧缓冲机制：**

```cpp
class RingBuf {
    // 环形缓冲区，默认深度 3 帧
    // 生产者 (WhoFetchNode) 写入最新帧
    // 消费者 (WhoDetect 等) 读取最新可用帧
    // 旧帧自动丢弃，保证消费者总是拿到最新数据
};
```

### 4.4 事件驱动状态机

任务通过 FreeRTOS EventGroup 实现状态同步：

```cpp
// 检测任务主循环
void WhoDetect::task() {
    while (true) {
        // 等待新帧事件（由 WhoFetchNode 发出）
        EventBits_t bits = xEventGroupWaitBits(
            m_frame_cap_node->get_event_group(),
            NEW_FRAME,        // 等待的事件位
            pdTRUE,           // 清除事件位
            pdFALSE,          // 任意一位即可
            portMAX_DELAY     // 无限等待
        );

        // 获取最新帧
        auto fb = m_frame_cap_node->get_frame();

        // 运行检测
        auto results = m_model->run(fb);

        // 回调通知下游
        if (m_result_cb) {
            m_result_cb({results, timestamp, img});
        }
    }
}
```

---

## 五、网络连接控制

### 5.1 WiFi 管理

`WhoWifi` 采用**单例模式**，全局唯一，通过状态机管理连接生命周期：

```cpp
class WhoWifi {
public:
    static WhoWifi *get_instance() {
        static WhoWifi instance;
        return &instance;
    }

    // 状态定义
    typedef enum {
        WIFI_STATE_INIT,          // 初始化
        WIFI_STATE_DISCONNECTED,  // 已断开
        WIFI_STATE_CONNECTING,    // 连接中
        WIFI_STATE_CONNECTED,     // 已连接
        WIFI_STATE_ERROR          // 错误
    } WifiState_t;

    esp_err_t init();                          // 初始化 WiFi 子系统
    esp_err_t connect(const char *ssid, const char *password, bool save = true);
    esp_err_t auto_connect();                  // 自动连接上次保存的 WiFi
    esp_err_t scan();                          // 扫描可用网络
    bool wait_connected(uint32_t timeout_ms);  // 阻塞等待连接成功

    void register_state_callback(WifiStateCallback callback);
    void register_ip_callback(WifiIPCallback callback);

private:
    EventGroupHandle_t m_event_group;
    static constexpr EventBits_t WIFI_CONNECTED_BIT    = BIT0;
    static constexpr EventBits_t WIFI_DISCONNECTED_BIT = BIT1;
    static constexpr EventBits_t WIFI_SCAN_DONE_BIT    = BIT2;
    static constexpr uint8_t WIFI_MAX_RECONNECT_RETRIES = 5;
};
```

**WiFi 初始化流程：**

```
NVS Flash 初始化 (保存配置)
    │
    ▼
TCP/IP 栈初始化
    │
    ▼
创建默认事件循环
    │
    ▼
创建 STA 网络接口
    │
    ▼
WiFi 初始化 (WIFI_INIT_CONFIG_DEFAULT)
    │
    ▼
设置 STA 模式
    │
    ▼
注册事件处理函数
    ├── WIFI_EVENT: 断开/连接/扫描
    └── IP_EVENT: 获取 IP 地址
    │
    ▼
自动连接保存的 WiFi (如果存在)
```

### 5.2 断线重连机制

```cpp
void WhoWifi::wifi_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data) {
    WhoWifi *self = static_cast<WhoWifi *>(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        auto *event = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected: reason=%s",
                 wifi_disconnect_reason_to_str(event->reason));

        if (self->m_reconnect_retry_count < WIFI_MAX_RECONNECT_RETRIES) {
            self->m_reconnect_retry_count++;
            esp_wifi_connect();  // 自动重连
        } else {
            self->set_state(WIFI_STATE_ERROR);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        self->m_reconnect_retry_count = 0;
        self->set_state(WIFI_STATE_CONNECTED);
        xEventGroupSetBits(self->m_event_group, WIFI_CONNECTED_BIT);
    }
}
```

### 5.3 配置持久化

WiFi 配置保存到 NVS Flash，断电不丢失：

```cpp
esp_err_t WhoWifi::connect(const char *ssid, const char *password, bool save) {
    // 保存到 NVS
    if (save) {
        WifiConfig_t cfg = {};
        strncpy(cfg.ssid, ssid, sizeof(cfg.ssid));
        strncpy(cfg.password, password, sizeof(cfg.password));
        save_config_to_nvs(cfg);
    }

    // 配置 WiFi
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect();
}
```

### 5.4 MQTT 智能家居

识别成功后通过 MQTT 上报事件，并支持远程控制：

```cpp
class WhoPassMqttClient {
    // Broker: mqtt://47.108.29.157:1883
    // 功能:
    //   - 上传访问日志 (JSON)
    //   - 上传抓拍照片 (Base64)
    //   - 接收临时密码管理命令
    //   - 接收远程注册/删除用户命令
};
```

### 5.5 远程监控

通过硬件 JPEG 编码器将摄像头画面编码后 HTTP POST 到云服务器：

```cpp
class WhoRemoteMonitor {
    // 1. 从 WhoFetchNode 获取帧
    // 2. PPA 硬件 RGB565→RGB888 转换
    // 3. 硬件 JPEG 编码器压缩
    // 4. HTTP POST 到 http://47.108.29.157:5000/api/monitor/frame
    // 帧率: ~10 FPS
};
```

---

## 六、内存管理

ESP32-P4 仅有 512KB 内部 RAM，关键配置：

```kconfig
# WiFi/LWIP 缓冲区分配到 PSRAM
CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y

# TLS 上下文分配到 PSRAM
CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y

# AFE/WakeNet 使用 PSRAM
CONFIG_AFE_MEMORY_ALLOC_MORE_PSRAM=y
```

**分区表：**

```
nvs         24K    — NVS 存储 (WiFi 配置等)
phy_init    4K     — PHY 初始化数据
factory     8000K  — 应用固件
model       500K   — SPIFFS (DL 模型文件)
voice_data  3200K  — FAT (WakeNet/ASR/TTS 模型数据)
storage     1M     — FAT (SD 卡仿真)
```

---

## 七、开发调试经验

### 7.1 活体检测调试历程

| 版本 | 问题 | 根因 | 修复 |
|------|------|------|------|
| V1 | 输出随机 | ESP-PPQ Softmax 索引交换 | 交换 data[0]/data[1] |
| V2 | 真人被判假 | 预处理对齐不一致 + 错误改了索引 | 回退索引 + 统一预处理 |
| V2.2 | 距离依赖漂移 | ISP AWB 自动调节 | 亮度均值减法 |
| V3 | ACER=0.147 饱和 | 模型容量不足 + 增强方向错误 | 升级 width_mult 0.5 |
| V5 | 鲁棒性提升 | R/B 通道增益增强 | 模拟 AWB 漂移 |

### 7.2 关键教训

1. **每次只改一个变量**：V2 同时改了预处理和索引，真正根因被掩盖
2. **增强要匹配硬件**：ISP 不活跃的模块（ACC/AEN）不需要增强，活跃的（AWB）要定向模拟
3. **训练-推理一致性**：裁剪方式、归一化方式必须完全一致
4. **实测是最终答案**：源码分析 ≠ 全链路验证

---

## 八、总结

本项目在 ESP32-P4 平台上实现了完整的边缘 AI 智能门锁系统：

1. **边缘计算流水线**：人脸检测→活体检测→人脸识别，全链路端侧推理，无需云端
2. **活体检测模型**：MobileNetV1 ×0.5，INT8 量化 ~160KB，定向 R/B 通道增益增强对抗 ISP 漂移
3. **FreeRTOS 多任务**：统一任务基类 + 事件驱动 + 双核分配，AI 推理与 I/O 任务隔离
4. **WiFi 联网**：单例管理 + 状态机 + NVS 持久化 + 自动重连
5. **MQTT 智能家居**：访问日志同步、远程控制、临时密码管理
6. **语音交互**：唤醒词→流式 ASR→LLM→TTS，支持自然语言控制门锁

---

**项目信息：**
- 芯片：ESP32-P4（双核 RISC-V, 512KB SRAM）
- 开发框架：ESP-IDF v5.5.3
- AI 推理：esp-dl ~3.2.0
- 活体模型：MobileNetV1 ×0.5, INT8 ~160KB
- 任务框架：FreeRTOS（事件驱动 + 任务组管理）
- 云服务：火山引擎 ASR/LLM + 自建 MQTT Broker
