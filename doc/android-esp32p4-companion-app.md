# Android 智能门锁配套 App：MQTT 双向通信与远程监控实现

> 基于 Eclipse Paho MQTT + OkHttp WebSocket + MJPEG 流解析，实现 Android 端与 ESP32-P4 智能门锁的实时双向通信、远程视频监控、语音对讲等功能。

## 一、系统架构

### 1.1 整体通信架构

```
┌──────────────┐    MQTT (pub/sub)    ┌──────────────┐    MQTT (pub/sub)    ┌──────────────┐
│  Android App │ ◄──────────────────► │  MQTT Broker │ ◄──────────────────► │  ESP32-P4    │
│  (本项目)     │    tcp://47.108...   │  (云端中转)   │                      │  (智能门锁)   │
└──────┬───────┘                      └──────────────┘                      └──────────────┘
       │
       │  HTTP REST / MJPEG Stream / WebSocket
       ▼
┌──────────────┐
│  Flask Server│  ← 照片存储、视频转发、访问日志
│  (云端中转)   │
└──────────────┘
```

**三层通信设计：**

| 层级 | 协议 | 用途 | 特点 |
|------|------|------|------|
| 控制层 | MQTT | 门锁控制、通行证管理、成员管理 | 双向、实时、QoS 1 |
| 数据层 | HTTP REST | 访问日志、照片获取 | 请求-响应、可靠 |
| 媒体层 | MJPEG + WebSocket | 视频监控、语音对讲 | 流式、低延迟 |

### 1.2 App 页面导航

```
MainActivity (登录)
  └─→ ControlActivity (控制面板)
        ├─→ GeneratePassActivity  — 临时通行证 (MQTT 生成 QR 码)
        ├─→ AccessLogActivity     — 访问日志 (HTTP REST)
        │     └─→ AccessLogDetailActivity — 日志详情 + 照片
        ├─→ EnrollListActivity    — 注册成员管理 (MQTT)
        │     └─→ EnrollDetailActivity — 成员详情 + 删除
        └─→ MonitorActivity       — 远程视频监控 (MJPEG + WebSocket 对讲)
```

---

## 二、MQTT 双向通信

### 2.1 MqttManager 单例

采用**单例模式**管理全局唯一的 MQTT 连接，支持多 Activity 共享：

```java
public class MqttManager {
    private static final String BROKER_URL = "tcp://47.108.29.157:1883";
    private static final String CLIENT_ID = "SML_ESP32P4_Android_" + System.currentTimeMillis();

    private static MqttManager instance;
    private MqttAndroidClient mqttClient;
    private boolean isConnected = false;
    private List<MqttCallback> callbacks = new ArrayList<>();  // 多回调支持

    public static synchronized MqttManager getInstance(Context context) {
        if (instance == null) {
            instance = new MqttManager(context);
        }
        return instance;
    }
}
```

### 2.2 主题设计

MQTT 主题采用 `sml/` 命名空间，按功能分层：

```
sml/
├── pass/
│   ├── generate        ← App 发送通行证生成请求
│   └── response        ← ESP32 返回通行证响应
├── control/
│   ├── command         ← App 发送控制命令
│   └── status          ← ESP32 上报设备状态
├── enroll/
│   ├── list            ← App 请求/ESP32 返回成员列表
│   ├── delete          ← App 发送删除请求
│   ├── delete/resp     ← ESP32 返回删除结果
│   ├── photo/get/{id}  ← App 触发设备上传照片
│   └── photo/resp/{id} ← ESP32 照片上传结果
└── access/
    ├── photo/get/{idx}  ← App 触发设备上传通行照片
    └── photo/resp/{idx} ← ESP32 照片上传结果
```

### 2.3 连接与订阅

连接时自动订阅所有响应主题，断线时通知所有回调：

```java
public void connect(IMqttActionListener connectCallback) {
    // 防止重复连接
    if (mqttClient.isConnected()) {
        isConnected = true;
        connectCallback.onSuccess(null);
        return;
    }

    MqttConnectOptions options = new MqttConnectOptions();
    options.setAutomaticReconnect(false);   // 由 MqttManager 统一管理重连
    options.setCleanSession(true);
    options.setConnectionTimeout(10);
    options.setKeepAliveInterval(60);
    options.setUserName("admin");
    options.setPassword("Melles2002".toCharArray());

    mqttClient.connect(options, null, new IMqttActionListener() {
        @Override
        public void onSuccess(IMqttToken token) {
            isConnected = true;
            subscribeToTopics();   // 连接成功后自动订阅
            connectCallback.onSuccess(token);
        }

        @Override
        public void onFailure(IMqttToken token, Throwable exception) {
            isConnected = false;
            connectCallback.onFailure(token, exception);
        }
    });
}

private void subscribeToTopics() {
    String[] topics = {
        "sml/pass/response",
        "sml/control/status",
        "sml/enroll/list",
        "sml/enroll/delete/resp"
    };
    for (String topic : topics) {
        mqttClient.subscribe(topic, 1, null, null);  // QoS 1
    }
}
```

### 2.4 多回调机制

支持多个 Activity 同时监听 MQTT 消息，避免单点回调的局限：

```java
private List<MqttCallback> callbacks = new ArrayList<>();

mqttClient.setCallback(new MqttCallback() {
    @Override
    public void messageArrived(String topic, MqttMessage message) {
        // 分发给所有注册的回调
        for (MqttCallback cb : callbacks) {
            cb.messageArrived(topic, message);
        }
    }

    @Override
    public void connectionLost(Throwable cause) {
        isConnected = false;
        for (MqttCallback cb : callbacks) {
            cb.connectionLost(cause);
        }
    }
});

// Activity 注册/注销回调
public void addCallback(MqttCallback callback) {
    if (!callbacks.contains(callback)) callbacks.add(callback);
}
public void removeCallback(MqttCallback callback) {
    callbacks.remove(callback);
}
```

### 2.5 消息发送

所有发送方法统一检查连接状态，QoS 1 保证至少送达一次：

```java
public void sendPassData(String passData, IMqttActionListener callback) {
    if (!isConnected) {
        callback.onFailure(null, new MqttException(MqttException.REASON_CODE_CLIENT_NOT_CONNECTED));
        return;
    }
    MqttMessage message = new MqttMessage(passData.getBytes("UTF-8"));
    message.setQos(1);
    mqttClient.publish("sml/pass/generate", message, null, callback);
}

public void sendControlCommand(String command, IMqttActionListener callback) {
    // 同样的模式，发布到 sml/control/command
    mqttClient.publish("sml/control/command", message, null, callback);
}
```

---

## 三、远程视频监控

### 3.1 MJPEG 流解析

`MonitorActivity` 通过 HTTP 长连接接收 MJPEG 流，使用**自定义状态机**解析帧边界：

```
MJPEG 流格式：
--frame\r\n
Content-Type: image/jpeg\r\n
\r\n
<JPEG 数据>\r\n
--frame\r\n
Content-Type: image/jpeg\r\n
\r\n
<JPEG 数据>\r\n
...
```

**状态机设计：**

```java
private class MjpegRunner implements Runnable {
    private static final int STATE_NEED_BOUNDARY = 0;   // 等待帧边界
    private static final int STATE_NEED_HEADER_END = 1; // 等待头结束
    private static final int STATE_READING_FRAME = 2;   // 读取 JPEG 数据

    @Override
    public void run() {
        HttpURLConnection conn = (HttpURLConnection) new URL(STREAM_URL).openConnection();
        conn.setConnectTimeout(5000);
        conn.setReadTimeout(15000);

        byte[] boundary = "frame".getBytes();
        byte[] CRLFCRLF = "\r\n\r\n".getBytes();
        byte[] buf = new byte[131072];  // 128KB 缓冲区

        int state = STATE_NEED_BOUNDARY;
        int frameStart = 0;

        while (!mStopStream) {
            int n = is.read(buf, total, buf.length - total);
            total += n;

            // 状态机：在缓冲区中搜索边界和帧数据
            while (searchFrom <= total - boundary.length) {
                int bi = indexOfBytes(buf, searchFrom, total, boundary);

                if (state == STATE_READING_FRAME) {
                    // 找到下一个边界 → 当前帧结束
                    byte[] jpg = Arrays.copyOfRange(buf, frameStart, bi - 2);
                    decodeAndDisplay(jpg);  // 解码并显示
                    state = STATE_NEED_BOUNDARY;
                }

                // 跳过 header，定位 JPEG 数据起始
                int he = indexOfBytes(buf, searchFrom, total, CRLFCRLF);
                frameStart = he + 4;
                state = STATE_READING_FRAME;
            }
        }
    }
}
```

### 3.2 帧解码与显示

使用 Android 原生 `BitmapFactory` 解码 JPEG，通过 `runOnUiThread` 更新 UI：

```java
private void decodeAndDisplay(byte[] jpeg) {
    Bitmap bmp = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
    if (bmp != null) {
        runOnUiThread(() -> mFrameView.setImageBitmap(bmp));
    }
}
```

### 3.3 MQTT 控制视频流

视频流的开启/停止通过 MQTT 命令控制 ESP32 端：

```java
// 开启监控
JSONObject cmd = new JSONObject();
cmd.put("command", "monitor.start");
mqttManager.sendControlCommand(cmd.toString(), callback);

// 关闭监控
cmd.put("command", "monitor.stop");
mqttManager.sendControlCommand(cmd.toString(), callback);
```

**流程：**

```
App 打开 MonitorActivity
    │
    ├──→ MQTT: {"command":"monitor.start"} ──→ ESP32 开始推流
    │
    └──→ HTTP GET /monitor/stream ──→ 接收 MJPEG 流

App 关闭 MonitorActivity
    │
    ├──→ MQTT: {"command":"monitor.stop"} ──→ ESP32 停止推流
    │
    └──→ 断开 HTTP 连接
```

### 3.4 全屏横屏切换

支持沉浸式全屏模式，隐藏系统栏，自动切换横屏：

```java
private void enterFullscreen() {
    // 隐藏系统栏
    WindowInsetsController controller = window.getInsetsController();
    controller.hide(WindowInsets.Type.systemBars());
    controller.setSystemBarsBehavior(
        WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);

    // 切换横屏
    setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
}
```

---

## 四、WebSocket 语音对讲

### 4.1 按住说话（Push-to-Talk）

`MonitorActivity` 支持按住麦克风按钮实时对讲，松开停止：

```java
mBtnMic.setOnTouchListener((v, event) -> {
    switch (event.getAction()) {
        case MotionEvent.ACTION_DOWN:
            startAudioRecording();   // 按下开始录音+推流
            return true;
        case MotionEvent.ACTION_UP:
        case MotionEvent.ACTION_CANCEL:
            stopAudioRecording();    // 松开停止
            return true;
    }
    return false;
});
```

### 4.2 音频采集与 WebSocket 推流

```java
private static final int AUDIO_SAMPLE_RATE = 16000;  // 16kHz，匹配 ESP32 扬声器
private static final int AUDIO_CHUNK_MS = 100;        // 100ms 一帧
private static final int AUDIO_CHUNK_BYTES = 3200;     // 1600 samples × 2 bytes

private void startAudioRecording() {
    // 1. 初始化 AudioRecord
    mAudioRecord = new AudioRecord(
        MediaRecorder.AudioSource.MIC,
        AUDIO_SAMPLE_RATE, AudioFormat.CHANNEL_IN_MONO,
        AudioFormat.ENCODING_PCM_16BIT, bufferSize);

    // 2. 连接 WebSocket
    OkHttpClient client = new OkHttpClient.Builder()
        .connectTimeout(3, TimeUnit.SECONDS).build();
    Request request = new Request.Builder()
        .url("ws://47.108.29.157:5000/ws/phone_audio").build();
    mAudioWs = client.newWebSocket(request, wsListener);

    // 3. 启动录音+推流线程
    mAudioRecord.startRecording();
    mAudioThread = new Thread(this::audioStreamingLoop);
    mAudioThread.start();
}

private void audioStreamingLoop() {
    byte[] chunk = new byte[AUDIO_CHUNK_BYTES];
    while (mIsRecording) {
        int read = mAudioRecord.read(chunk, 0, AUDIO_CHUNK_BYTES);
        if (read > 0 && mAudioWs != null) {
            mAudioWs.send(ByteString.of(chunk));  // 二进制帧发送
        }
    }
}
```

**音频参数匹配：**

| 参数 | App 端 | ESP32 扬声器端 |
|------|--------|---------------|
| 采样率 | 16000 Hz | 16000 Hz |
| 声道 | 单声道 | 单声道 |
| 位深 | 16-bit PCM | 16-bit PCM |
| 帧长 | 100ms (3200 bytes) | — |

---

## 五、临时通行证系统

### 5.1 QR 码生成流程

```
App 端：
  用户设置有效期 + 使用次数
      │
      ▼
  MQTT: sml/pass/generate
  {"start":"2026-06-07 09:00","end":"2026-06-07 18:00","limit":5}
      │
      ▼
  ESP32 端生成通行证数据
      │
      ▼
  MQTT: sml/pass/response
  {"pass_id":"xxx","qr_data":"..."}
      │
      ▼
  App 端生成 QR 码图片 (ZXing)
  用户可保存到相册
```

### 5.2 QR 码生成与保存

```java
// 使用 ZXing 生成 QR 码
Bitmap qrBitmap = QRCodeUtils.createQRCodeBitmap(passData, 400, 400);

// 保存到相册
ContentValues values = new ContentValues();
values.put(MediaStore.Images.Media.DISPLAY_NAME, "pass_" + timestamp + ".jpg");
values.put(MediaStore.Images.Media.MIME_TYPE, "image/jpeg");
OutputStream out = getContentResolver().insert(uri, values);
qrBitmap.compress(Bitmap.CompressFormat.JPEG, 90, out);
```

---

## 六、访问日志与照片管理

### 6.1 访问日志获取

通过 HTTP REST 从 Flask 服务器获取访问日志列表：

```java
// HTTP GET 请求访问日志
OkHttpClient client = new OkHttpClient();
Request request = new Request.Builder()
    .url("http://47.108.29.157:5000/api/access_log")
    .build();
Response response = client.newCall(request).execute();
String json = response.body().string();
// 解析为 AccessLogItem 列表
```

### 6.2 照片获取的 MQTT+HTTP 桥接

照片获取采用**MQTT 触发 + HTTP 获取**的桥接模式：

```
App                          MQTT Broker                    ESP32-P4
 │                              │                              │
 │  sml/enroll/photo/get/{id}   │                              │
 │ ─────────────────────────────│─────────────────────────────►│
 │                              │                              │  上传照片到 Flask
 │                              │                              │ ──────────────► Flask
 │                              │  sml/enroll/photo/resp/{id}  │
 │ ◄────────────────────────────│◄─────────────────────────────│
 │                              │                              │
 │  HTTP GET /api/photo/{id}    │                              │
 │ ──────────────────────────────────────────────────────────────────────────────► Flask
 │ ◄──────────────────────────────────────────────────────────────────────────────│
 │  JPEG 图片                                                    │
 │  Glide 加载显示                                               │
```

**设计原因：** ESP32-P4 无法直接被 App 访问（NAT 穿透困难），通过 MQTT 触发设备主动上传到公网 Flask 服务器，App 再从 Flask 获取。

---

## 七、网络连接管理

### 7.1 连接生命周期

```
App 启动 → MainActivity 登录
    │
    ▼
ControlActivity 创建
    │
    ├── MqttManager.getInstance() → 单例初始化
    │
    ├── mqttManager.connect() → 连接 Broker
    │       │
    │       ├── onSuccess → subscribeToTopics()
    │       └── onFailure → 日志记录
    │
    └── 各功能页面使用同一 MqttManager 实例

App 退出 → MainActivity.onDestroy()
    │
    ▼
mqttManager.release()  → disconnect() → close() → instance = null
```

### 7.2 断线处理

```java
// connectionLost 回调
@Override
public void connectionLost(Throwable cause) {
    isConnected = false;
    // 通知所有注册的 Activity
    for (MqttCallback cb : callbacks) {
        cb.connectionLost(cause);
    }
}

// MonitorActivity 中的重连检查
@Override
protected void onResume() {
    if (mqttManager != null && !mqttManager.isConnected()) {
        connectAndSendMonitorStart();  // 重新连接并发送 monitor.start
    }
}
```

### 7.3 权限处理

Android 12+ 需要精确闹钟权限（MQTT 长连接保活），麦克风权限（语音对讲）：

```java
// 麦克风权限
if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
        != PackageManager.PERMISSION_GRANTED) {
    ActivityCompat.requestPermissions(this,
        new String[]{Manifest.permission.RECORD_AUDIO}, RC_RECORD_AUDIO);
    return;
}
```

---

## 八、依赖库

| 库 | 版本 | 用途 |
|----|------|------|
| Eclipse Paho MQTT Client | 1.2.5 | MQTT 协议客户端 |
| Paho Android Service | 1.1.1 | Android MQTT 后台服务 |
| OkHttp | — | HTTP 客户端 + WebSocket |
| ZXing Core | 3.5.1 | QR 码生成 |
| zxing-android-embedded | 4.3.0 | QR 码扫描/生成 UI |
| Glide | 4.16.0 | 图片加载（照片显示） |
| AndroidX Material | 1.10.0 | UI 组件 |

---

## 九、总结

本 App 作为 ESP32-P4 智能门锁的 Android 配套应用，实现了完整的远程控制与监控功能：

1. **MQTT 双向通信**：单例管理 + 多回调机制 + QoS 1 保证，实现门锁控制、通行证管理、成员管理的实时双向通信
2. **远程视频监控**：自定义状态机解析 MJPEG 流，支持全屏横屏、实时显示
3. **WebSocket 语音对讲**：按住说话 + PCM 16kHz 实时推流，与 ESP32 扬声器联动
4. **MQTT+HTTP 桥接**：MQTT 触发设备上传照片 → Flask 存储 → App HTTP 获取，解决 NAT 穿透问题
5. **临时通行证**：QR 码生成 + 有效期管理 + 使用次数控制

---

**项目信息：**
- 平台：Android (minSdk 24, targetSdk 36)
- 语言：Java 11
- 构建：Gradle Kotlin DSL
- 配套硬件：ESP32-P4 Function EV Board
- MQTT Broker：mqtt://47.108.29.157:1883
- 媒体服务器：http://47.108.29.157:5000
