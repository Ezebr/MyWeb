# 基于国产安路 FPGA 的智慧信号显示器——DAC 信号发生器设计与实现

> 以安路 EG4S20BG256 FPGA 为核心，实现数字存储示波器（ADC）与信号发生器（DAC）一体化系统。本文重点介绍信号发生器部分的 DDS 波形生成、幅度相位控制及多模交互设计。

## 一、项目概述

### 1.1 系统简介

本项目设计并实现了一款**智慧信号显示器**，集数字存储示波器与信号发生器于一体。系统以国产安路（Anlogic）EG4S20BG256 FPGA 为核心处理单元，通过高速 ADC（AD9248）完成信号采集与示波显示，通过高精度 DAC（DAC904）实现多种波形的信号生成，并支持语音控制与 WiFi 上位机远程操控。

该项目为嵌入式大赛 FPGA 赛道参赛作品。**我在项目中主要负责 DAC 信号发生器部分的开发**，包括 DDS 波形生成、频率/幅度/相位控制、UART 语音解析、WiFi 模块驱动等模块的设计与实现。

### 1.2 系统架构

```
                    ┌──────────────────────────────────────────┐
                    │          安路 EG4S20BG256 FPGA           │
                    │                                          │
  ┌──────────┐      │  ┌───────────┐      ┌──────────────┐    │      ┌──────────┐
  │ AD9248   │──14bit──│ ADC 采集   │──→  │  VGA 显示驱动 │──hsync/vsync──│ VGA 显示器│
  │ 双通道ADC │      │  │ 数据缓存   │      │  波形+参数   │    │      │          │
  └──────────┘      │  └───────────┘      └──────────────┘    │      └──────────┘
                    │                                          │
                    │  ┌───────────┐      ┌──────────────┐    │      ┌──────────┐
                    │  │ DDS 核心  │──→  │  波形选择     │──14bit──│  DAC904  │──→ 信号输出
                    │  │ 相位累加器 │      │  幅度/相位控制 │    │      │ 165Msps  │    SMA
                    │  └───────────┘      └──────────────┘    │      └──────────┘
                    │                                          │
  ┌──────────┐      │  ┌───────────────────────────────────┐   │
  │ 语音模块  │─UART─│  │         控制逻辑                  │   │
  │ WiFi 模块 │─UART─│  │  按键消抖 | 语音解析 | WiFi 初始化 │   │
  │ 按键/拨码 │──────│  └───────────────────────────────────┘   │
  └──────────┘      └──────────────────────────────────────────┘
```

### 1.3 主要性能指标

| 模块 | 参数 | 指标 |
|------|------|------|
| **ADC（示波器）** | 芯片型号 | AD9248，14 位双通道 |
| | 采样率 | 最高 50 MS/s |
| | 输入范围 | ±5V (10Vpp)，50Ω 输入阻抗 |
| **DAC（信号发生器）** | 芯片型号 | DAC904，14 位单通道 |
| | 更新率 | 165 Msps |
| | 输出波形 | 正弦波、方波、三角波 |
| | 频率范围 | 1KHz ~ 60MHz |
| | 输出幅度 | ±3.2V (6.4Vpp)，10 档可调 |
| **交互** | 控制方式 | 按键 + 语音 + WiFi 上位机 |

---

## 二、DDS 信号发生器设计

### 2.1 DDS 工作原理

DDS（Direct Digital Synthesis，直接数字频率合成）是信号发生器的核心技术。其基本思路是：

1. **相位累加器**每个时钟周期累加一个频率控制字（FCW）
2. 取累加器高位作为**波形 ROM 的查表地址**
3. ROM 输出对应的**波形采样值**送入 DAC
4. DAC 将数字量转换为**模拟电压输出**

```
         FCW (频率控制字)
          │
          ▼
  ┌───────────────┐    高12位     ┌──────────┐    14bit     ┌────────┐
  │  64位相位累加器 │ ──────────→ │  波形ROM  │ ──────────→ │  DAC   │ → 模拟波形
  │  Add += FCW    │   地址截取   │  4096点   │   采样数据   │ 904    │
  └───────────────┘              └──────────┘              └────────┘
```

**输出频率公式：**

```
f_out = FCW × f_clk / 2^N

FCW = 频率控制字
f_clk = DDS 系统时钟 = 180MHz（由 50MHz 经 PLL 倍频）
N = 累加器位宽 = 64 位
```

64 位累加器提供了极高的频率分辨率，理论上可达 μHz 级别。

### 2.2 相位累加器

相位累加器是 DDS 的核心模块，本质就是一个不断做加法的寄存器：

```verilog
module ROM_Addr_Add(
    input           Clk_I,       // 180MHz DDS 时钟
    input           Rst_N_I,
    input  [63:0]   Fre_W_I,     // 频率控制字
    output [11:0]   Addr_O       // 查表地址（取高12位）
);

reg [63:0] Add;

always @(posedge Clk_I) begin
    if (!Rst_N_I)
        Add <= 64'd0;
    else
        Add <= Add + Fre_W_I;    // 每个时钟周期累加一次
end

// 取高 12 位作为 ROM 地址，对应 4096 个采样点
assign Addr_O = Add[63:52];

endmodule
```

累加器溢出时自动回绕，形成周期性的相位扫描，对应一个完整的波形周期。

### 2.3 三种波形生成策略

#### 正弦波 — ROM 查表

正弦波数据预先计算好存入 FPGA 的 Block RAM（安路 IP 核 `EG_LOGIC_BRAM`），共 4096 个采样点，14 位量化：

```verilog
EG_LOGIC_BRAM #(
    .DATA_WIDTH_A(14),
    .ADDR_WIDTH_A(12),
    .DATA_DEPTH_A(4096),
    .MODE("SP"),
    .IMPLEMENT("9K"),
    .INIT_FILE("../mif/SineWaveData.mif")   // MATLAB 生成的正弦数据
) inst (
    .addra(addra),
    .doa(doa),
    .clka(clka),
    .wea(1'b0),     // 只读
    // ...
);
```

#### 三角波 — 线性计算

三角波不需要 ROM，直接利用地址的线性关系生成，节省 BRAM 资源：

```verilog
// Rom_Addr[11] = 0：上升沿（0→满幅）
// Rom_Addr[11] = 1：下降沿（满幅→0）
assign wave_Triangular = Rom_Addr[11]
    ? {11'd2047 - Rom_Addr[10:0], 3'b000}   // 下降
    : {Rom_Addr[10:0], 3'b000};              // 上升
```

高 11 位做线性映射，低 3 位补零扩展到 14 位。

#### 方波 — 阈值判断

方波最简单，根据相位的最高位判断输出满幅或零：

```verilog
assign wave_Square = Rom_Addr[11]
    ? 14'b11_1111_1111_1111   // 满幅
    : 14'b00_0000_0000_0000;  // 零
```

### 2.4 频率控制字计算

系统时钟 50MHz 经 PLL 倍频至 180MHz 作为 DDS 工作时钟：

```verilog
// PLL: 50MHz × 18 / 5 = 180MHz
EG_PHY_PLL #(
    .FIN("50.000000"),
    .REFCLK_DIV(5),
    .FBCLK_DIV(18),
    .CLKC0_DIV(6)
) pll_inst ( ... );
```

频率控制字 FCW 的计算：

```
FCW = f_out × 2^64 / f_clk

1KHz  → FCW ≈ 102,481,911,520,608
10KHz → FCW ≈ 1,024,819,115,206,080
1MHz  → FCW ≈ 10,248,191,152,060,800
60MHz → FCW ≈ 614,891,469,123,651,720,500
```

频率控制模块采用**多档位加速**策略——频率越高步进越大，兼顾低频精细调节和高频快速切换：

```verilog
module Fre_ControlWord (
    input           Clk_I,
    input           Rst_N_I,
    input           Key_Fre_I,
    input  [3:0]    voice_i,
    output [63:0]   Fre_W_O
);

reg [63:0] Fre;

// 按键消抖
Key_Delay u_Key_Delay(.Clk_I(Clk_I), .Key_I(Key_Fre_I), .Key_O(Key_Out));

// 语音控制与按键控制取或
assign Sum_Ctrl = ~(Voice_Switch ^ Key_Out);

always @(posedge Sum_Ctrl or negedge Rst_N_I) begin
    if (!Rst_N_I)
        Fre <= 64'd102481911520608;             // 默认 1KHz
    else begin
        if      (Fre < 64'd1024819115206080)     // < 10KHz
            Fre <= Fre + 64'd102481911520608;    // 步进 1KHz
        else if (Fre < 64'd10248191152060800)    // < 100KHz
            Fre <= Fre + 64'd1024819115206080;   // 步进 10KHz
        else if (Fre < 64'd102481911520608000)   // < 1MHz
            Fre <= Fre + 64'd10248191152060800;  // 步进 100KHz
        else if (Fre < 64'd6148914691236517205)  // < 60MHz
            Fre <= Fre + 64'd102481911520608000; // 步进 1MHz
        else
            Fre <= 64'd102481911520608;          // 溢出回绕
    end
end

assign Fre_W_O = Fre;
endmodule
```

### 2.5 波形选择与幅度缩放

波形选择模块负责在三种波形间切换，并实现幅度的 10 档缩放：

```verilog
module Sel_WaveType #(parameter AMP_ZERO = 14'd8192) (
    input           Clk_I,
    input           Rst_N_I,
    input           Key_Wave_I,         // 波形切换
    input           Key_Amp_I,          // 幅度调节
    input           Switch_I,           // 加减方向
    input   [3:0]   voice_i,
    input   [13:0]  Wave_Sine_I,
    input   [13:0]  wave_Triangular_I,
    input   [13:0]  wave_Square_I,
    output  [13:0]  Dac_Scale_O,
    output  [3:0]   scale_o
);

reg [1:0]  Sel;      // 1=正弦, 2=三角, 3=方波
reg [3:0]  Scale;    // 幅度档位 1~10

// 波形循环切换
always @(posedge Sum_ctrl1 or negedge Rst_N_I) begin
    if (!Rst_N_I)       Sel <= 2'd1;
    else if (Sel == 2'd3) Sel <= 2'd1;
    else                Sel <= Sel + 2'd1;
end

// 幅度缩放：以 8192 (14位中值) 为零点，正负半周分别缩放
always @(posedge Clk_I or negedge Rst_N_I) begin
    case (Sel)
        2'd1: Dac_Data_Reg_i <= Wave_Sine_I;
        2'd2: Dac_Data_Reg_i <= wave_Triangular_I;
        2'd3: Dac_Data_Reg_i <= wave_Square_I;
    endcase

    if (Dac_Data_Reg_i > AMP_ZERO)
        Dac_Data_Reg <= (Dac_Data_Reg_i - AMP_ZERO) * Scale / 10 + AMP_ZERO;
    else
        Dac_Data_Reg <= AMP_ZERO - (AMP_ZERO - Dac_Data_Reg_i) * Scale / 10;
end
```

| Scale | 缩放比例 | 输出幅度 |
|-------|---------|---------|
| 10 | 100% | ±3.2V (6.4Vpp) |
| 5 | 50% | ±1.6V (3.2Vpp) |
| 1 | 10% | ±0.32V (0.64Vpp) |

### 2.6 相位偏移

相位偏移模块在波形数据上叠加可控的直流偏移，偏移上限随幅度自适应：

```verilog
module Translation #(parameter AMP_STEP = 14'd256) (
    input  [13:0] Dac_Data,
    input  [3:0]  scale,
    input         Key_Trans_I,
    input         Switch_I,
    input  [3:0]  voice_i,
    output [13:0] Dac_Data_O
);

reg [6:0] cnt;
reg       symbol;   // 0=正方向, 1=负方向

// 偏移上限随幅度档位变化，幅度越小可偏移范围越大
assign max_cnt = (10 - scale) * AMP_ZERO / (10 * AMP_STEP);

// 输出 = 波形 ± 步进 × 计数，限幅保护
always @(posedge Clk_I or negedge Rst_N_I) begin
    Dac_Data_Reg <= (symbol == 0)
        ? {6'd0, Dac_Data} + AMP_STEP * cnt
        : {6'd0, Dac_Data} - AMP_STEP * cnt;

    if (Dac_Data_Reg > AMP_MAX) Dac_Data_Reg <= AMP_MAX;
    if (Dac_Data_Reg < AMP_MIN) Dac_Data_Reg <= AMP_MIN;
end
```

---

## 三、控制接口设计

### 3.1 语音控制（UART 接收）

天问语音模块通过 UART 发送识别结果，FPGA 端用状态机解析串口数据，将接收到的字节映射为 4 位控制码：

```verilog
module uart_rx #(
    parameter CLK_FRE = 50,
    parameter BAUD_RATE = 115200
)(
    input            clk,
    input            rst_n,
    input            rx_pin,
    output reg [3:0] ctrl
);

// 状态机：IDLE → START → REC_BYTE → STOP → DATA
// 在 STOP→DATA 的跳变沿锁存数据

// 控制码映射
always @(posedge clk or negedge rst_n) begin
    if (state == S_DATA)
        case (rx_data)
            8'd1: ctrl <= 4'd1;   // 增大频率
            8'd2: ctrl <= 4'd2;   // 改变波形
            8'd3: ctrl <= 4'd3;   // 增大振幅
            8'd4: ctrl <= 4'd4;   // 减小振幅
            8'd5: ctrl <= 4'd5;   // 正向相移
            8'd6: ctrl <= 4'd6;   // 负向相移
            8'd7: ctrl <= 4'd7;   // WiFi 重置
            default: ctrl <= 4'd0;
        endcase
    else
        ctrl <= 4'd0;   // 脉冲输出，非锁存
end
```

### 3.2 WiFi 远程控制

WiFi 模块上电后通过 FPGA 发送 AT 指令初始化为 TCP 服务器模式，上位机通过局域网连接后即可发送控制命令：

```verilog
module Wifi_Init #(parameter CLK_FRE = 50, parameter BAUD_RATE = 115200)(
    input          clk,
    input          rst_n,
    input  [3:0]   voice_i,
    output         tx_pin
);

// 初始化流程：
// 阶段1: AT+CIPMUX=1         (启用多连接)
// 阶段2: 等待 4 秒
// 阶段3: AT+CIPSERVER=1,8080  (开启 TCP 服务器，端口 8080)

// AT 指令逐字节发送
always @(*) begin
    case (tx_cnt)
        8'd0:  tx_str <= "A";
        8'd1:  tx_str <= "T";
        8'd2:  tx_str <= "+";
        8'd3:  tx_str <= "C";
        8'd4:  tx_str <= "I";
        8'd5:  tx_str <= "P";
        8'd6:  tx_str <= "M";
        8'd7:  tx_str <= "U";
        8'd8:  tx_str <= "X";
        8'd9:  tx_str <= "=";
        8'd10: tx_str <= "1";
        8'd11: tx_str <= 8'h0d;  // \r
        8'd12: tx_str <= 8'h0a;  // \n
        // ... AT+CIPSERVER 指令
    endcase
end
```

### 3.3 按键消抖

所有物理按键经过硬件消抖处理，采用计数器方式，阈值约 20ms：

```verilog
module Key_Delay(
    input       Clk_I,
    input       Key_I,
    output reg  Key_O
);

parameter [31:0] Delay = 4000000;  // 180MHz 下约 22ms

always @(posedge Clk_I) begin
    if (!Key_I) Key_L <= Key_L + 1'b1;
    else        Key_L <= 0;
end

always @(posedge Clk_I) begin
    if (Key_L > Delay) Key_O <= 1'b0;
    if (Key_H > Delay) Key_O <= 1'b1;
end
endmodule
```

---

## 四、顶层集成与数据流

顶层模块将 DAC 信号发生器与 ADC 示波器、VGA 显示集成在同一 FPGA 中：

```verilog
module TOP (
    input         sys_clk,        // 50MHz
    input         sys_rst_n,
    // --- ADC 示波器部分 ---
    input  [13:0] Adc_In,         // AD9248 采集数据
    output        Adc_Clk_A,      // ADC 采样时钟
    output        Adc_Clk_B,
    output        hsync, vsync,
    output [11:0] vga_rgb,        // VGA 显示输出
    // --- DAC 信号发生器部分 ---
    input         Uart_Key,       // 语音串口
    input         Wifi_Key,       // WiFi 串口
    input         Key_Fre,        // 频率按键
    input         Key_Wave,       // 波形按键
    input         Key_Amp,        // 幅度按键
    input         Key_Trans,      // 相位按键
    input         Switch1,        // 幅度方向
    input         Switch2,        // 相位方向
    output        Uart_Wifi,      // WiFi 串口发送
    output        Dac_Clk,        // DAC 时钟 (180MHz)
    output [13:0] Dac_Data        // DAC 数据 (14位)
);

// ====== ADC 示波器部分 ======
AD9248 AD9248_inst(
    .sys_clk(sys_clk), .sys_rst_n(sys_rst_n),
    .Adc_In(Adc_In), .Adc_Clk_A(Adc_Clk_A), .Adc_Clk_B(Adc_Clk_B),
    .CHA_DATA(CHA_DATA), .CHB_DATA(CHB_DATA)
);
// ... VGA 显示、触发、参数计算等

// ====== DAC 信号发生器部分（本文重点） ======
// DAC 数据取反，匹配 DAC904 输出极性
assign Dac_Data = 14'h3FFF - Dac_Data_Wire;

DAC904 U_DAC(
    .Ext_Rst_n(sys_rst_n),
    .Ext_Clk  (sys_clk),
    .Uart_Key (Uart_Key),
    .Wifi_Key (Wifi_Key),
    .Key_Fre  (Key_Fre),
    .Key_Wave (Key_Wave),
    .Key_Amp  (Key_Amp),
    .Key_Trans(Key_Trans),
    .Switch1  (Switch1),
    .Switch2  (Switch2),
    .Uart_Wifi(Uart_Wifi),
    .Dac_Clk  (Dac_Clk),
    .Dac_Data (Dac_Data_Wire)
);
endmodule
```

DAC 子系统的内部数据流：

```
PLL (50→180MHz)
    │
    ├─→ Fre_ControlWord  → 频率控制字 FCW
    │        ↑
    │    Key_Fre / Voice
    │
    ├─→ ROM_Addr_Add     → 12位相位地址
    │        │
    │        ├─→ Rom_SineData   → 正弦波 (ROM查表)
    │        ├─→ wave_Triangular → 三角波 (线性计算)
    │        └─→ wave_Square     → 方波   (阈值判断)
    │
    ├─→ Sel_WaveType     → 波形选择 + 幅度缩放
    │        ↑
    │    Key_Wave / Key_Amp / Switch1 / Voice
    │
    ├─→ Translation      → 相位偏移 + 限幅保护
    │        ↑
    │    Key_Trans / Switch2 / Voice
    │
    └─→ DAC904 输出      → 14位并行数据 → 模拟信号
```

---

## 五、开发调试经验

### 5.1 开发环境

- **IDE**: 安路 TangDynasty (TD) v6.2
- **FPGA**: EG4S20BG256（20K LUT，国产）
- **语言**: Verilog HDL
- **IP 核**: EG_PHY_PLL（时钟）、EG_LOGIC_BRAM（ROM）、FIFO（缓存）

### 5.2 关键调试问题

**1. DAC 数据极性**

DAC904 的输出与输入数据呈反码关系，需要在顶层做取反处理：

```verilog
assign Dac_Data = 14'h3FFF - Dac_Data_Wire;
```

如果不取反，输出波形会翻转（正弦波变成倒相正弦波）。

**2. PLL 锁定时序**

上电后 PLL 需要一定时间锁定，必须等待 `locked` 信号拉高后再释放系统复位，否则 DDS 时钟不稳定，输出频率会跳变。

**3. 按键消抖阈值**

180MHz 时钟下，消抖计数器阈值设为 4000000（约 22ms）。过短会导致误触发（一次按键被识别为多次），过长会导致操作迟钝。

**4. 频率步进设计**

低频段步进太大会导致频率调节不精细，高频段步进太小会导致切换太慢。采用分段递增策略解决：

| 频率范围 | 步进 | 调节体验 |
|---------|------|---------|
| < 10KHz | 1KHz | 精细 |
| 10KHz ~ 100KHz | 10KHz | 适中 |
| 100KHz ~ 1MHz | 100KHz | 较快 |
| 1MHz ~ 60MHz | 1MHz | 快速 |

**5. 幅度缩放的定点运算**

`Dac_Data_Reg * Scale / 10` 在综合时会生成乘法器和除法器。如果资源紧张，可以将 `/10` 曉换为 `>>3`（除以 8）做近似，或使用移位+加法组合实现除以 10。

---

## 六、总结

本项目在国产安路 FPGA 平台上实现了完整的 DDS 信号发生器系统，我主要完成了以下工作：

1. **DDS 核心**：64 位相位累加器 + 4096 点 ROM 查表，输出频率 1KHz~60MHz
2. **多波形生成**：正弦波（ROM）、三角波（线性计算）、方波（阈值判断）
3. **幅度/相位控制**：10 档幅度缩放 + 连续相位偏移，支持语音和按键控制
4. **UART 语音解析**：状态机串口接收，指令映射为控制码
5. **WiFi 远程控制**：AT 指令驱动 ESP 模块，局域网 TCP 服务器
6. **顶层集成**：DAC 子系统与 ADC 示波器、VGA 显示在同一 FPGA 内协同工作

系统实测可输出稳定的正弦波、方波、三角波，频率和幅度均可通过语音或上位机远程调节。该项目作为嵌入式大赛 FPGA 赛道的参赛作品，验证了国产安路 FPGA 在混合信号系统中的工程可行性。

---

**项目信息：**
- 竞赛：嵌入式大赛 FPGA 赛道
- FPGA：安路 EG4S20BG256
- DAC：TI DAC904 (14-bit, 165Msps)
- ADC：AD9248 (14-bit, 65Msps，示波器部分)
- 开发工具：Anlogic TangDynasty v6.2
