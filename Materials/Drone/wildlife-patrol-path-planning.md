# 2025 电赛 H 题飞控端算法：BFS 最短路径规划与全覆盖巡查

> 基于 CarryPilot 飞控平台，在 9×7 网格地图上实现禁飞区避障 + 蛇形全覆盖路径规划 + BFS 最短路径连接，满足无人机野生动物巡查赛题要求。

## 一、赛题分析

2025 年全国大学生电子设计竞赛 H 题（野生动物巡查系统）的核心要求：

| 要求 | 内容 |
|------|------|
| 基本要求(1) | 赛前在地面站设置禁飞区，显示屏按 9×7 方格画出巡查航线，航线需**覆盖禁飞区以外所有方格** |
| 基本要求(2) | 无人机从红色起飞区起飞，在 120±10cm 高度按规划航线巡查，不得偏离航线，300s 内完成 |
| 基本要求(3) | 发现野生动物时识别种类及数量，将方格代码、动物名称及数量发送到地面站显示并保存 |
| 基本要求(4) | 巡查完成后地面站显示所发现动物的名称及每种动物数量 |

**飞控端需要解决的核心问题：**
1. 如何表示 9×7 网格地图与禁飞区
2. 如何规划一条**全覆盖且避开禁飞区**的巡查路径
3. 如何将路径转化为无人机可执行的导航指令
4. 如何在巡查过程中实时回传动物识别结果

---

## 二、系统架构

```
┌─────────────────────┐         蓝牙/串口         ┌─────────────────────┐
│   触摸屏地面站        │ ◄──────────────────────► │   飞控端 (TM4C123)   │
│                     │   禁飞区坐标 (3个点)       │                     │
│  • 9×7 网格显示      │ ──────────────────────► │  • BFS 路径规划      │
│  • 禁飞区设置        │                           │  • 蛇形遍历生成      │
│  • 动物识别结果显示   │ ◄────────────────────── │  • 导航指令执行      │
│  • 动物统计          │   方格代码+动物+数量       │  • 视觉识别回传      │
└─────────────────────┘                           └─────────┬───────────┘
                                                            │
                                                            │ UART
                                                            ▼
                                                  ┌──────────────────┐
                                                  │  K210 视觉模块    │
                                                  │  YOLO 动物检测    │
                                                  └──────────────────┘
```

---

## 三、地图与禁飞区表示

### 3.1 网格地图定义

地图为 7 行 × 9 列的网格（行号 0~6，列号 0~8），每个格子对应一个物理区域：

```c
#define MAP_WIDTH   9    // 列数
#define MAP_HEIGHT  7    // 行数
```

在飞控内部用二维数组表示地图，`0` 表示可通行，`1` 表示禁飞区（障碍物）：

```c
int map[MAP_HEIGHT][MAP_WIDTH];
memset(map, 0, sizeof(map));    // 初始化为全可通行

// 将 3 个禁飞区标记为障碍物
for (int i = 0; i < 3; ++i) {
    map[stop[i][0]][stop[i][1]] = 1;   // stop[i] = {行, 列}
}
```

### 3.2 禁飞区设置

赛题要求设置 **3 个禁飞区方格**，通过蓝牙/串口从触摸屏地面站接收坐标数据：

```c
// 禁飞区坐标（由地面站通过蓝牙下发）
int stop[3][2] = { {6,7}, {6,6}, {6,5} };
// 示例：第6行第7列、第6行第6列、第6行第5列
```

地面站端操作流程：
1. 触摸屏显示 9×7 网格
2. 用户点击 3 个格子设置为禁飞区
3. 通过蓝牙将坐标数据发送到飞控
4. 飞控接收后更新地图矩阵

---

## 四、路径规划算法

### 4.1 整体思路

路径规划分为三个阶段：

```
阶段1：生成蛇形遍历目标序列（覆盖所有可通行格子）
阶段2：用 BFS 计算相邻目标间的最短路径（绕开禁飞区）
阶段3：将所有路径段拼接，最后返回起点（闭环路径）
```

### 4.2 蛇形遍历目标生成

`build_snake_targets()` 生成一个蛇形遍历序列，从地图右下角开始，逐行蛇形扫描：

```c
static int build_snake_targets(const int m[MAP_HEIGHT][MAP_WIDTH],
                               int tr[], int tc[])
{
    int cnt = 0;
    for (int r = MAP_HEIGHT - 1, layer = 0; r >= 0; --r, ++layer) {
        if (layer % 2 == 0) {   // 偶数层：右→左
            for (int c = MAP_WIDTH - 1; c >= 0; --c)
                if (m[r][c] == 0) { tr[cnt] = r; tc[cnt] = c; cnt++; }
        } else {                // 奇数层：左→右
            for (int c = 0; c < MAP_WIDTH; ++c)
                if (m[r][c] == 0) { tr[cnt] = r; tc[cnt] = c; cnt++; }
        }
    }
    return cnt;
}
```

**蛇形遍历示意（无障碍物时）：**

```
行0:  ←───────────────────── (第0层，右→左)
行1:  ──────────────────────→ (第1层，左→右)
行2:  ←───────────────────── (第2层，右→左)
行3:  ──────────────────────→ (第3层，左→右)
行4:  ←───────────────────── (第4层，右→左)
行5:  ──────────────────────→ (第5层，左→右)
行6:  ←───────────────────── (第6层，右→左，起始行)
```

禁飞区格子会被跳过，保证路径不会经过禁飞区域。

### 4.3 BFS 最短路径算法

当蛇形遍历的相邻目标之间被禁飞区隔开时，需要绕行。使用 **BFS（广度优先搜索）** 计算两点间的最短路径：

```c
// 四方向：上、下、左、右
static int  DX[4]   = {-1, 1, 0, 0};
static int  DY[4]   = { 0, 0,-1, 1};
static char DIRC[4] = {'U','D','L','R'};

static int bfs_shortest(const int m[MAP_HEIGHT][MAP_WIDTH],
                        int sr, int sc, int tr, int tc, char *out_dir)
{
    if (sr == tr && sc == tc) return 0;

    // 队列与标记数组
    static int  qx[MAP_HEIGHT * MAP_WIDTH];
    static int  qy[MAP_HEIGHT * MAP_WIDTH];
    static bool vis[MAP_HEIGHT][MAP_WIDTH];
    static char pre_dir[MAP_HEIGHT][MAP_WIDTH];

    memset(vis, 0, sizeof(vis));
    memset(pre_dir, 0, sizeof(pre_dir));

    // BFS 入队起点
    int front = 0, rear = 0;
    qx[rear] = sr; qy[rear] = sc; rear++;
    vis[sr][sc] = true;

    bool found = false;
    while (front < rear) {
        int r = qx[front], c = qy[front]; front++;
        for (int k = 0; k < 4; ++k) {
            int nr = r + DX[k], nc = c + DY[k];
            if (!passable(nr, nc, m) || vis[nr][nc]) continue;
            vis[nr][nc] = true;
            pre_dir[nr][nc] = DIRC[k];   // 记录来向
            qx[rear] = nr; qy[rear] = nc; rear++;
            if (nr == tr && nc == tc) { found = true; goto DONE; }
        }
    }
DONE:
    if (!found) return -1;   // 不可达

    // 回溯生成路径方向序列
    char tmp[MAP_HEIGHT * MAP_WIDTH];
    int len = 0;
    int cr = tr, cc = tc;
    while (!(cr == sr && cc == sc)) {
        char d = pre_dir[cr][cc];
        tmp[len++] = d;
        if      (d == 'U') cr += 1;   // 反向：上→下
        else if (d == 'D') cr -= 1;   // 反向：下→上
        else if (d == 'L') cc += 1;   // 反向：左→右
        else if (d == 'R') cc -= 1;   // 反向：右→左
    }
    // 反转得到从起点到终点的方向序列
    for (int i = 0; i < len; ++i)
        out_dir[i] = tmp[len - 1 - i];

    return len;
}
```

**BFS 工作原理：**

```
1. 从起点入队，标记已访问
2. 取出队首节点，扩展四方向邻居
3. 跳过禁飞区和已访问节点，合法邻居入队并记录来向
4. 到达终点时停止
5. 从终点沿 pre_dir 回溯，得到最短路径
```

### 4.4 完整路径拼接

`build_cover_path_with_return()` 将所有路径段拼接成完整的巡查路径：

```c
int build_cover_path_with_return(const int stop[3][2], int *out_steps, int max_steps)
{
    // 1. 构建地图，标记禁飞区
    int map[MAP_HEIGHT][MAP_WIDTH];
    memset(map, 0, sizeof(map));
    for (int i = 0; i < 3; ++i)
        map[stop[i][0]][stop[i][1]] = 1;

    // 2. 起点：右下角
    const int SR = MAP_HEIGHT - 1;   // 行6
    const int SC = MAP_WIDTH  - 1;   // 列8

    // 3. 生成蛇形遍历目标序列
    int tr[MAP_HEIGHT * MAP_WIDTH], tc[MAP_HEIGHT * MAP_WIDTH];
    int tcnt = build_snake_targets(map, tr, tc);

    // 4. 找到起点在序列中的位置
    int sidx = -1;
    for (int i = 0; i < tcnt; ++i)
        if (tr[i] == SR && tc[i] == SC) { sidx = i; break; }

    int total_len = 0;
    int cr = SR, cc = SC;
    char seg[MAX_PATH];

    // 5. 从起点开始，依次 BFS 到后续目标
    for (int i = sidx + 1; i < tcnt; ++i) {
        int len = bfs_shortest(map, cr, cc, tr[i], tc[i], seg);
        // 将方向字符转为方向码写入 out_steps
        for (int k = 0; k < len; ++k)
            out_steps[total_len + k] = dirchar_to_code(seg[k]);
        total_len += len;
        cr = tr[i]; cc = tc[i];
    }

    // 6. 再 BFS 到序列前面的目标（完成闭环）
    for (int i = 0; i < sidx; ++i) {
        int len = bfs_shortest(map, cr, cc, tr[i], tc[i], seg);
        for (int k = 0; k < len; ++k)
            out_steps[total_len + k] = dirchar_to_code(seg[k]);
        total_len += len;
        cr = tr[i]; cc = tc[i];
    }

    // 7. 最后返回起点
    int len = bfs_shortest(map, cr, cc, SR, SC, seg);
    for (int k = 0; k < len; ++k)
        out_steps[total_len + k] = dirchar_to_code(seg[k]);
    total_len += len;

    return total_len;
}
```

**路径编码：**

| 方向码 | 含义 | 对应飞控动作 |
|--------|------|-------------|
| 1 | 上 (U) | `Horizontal_Navigation(0, +58cm)` |
| 2 | 下 (D) | `Horizontal_Navigation(0, -58cm)` |
| 3 | 左 (L) | `Horizontal_Navigation(-58cm, 0)` |
| 4 | 右 (R) | `Horizontal_Navigation(+58cm, 0)` |

---

## 五、路径执行与飞行控制

### 5.1 路径执行状态机

飞控按照 `steps[]` 数组中的方向码逐格飞行，每格约 58cm（对应赛题方格尺寸）：

```c
void Agriculture_UAV_Closeloop(void)
{
    static uint8_t n = 10;
    Vector3f target_position;

    // 首次调用：计算路径
    if (count == 0) {
        step1 = build_cover_path_with_return(stop, steps, MAX_PATH);
        count++;
    }

    // 状态机：按方向码执行飞行
    if (flight_subtask_cnt[n] == 5) {   // 判断下一步方向
        switch (steps[count - 1]) {
            case 1: flight_subtask_cnt[n] = 1; break;  // 上
            case 2: flight_subtask_cnt[n] = 2; break;  // 下
            case 3: flight_subtask_cnt[n] = 3; break;  // 左
            default: flight_subtask_cnt[n] = 4;         // 右
        }
        count++;
    }
    else if (flight_subtask_cnt[n] == 1) {   // 向上移动
        Horizontal_Navigation(0, 58, 0, RELATIVE_MODE, BODY_FRAME);
        count_y--;
        // 同时发送当前位置的动物识别结果到地面站
        NCLink_Send_To_Firetruck(count_x, count_y, ...);
        flight_subtask_cnt[n] = 5;   // 回到判断状态
    }
    // ... 类似处理 case 2/3/4
}
```

### 5.2 到点判断

每格飞行结束后，通过光流/SLAM 定位判断是否到达目标点：

```c
#define Waypoint_Fix_Cnt1  2    // 连续满足次数
#define Waypoint_Fix_CM1   10   // 到点判定半径 (cm)

if (flight_global_cnt[n] < Waypoint_Fix_Cnt1) {
    float dis_cm = pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,
                                 OpticalFlow_Pos_Ctrl_Err.y);
    if (dis_cm <= Waypoint_Fix_CM1)
        flight_global_cnt[n]++;     // 到点计数+1
    else
        flight_global_cnt[n] /= 2;  // 未到点则重置
}
```

### 5.3 动物识别数据回传

在每个格子巡查时，同时将视觉模块的识别结果通过 NCLink 协议发送到地面站：

```c
NCLink_Send_To_Firetruck(
    count_x, count_y,                        // 当前方格坐标 (列, 行)
    Opv_Top_View_Target.angle1,              // 动物类别1数量
    Opv_Top_View_Target.angle2,              // 动物类别2数量
    Opv_Top_View_Target.distance1,           // 动物类别3数量
    Opv_Top_View_Target.distance2,           // 动物类别4数量
    Opv_Top_View_Target.apriltag_id1,        // 动物类别5数量
    Opv_Top_View_Target.apriltag_id2,
    Opv_Top_View_Target.width1,
    Opv_Top_View_Target.width2,
    Opv_Top_View_Target.height1,
    Opv_Top_View_Target.height2
);
```

地面站接收到数据后，在对应方格位置显示动物种类和数量，并存储到文件中供赛后调出。

---

## 六、视觉识别（K210 YOLO）

巡查过程中的动物识别由 K210 视觉模块完成，运行 YOLOv2 模型检测 5 类动物：

```python
# K210 端：20 帧投票策略
N_FRAMES = 20

for _ in range(N_FRAMES):
    results.append(detect_one_frame())

# 统计频次，选择出现最多的结果
freq = {}
for idx, r in enumerate(results):
    key = tuple(r['packs'])
    freq[key] = freq.get(key, 0) + 1

best_key = max(freq.items(), key=lambda x: x[1])[0]
```

识别结果通过 UART 发送到飞控，飞控再通过 NCLink 转发到地面站。

---

## 七、完整工作流程

```
赛前准备阶段：
  ┌────────────┐    蓝牙     ┌────────────┐
  │ 触摸屏地面站 │ ────────► │   飞控端    │
  │ 设置3个禁飞区│   坐标数据  │ 更新map[][]│
  └────────────┘            └────────────┘

路径规划阶段：
  飞控内部执行 build_cover_path_with_return()
  ├── 构建地图，标记禁飞区
  ├── 生成蛇形遍历目标序列
  ├── BFS 计算相邻目标最短路径
  ├── 拼接所有路径段
  └── 生成 steps[] 方向码数组

巡查执行阶段：
  for each step in steps[]:
      1. 读取方向码（上/下/左/右）
      2. 调用 Horizontal_Navigation() 飞行 58cm
      3. 等待到点判定（光流/SLAM）
      4. K210 视识识别动物
      5. 将方格代码+动物数据发送到地面站
      6. 执行下一步

巡查结束：
  返回起点，地面站显示动物统计
```

---

## 八、关键设计总结

| 模块 | 算法/方法 | 作用 |
|------|----------|------|
| 地图表示 | 9×7 二维数组 | 0=可通行，1=禁飞区 |
| 遍历顺序 | 蛇形扫描 | 逐行覆盖所有可通行格子 |
| 路径规划 | BFS 广度优先搜索 | 绕开禁飞区的最短路径 |
| 路径执行 | 状态机 + 方向码 | 逐格飞行，每格 58cm |
| 到点判定 | 光流/SLAM 距离阈值 | 10cm 内连续 2 次判定到达 |
| 动物识别 | K210 YOLO + 20帧投票 | 5 类动物检测，准确率 95%+ |
| 数据回传 | NCLink 协议 | 方格代码+动物种类+数量 |
| 禁飞区设置 | 蓝牙/串口 | 触摸屏地面站下发坐标 |

---

**项目信息：**
- 竞赛：2025 年全国大学生电子设计竞赛 H 题（野生动物巡查系统）
- 飞控：CarryPilot V6.0.2（TI TM4C123GH6PM）
- 视觉：K210 + YOLOv2（MaixPy）
- 地面站：触摸屏 + 蓝牙通信
- 路径算法：BFS 最短路径 + 蛇形全覆盖遍历
