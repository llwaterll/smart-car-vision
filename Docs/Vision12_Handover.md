# 给视觉组的 vision1.2 联调交接

更新于 2026-11-14。接收方为维护香橙派图像程序的同学，配套控制器为 STM32F407VE。本文对应本工程已合入的 vision1.2 数独代码、当前串口协议及三级菜单。

## 当前对接方式

视觉发送道路观测和沿途识别的四个原始 3×3 子块。控制端按块号收集，在完赛停车后拼为 6×6，按每行、每列和每个 2×3 宫数字 1–6 不重复求唯一解，并在车载 LCD 显示。空格用 0，低置信度格应保留为未确认，整块暂缓发送。图像侧自带的求解器用于离线校验，车载答案来自本次会话收到的四块题面。

规则依据项目内赛题说明第 3 页。真正停车点与软件盲盒由现场公布，当前练习程序使用两圈累计里程 110.122 m 触发终点。正式停车条件公布后需共同更新任务层。[赛题说明](<NEUQ智能车实验室_校赛赛题说明 (初版) (1).pdf>)

## 视觉组接续开发的文件

| 文件 | 当前职责及本次接入内容 |
|---|---|
| `vision_inspection/vision/sudoku.hpp` | vision1.2 数独模块。Tag 配对、网格边框透视矫正、模板识别、四块拼接及有界唯一解检查 |
| `vision_inspection/vision/sudoku_reader.hpp` | 每块独立做连续两次一致确认，缓存字形模板。`readAll()` 返回同帧全部确认块，`read()` 保留单块调用兼容 |
| `vision_inspection/vision/main.cpp` | 运行入口。先处理并发送道路，每隔一帧识别数独，逐块发送 TYPE=2，同时写 `sudoku.csv` |
| `vision_inspection/vision/vision_link.hpp` | HELLO、道路和子块编码，共用 CRC16、大端字段 |
| `vision_inspection/vision/vision_serial.hpp` | Linux 串口发送，共用一个 session 和业务 sequence |
| `CODE/Proto_Vision.c` | F407 校验并缓存观测、子块。串口异常保留当前会话已验证的子块 |
| `CODE/Platform_Main.c` | 终点停车、末帧收尾、冻结题面和前台求解 |
| `CODE/Sudoku.c`、`CODE/Menu_Task.c` | 2×3 宫求解、采集进度和结果显示 |

新版源包的数独函数已接到主循环。定位使用 Tag 对角顺序确定方向，并寻找网格外框，使九格切分对应实际网格。新增检查覆盖识别回调越界、完整题面矛盾和多解，求解失败保持原输入或答案缓冲。

## 接线与启动

香橙派 3.3 V TTL TX 接 F407 的 PC11，香橙派 RX 接 F407 的 PC10，双方 GND 共地。
使用 UART4（NEUQ 库 `UART_3`）、115200、8N1。链路是双向的：香橙派发
HELLO/TRACK/TILE/CAMERA_STATUS，F407 每 100 ms 回一帧 36 字节 STATUS；
反向线（PC10 -> 香橙派 RX）是自动握手的前提。板上蓝牙使用独立 USART1（PA9/PA10）。

在香橙派上安装含 aruco 的 OpenCV 4 开发库、C++17 编译器和 pkg-config，进入本工程的视觉目录。将命令中的 `/dev/ttyS1` 替换为实际串口设备。

```bash
cd vision_inspection/vision
bash build.sh vision
bash build.sh test_sudoku
./test_sudoku
./vision --camera 0 --config track_frontend.yml --vision-uart /dev/ttyS1 --publish-curvature
```

先给 F407 上电并保持 STOP，再启动上述摄像头命令。摄像头输入为 640×480。
vision1.4 默认自动握手：先收 F407 的 STATUS，看到
`STOP + ALLOW_HELLO + !FINISH_PENDING` 才生成新 session 并发 HELLO，
未绑定前每 300 ms 重试；绑定后停止 HELLO 并开始发 TYPE=1。
新一轮比赛不需要人工重启或 `--hello`：F407 结束一轮后 `session_valid=0`，
图像端会自动换新 session 重绑；F407 复位时 `uptime_ms` 回退也会触发重绑。
本仓库 `vision_inspection/` 若仍是旧版，必须换成 vision1.4，或临时用
`--legacy-hello` 人工确认 STOP 后建立一次会话；后者不具备自动重绑。
`--publish-curvature` 是发车前提：F407 要求实测曲率有效；`--single-edge-send`
的 bit4 单边估计永远不会被允许发车。

车上选择 `Race → Start race`，等发车页第 3 行显示“视觉: 已开启”、
第 4 行不再等待握手后，再按 PE7 确认发车。首页第 6 行“香橙派: 未握手”
表示链路在线但没有绑定会话；此时按 OK 只能进入“预备”，等自动握手完成才会开跑。
未发车时可在 `Sensors → Vision` 核对 session、OK/拒绝计数和道路数据，
在 `Race → Sudoku result` 检查子块。PE10 停车时向上、运行时停车，PE8 向下，
PE7 确认，PE9 返回。

本机 WSL 编译产物供回归使用，香橙派部署时运行 `bash build.sh vision` 为板上 CPU 重新编译。

## 四块数据约定

| 子块索引 | 位置 | Tag 左上 / 右下 | 映射到完整棋盘 |
|---|---|---|---|
| 0 | 左上 TL | 0 / 1 | 第 1–3 行，第 1–3 列 |
| 1 | 右上 TR | 2 / 3 | 第 1–3 行，第 4–6 列 |
| 2 | 左下 BL | 4 / 5 | 第 4–6 行，第 1–3 列 |
| 3 | 右下 BR | 6 / 7 | 第 4–6 行，第 4–6 列 |

九格按矫正后子块的行优先顺序排列，先第一行左到右，再第二行和第三行。标签用于方向定位，旋转图像应仍得到相同九格顺序。允许四块乱序到达，同一块新的有效帧覆盖先前值。

## 串口协议

完整帧的多字节字段均为大端。

```text
A5 5A | VERSION u8 | TYPE u8 | LEN u16 | SEQ u16
      | SESSION u32 | AGE_MS u16 | PAYLOAD | CRC16 u16
```

VERSION 为 1。CRC 使用 CRC-16/CCITT-FALSE，多项式 0x1021、初值 0xFFFF，覆盖 VERSION 到 PAYLOAD 末尾，CRC 自身按大端发送。

| TYPE | LEN | 负载 |
|---|---|---|
| 0 HELLO | 0 | 无负载，SEQ=0、AGE_MS=0，SESSION 非零 |
| 1 TRACK | 16 | 道路与元素观测，字段定义见通信接口文档 |
| 2 SUDOKU_TILE | 10 | 第 0 字节为块号 0–3，后 9 字节为格值 0–6 |
| 3 CAMERA_STATUS | 1 | 1=有画面，0=近 1 s 无画面；不占用业务序号 |

TYPE=2 整帧为 26 字节。业务 SEQ 在道路帧与子块帧之间共用，连续递增并允许 65535 回绕到 0。四个子块各发送一次需要四个不同业务序号。重新观察同一块时，发送新序号的新帧。

下面的已核对测试帧使用 session=123、seq=1、age=15 ms、tile=0，九格为 `1,0,3,0,5,0,2,0,1`。

```text
A5 5A 01 02 00 0A 00 01 00 00 00 7B 00 0F
00 01 00 03 00 05 00 02 00 01 25 15
```

AGE_MS 为该帧采集后到发送前的处理年龄，允许 0–200 ms。主循环按实际经过时间向上取整，超过 200 ms 的数独帧跳过本次发送。道路观测按自身年龄检查，数独帧不能刷新道路观测的有效期。该年龄未覆盖相机内部、发送队列和接收队列的全部延迟，现场联调需记录真实帧率和端到端时序。

控制端丢弃 CRC 错误、非法块号、非法格值、旧序号及不匹配会话。字节间隔超过 50 ms 时重新找帧头。已通过校验的数独块跨帧保留，新会话清空旧块。串口临时错误清掉道路观测及半帧，保留本会话已采集数独块。

道路帧与数独帧使用同一个 `SerialSender`。当前道路几何采用向左为正、逆时针为正，保留现有鸟瞰尺度和串口转换。协议完整定义及 Python 参考编码器见项目接口说明。[通信接口](Vision_Interface.md)

反向 STATUS 帧（F407 -> 香橙派）使用相反帧头 `5A A5`、TYPE=0x01、LEN=26，
总长 36 字节；字段、flags 和 reject 取值见通信接口文档的 STATUS 表。
它承载 `ALLOW_HELLO / SESSION_VALID / FINISH_PENDING`、`bound_session`、
`accepted/rejected/bad`、`obs_age_ms` 和 `uptime_ms`。图像端据此自动 HELLO、
换会话和进入低延迟；它不是路障动作的 `eventId/ACCEPTED/COMPLETED` ACK。

## 运行入口与控制端反馈

当前 `main.cpp` 已执行以下顺序。

1. 读取摄像头帧，计算道路、路障观测，发送 TYPE=1。
2. 每隔一帧调用 `tileReader.readAll(current)`，按块号分别累计连续一致次数。目标丢失或识别失败会重置该块的确认次数。
3. 对返回的每个确认块调用 `sendTile(age, index, cells)`。同一画面有多块时逐块发送，共用全局业务序号。
4. 指定 `--output` 时记录确认结果及发送状态。

绿障直接通过、黄障绕行、红障第二圈停车 5 秒的执行与圈次判断在 F407。包内 `obstacle_task` 的 Event/ACK 仍是进程内实验接口，当前串口使用 TYPE=1 颜色和几何观测驱动现有控制状态机。STATUS 反向帧只用于视觉握手和诊断，不是动作 ACK；后续若双方改用事件握手，需要另行定义 `eventId/ACCEPTED/COMPLETED` 线协议。

车载 `Race → Sudoku result` 显示 TL、TR、BL、BR 是否收到，比赛状态页显示已收数量。达到练习终点后保持电机停止，连续 100 ms 确认两轮低于 2 rpm；缺块时继续等到终点触发后 1 秒，再冻结本会话题面。求解放在前台，控制中断继续运行。

| 屏幕状态 | 对接时如何判断 |
|---|---|
| Collecting | 还在收集，核对四个位置是否全部 received |
| Stopped: receiving last tiles | 车辆已进入终点收尾，仍处理末帧 |
| Missing tiles | 冻结题面时缺块，查看 missing 的块号及视觉日志 |
| Conflicting / unsolvable cells | 收到的格值有冲突或无解，复核九格顺序、旋转方向、识别数字 |
| Multiple solutions | 给定数据未确定唯一答案，检查漏识别为空格的情况 |
| Solver search limit | 搜索超过 20000 节点，保留日志及题面用于分析 |
| 六行六列数字 | 唯一解已生成并显示 |

只有本地串口写完时 `sudoku.csv` 的 sent 才记录 1；是否已被 F407 接收，以车载子块进度为准。结果页展示冻结时的块状态，迟到帧不会改写本次已经输出的结果。

## 日志与复现

离线回放可直接核对图像侧识别结果。

```bash
./vision --video recording.avi --output replay-001 --save-every 10
```

`replay-001` 需要为新的输出目录。程序写出 `frames.csv`、`sudoku.csv` 和抽样调试图。数独 CSV 每行包含 frame、tile、c0–c8、age_ms、sent。离线回放时 sent 为 0。frames.csv 的 processing_ms 及控制台 mean_processing_ms 统计道路前端处理时间，数独记录的 age_ms 包含该帧道路和数独处理经过时间。

从工程根目录可运行本次完整软件回归。

```bash
bash Tools/check_race.sh
```

脚本生成四块数独合成视频，用实际视觉主程序回放并核对日志中的每一格；同时通过 Linux 伪串口把 C++ 编码器输出送入 F407 的 C 接收器，再拼接求解。结果保存在 `build/race-check/sudoku-replay.*/output`。

本次已通过模板识别、旋转与同帧多块、四块映射、协议拒绝及会话隔离、唯一解与失败保护、菜单显示、双圈比赛仿真和上述主程序回放。Keil 构建为 0 错误、0 警告。

## 下一次双方联调需要记录

视觉侧提供每块原始画面、Tag 尺寸与位置、预期九格值、识别日志、相机帧率和处理延迟。重点核对实际字体、光照、运动模糊、空格与未知格的区别，以及网格外框是否稳定可见。当前提供的证据覆盖合成样片和本机软件，尚未完成真实相机与 F407 UART 联调。

控制侧核对同一会话四块接收进度、末块到达时机、停车后冻结的题面和 LCD 答案。双方共同确认正式终点与盲盒规则、实际串口设备、相机几何标定和路障阈值。新一轮联调统一启动新 session，并随日志记录源码版本。
