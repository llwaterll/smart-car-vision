# 香橙派与 F407 图像通信接口 v1

本页记录 vision1.2 与 F407 的串口协议。按键启停、曲率符号、子块采集和标定步骤见[比赛交接文档](Race_Handover.md)。

接收器将经过校验的观测交给比赛状态机，发车由车载菜单确认。蓝牙底盘调试可在手动模式独立运行。

## 接线和处理器分工

赛题第3页固定香橙派和核心板型号，底盘、轮胎半径及电机不能更改。因此香橙派承担后续图像识别/道路几何/数独识别，F407承担传感器闭环、运动执行和安全。

- 蓝牙为 USART1（库UART_0），PA9 TX / PA10 RX，9600。
- 图像单独使用 UART4（库UART_3），PC10 TX / PC11 RX，115200、8N1。
- 香橙派 TX 接 F407 的 PC11，香橙派 RX 接 F407 的 PC10，GND 共地，使用 3.3V TTL。
  v1 是双向链路：香橙派发 HELLO/TRACK/TILE/CAMERA_STATUS，F407 以 100 ms 周期回 STATUS；
  反向线是自动握手的前提。没有反向线时只能用 `--legacy-hello` 人工兜底。
- 香橙派具体串口设备名与引出脚需按实验室固定型号确认，不猜测 `/dev/ttyS*` 编号。不要把相机原始图像按此协议发送；发送的是识别结果和道路观测。

### LCD 连接诊断

车载主菜单在四个选项下方显示摄像头和香橙派。香橙派的 `vision` 以 `--camera ... --vision-uart …` 启动后，每约 250 ms 发送一次 `TYPE=3` 摄像头状态帧。该帧使用 v1 帧头和 CRC，`LEN=1`，`SEQ=0`，处理年龄为 0，会话字段非零。负载 `0` 表示近 1 秒没有读到画面，`1` 表示有画面。状态帧无需绑定比赛会话，且不改变道路观测、比赛会话或业务序号。

摄像头打开或读帧失败时，视觉程序保留串口状态回报并重试相机。STM32 超过 1 秒收不到状态帧时，仍可根据新鲜道路观测确认旧版视觉程序在线。两类数据都没有时，LCD 显示香橙派未连接、摄像头无法确认。要单独定位相机故障，请同时更新 STM32 固件和香橙派视觉程序。

### 自动握手与菜单状态

vision1.4 默认自动握手：上电后先等 F407 的 STATUS，只有看到
`state=STOP && ALLOW_HELLO && !FINISH_PENDING` 才生成新会话并发 HELLO，
未绑定前每 300 ms 重试；STATUS 报 `SESSION_VALID && bound_session == 本端会话`
后停止 HELLO 并开始发 TYPE=1。F407 结束一轮、复位、或拒绝原因为 3/5 时，
图像端会自动换新会话重绑，不需要人工 `--hello`。没有反向线时才使用
`--legacy-hello`：只发一次 HELLO，不假装 STATUS 在线。

菜单按握手阶段显示，避免“首页已连接、发车页视觉未开启”的歧义：

| 握手阶段 | 首页香橙派 | 发车页视觉 | 发车条件 |
|---|---|---|---|
| 视觉未编译 | 已关闭 | 配置已关闭 | 视觉功能已关闭 |
| 链路无合法帧 | 未连接 | 链路离线 | 等待视觉握手 / 视觉无新帧 |
| 链路在线、未绑定 | 未握手 | 等待握手 | 等待视觉握手 |
| 已绑定、无新鲜 TYPE=1 | 已连接 | 握手后无帧 | 视觉无新帧 |
| 已绑定、有新鲜 TYPE=1 | 已连接 | 已开启 | 等待有效赛道 / 可发车 |
| reject=6/7/8 | 协议错误 | 协议错误 | 视觉协议错误 |

只有 `SESSION_VALID` 且 200 ms 内有新鲜 TYPE=1 才显示“已连接/已开启”；
`TYPE=3` 和任何被业务拒绝的帧只证明链路在线，不计入 `accepted`。

## 固件API

```c
#include "Platform_Main.h"

CarVisionObservation observation;
if (Car_GetVisionObservation(&observation)) {
    /* 必须再检查对应valid_mask、confidence和任务所需的元素条件。
     * 本函数返回1仅代表存在新鲜、校验通过的数据，不代表识别到了道路。 */
}

CarVisionTile tile;
if (Car_GetVisionTile(0, &tile)) {
    /* 第0份数独的9个格；0为空，1..6为识别出的数字。 */
}
```

API拷贝一致快照，调用后不会暴露解析器可修改指针。UART ISR仅入队；控制周期在电机更新后有界处理最多64字节，不在ISR打印或发送串口整帧。

`CarVisionObservation` 含 session、sequence、本地解析时间、发送端报告的处理年龄、有效字段、横向误差m、航向误差rad、曲率1/m、元素距离m、置信度、元素类型和Tag编号。局部道路坐标约定：横向左为正、航向逆时针为正。图像端必须标定单位，不能把像素直接填入毫米字段。

## 线协议

所有多字节整数为**大端**。不直接传C结构体内存，也不传NaN/浮点原始表示。

```text
A5 5A | VERSION:u8 TYPE:u8 LEN:u16 SEQ:u16 SESSION:u32 AGE_MS:u16
      | PAYLOAD[LEN] | CRC16:u16
```

- VERSION=1；最大载荷32字节，最大帧48字节。
- CRC16/CCITT-FALSE，多项式0x1021、初始0xFFFF、不反射、xorout=0，覆盖VERSION到PAYLOAD末尾。标准测试串123456789结果0x29B1。
- SEQ为一条会话内所有观测/数独消息共享的16位递增序号；65535后回到0。拒绝重复/倒退，不能一次跳跃半个序号空间；连续丢失超过半空间须停车重新建立会话。
- SESSION为香橙派每次启动/每次新比赛生成的非零32位会话号。
- AGE_MS为从采集到发送前的处理年龄，范围0..200ms；需计入视觉处理队列等待，不能只填写单次算法耗时。
- 字节间隔超过50ms丢弃残帧。CRC错误、长度非法、版本错误或载荷不合法均不发布新观测；滑动查找帧头恢复同步。

### TYPE=0：HELLO

LEN=0，AGE_MS=0，SEQ约定0。F407 只在停车且没有收尾任务时绑定新会话，绑定时清除旧观测、业务序号和四份数独缓存。行驶中拒绝不同会话的 HELLO。相同会话的重复 HELLO 直接忽略，观测时效、序号、数独和收帧计数均不受影响。

vision1.4 默认自动握手：串口以 O_RDWR 打开，先等 F407 的 STATUS。只有看到
`STOP + ALLOW_HELLO + !FINISH_PENDING` 才生成新会话并发 HELLO，未绑定前每
300 ms 重试；收到 `SESSION_VALID && bound_session == 本端会话` 后停止 HELLO
并开始发 TYPE=1。只按 F407 的 RESET 时，香橙派继续运行，STATUS 的
`UPTIME_MS` 回退会让图像端换新会话重绑。停车且上一轮收尾完成后，F407 自动
关闭旧轮次（`CarVision_CloseRaceSession`），STATUS 变为未绑定；图像端自动
重绑，不需要人工 `--hello`。没有反向线时才使用 `--legacy-hello`：只发一次
HELLO，不假装 STATUS 在线。v1 的 STATUS 是握手/诊断通道，不是路障动作 ACK；
联调时以 LCD 上的会话号、持续变化的帧号和「正常/拒绝」计数为准。

### TYPE=1：道路与元素观测，LEN=16

| 偏移 | 字段 | 范围/单位 |
|---|---|---|
| 0 | valid_mask u16 | bit0横向、bit1航向、bit2曲率、bit3距离；bit4 表示单边估计曲率，须与 bit2 同时置位；其余位必须为 0 |
| 2 | confidence u16 | 0..1000，对外转换为0..1 |
| 4 | lateral i16 | -1000..1000 mm，左正 |
| 6 | heading i16 | -3142..3142 mrad，逆时针正 |
| 8 | curvature i16 | -20000..20000，单位0.001/m |
| 10 | element u8 | 0无、1环岛、2十字、3断路、4绿、5黄、6红、7终点候选、8Tag |
| 11 | tag_id u8 | Tag36h11编号0..7，255未识别 |
| 12 | distance u16 | 0..65535 mm，有效性由bit3指定 |
| 14 | reserved u16 | 必须0 |

无道路时仍发送valid_mask=0的新帧，明确区分“视觉在线但未检出”和“通信断流”。元素是候选观测，不是命令；F407任务层需处理稳定判定、圈次、红障停车5秒、重复触发、环岛进出与真正停车点。收到红色或终点标志不会在此接口里直接停车。

HSV 普通巡线可短暂使用单侧边线，沿用最近一次双边测得的路宽，并检查估计中点与上一帧及最近实测中点的偏差。最多允许连续 4 帧且距离最近双边实测不超过 250 ms。估计帧置 bit4，置信度为 450；STM32 只在普通巡线中使用该曲率，限速 0.18 m/s。估计帧不能用于发车、确认断路接回或刷新普通巡线的丢线停车计时。识别到蓝底断路入口时仍上报原有断路元素。使用此位需要同时更新发送和接收端。

### TYPE=3：摄像头状态，LEN=1

仅用于 LCD 诊断，不要求绑定比赛会话。`LEN=1`、`SEQ=0`、`AGE_MS=0`；负载 `0` 表示相机近 1 秒没有画面，`1` 表示有画面。F407 收到后刷新摄像头状态，超过 1 秒没收到则显示“无法确认”；该帧不改变道路观测、比赛会话或业务序号。

### F407 → 香橙派：STATUS，总长 36 字节

方向帧头 `5A A5`（与香橙派上行 `A5 5A` 相反），多字节字段为大端：

| 偏移 | 字段 | 说明 |
|---:|---|---|
| 0 | SYNC0=0x5A | 方向帧头 |
| 1 | SYNC1=0xA5 | |
| 2 | VERSION=1 | |
| 3 | TYPE=0x01 | STATUS |
| 4 | LEN=26 | |
| 6 | SEQ u16 | 仅真正启动发送时递增，允许回绕 |
| 8 | STATE u8 | 0 STOP、1 RUN、2 FAULT |
| 9 | FAULT u8 | `CAR_*_FAULT` |
| 10 | FLAGS u8 | 见下 |
| 11 | REJECT u8 | 最近业务拒绝原因 0..8 |
| 12 | BOUND_SESSION u32 | 0 表示未绑定 |
| 16 | ACCEPTED u32 | 含 HELLO 的接受帧数；TYPE=3 不计入 |
| 20 | REJECTED u32 | CRC 正确但业务拒绝的帧数 |
| 24 | BAD u32 | 长度/CRC 错误帧数 |
| 28 | OBS_AGE_MS u16 | 0xFFFF 表示无观测 |
| 30 | UPTIME_MS u32 | F407 上电毫秒，回退视为复位 |
| 34 | CRC16 u16 | CRC-16/CCITT-FALSE，覆盖偏移 2..33 |

FLAGS：bit0 `ALLOW_HELLO`、bit1 `SESSION_VALID`、bit2 `FINISH_PENDING`、
bit3 `UART_FAULT`、bit4 `OBS_FRESH`、bit5 `AUTONOMOUS`、bit6 `LINK_ALIVE`。
周期 100 ms；state/flags/bound_session 变化立即发送，但两次发送最小间隔 20 ms。

图像端自动握手完全由这些字段驱动：没有 STATUS 不自动 HELLO；只在
`STOP + ALLOW_HELLO + !FINISH_PENDING` 重绑；`UPTIME_MS` 回退视为 F407 复位；
`session_valid=0` 或 `reject=3/5` 触发换会话；`reject=4` 或 `obs_age>160`
进入低延迟；`reject=6/7/8` 停止业务帧。控制端在 HELLO 绑定成功和有效
TYPE=1 时都会把 `REJECT` 清为 0，避免旧拒绝误导图像端。

### TYPE=2：3×3数独子块，LEN=10

第0字节是子块索引0左上/1右上/2左下/3右下；后9字节按行排列，每格0空白或1..6。四份缓存仅在当前会话内保留；上一轮停车并完成收尾后清空缓存，再次 HELLO 可沿用同一发送端的 SESSION。Tag0/1、2/3、4/5、6/7分别关联四份子块，由香橙派识别与映射。

香橙派运行入口使用 `sudoku.hpp` 与 `sudoku_reader.hpp` 完成 Tag 定位、网格矫正、数字识别及逐块两次一致确认。未知格使整块保持未确认，只有已识别的空格写 0。每个已确认子块由 `sendTile()` 使用全会话递增序号发送，同帧多块分别发送，单次处理年龄超过 200 ms 时跳过本次发送。重复观察时重新发帧，便于恢复丢包。

F407 允许四块乱序到达，按块号缓存，同一块的新有效帧可以更新旧值。CRC 错误、非法值、旧帧均保持已收结果。UART 接收异常清除道路瞬时观测与半帧，已校验数独块保留到新 HELLO。比赛结束且两轮静止后，控制端给末帧最多 1 秒收尾窗口，冻结四块并按 2×3 宫规则求唯一解。搜索最多访问 20000 个节点，超限时报告状态，失败时保持答案缓冲。此时新会话被拒绝，避免混入下一轮数据。

## 香橙派侧参考发送器

`Tools/vision_link.py` 是可复用的协议编码器，使用标准库打包和CRC；传输使用与蓝牙工具相同的pyserial依赖。

```python
import serial
from vision_link import VisionSender

# 按实际香橙派型号选择串口设备。先让F407保持STOP。
port = serial.Serial('/dev/ttySx', 115200, write_timeout=0.1)
sender = VisionSender(port)
sender.hello()

# 放入真实视觉处理循环，数值必须来自识别与标定。
sender.send_track(valid_mask=3, confidence=900,
                  lateral_mm=12, heading_mrad=-20,
                  processing_age_ms=30)
sender.send_tile(0, [1, 0, 3, 0, 5, 0, 2, 0, 0], processing_age_ms=30)
```

上例数字只是协议用法，不是图像算法。离线生成测试帧无需串口或摄像头：

```powershell
python Tools/vision_link.py --hex hello
python Tools/vision_link.py --hex track
python Tools/vision_link.py --hex tile
```

## 板端诊断（联调优先看这里）

LCD 菜单 `Sensors -> Vision` 显示最近一帧的会话号、序号、置信度、曲率和元素，并在最后两行显示累计接收统计：

- `OK`：完整通过 CRC 和业务校验的帧数，含 HELLO；**TYPE=3 不计入**；
- `bad`：长度越界或 CRC 错误的帧数，持续增长通常是波特率/接线/字节丢失；
- `Rejected`：CRC 正确但会话、序号、年龄或字段越界被拒的帧数，持续增长通常是
  HELLO 未建立、会话不匹配、序号不递增或字段超出协议范围。

`OK` 不增长且 `bad`/`Rejected` 也不增长，说明 UART4 完全没有收到字节（接线、共地、
香橙派串口设备或发送端未启动）。`OK` 增长但页面显示 `No fresh observation`，说明链路正常、
只是最近帧超过 200 ms 时效。`OK=0` 而 `Rejected` 持续增长，说明 TYPE=1 都在被拒，
最常见原因是图像端还没绑定会话；此时首页应显示“香橙派: 未握手”，发车页显示
“视觉: 等待握手”。三个计数自复位起累计，重新上电清零。

发车页第 3 行按握手阶段显示“链路离线 / 等待握手 / 握手后无帧 / 协议错误 / 已开启”；
首页第 6 行按“未连接 / 未握手 / 协议错误 / 已连接”显示。只有 `SESSION_VALID`
且 200 ms 内有新鲜 TYPE=1 才是“已连接/已开启”。

## 新鲜度和当前边界

重复/乱序/错误帧不刷新时间。道路数据按“发送端报告处理年龄 + 本地解析后的年龄”检查200ms上限，处理本地32位毫秒时钟回绕。

这个年龄**尚不包含无法测量的发送缓冲、线路和接收队列延迟**，不是严格曝光到控制的端到端期限。后续闭环启用前，需要限制队列、确认视觉帧率和最坏通信延迟，必要时补时钟同步/采集时间戳及更保守的延迟预算。200ms只是接收接口初始上限，不代表可接受的高速驾驶延迟。

UART 溢出或错误使道路观测失效，比赛状态机不再直接停车：普通巡线沿用最近一次有效曲率，把目标速度限制到 0.18 m/s，并在 LCD 状态栏显示 `VISION LOST`。`CODE/Track_Fsm.h` 的 `TRACK_VISION_LOST_STOP_MS` 默认 0 表示无限期保持；设为非零毫秒数可恢复“持续失效后故障停车”的保护。单边估计曲率也按丢线处理：可用于普通巡线转向，但不刷新实测道路时间、不累计直线里程，也不能用于发车或断路接回。断路阶段仍按里程和航向盲走，接回白路须连续三帧实测确认。当前会话此前接收的数独块继续保留。手动模式继续使用蓝牙断链停车逻辑。

已验证主机C协议测试、Python编码器测试及Keil构建；香橙派侧 `vision_link.hpp` 的帧格式、
大端字段、CRC 覆盖范围与 F407 解析器逐项对照一致。尚未连接香橙派或摄像头，没有实车联调结果。

联调顺序建议：F407 保持 STOP 上电 -> LCD `Sensors -> Vision` 确认计数 -> 香橙派执行
`--vision-uart <设备> --publish-curvature`（自动握手，不需要 `--hello`）->
观察 `[link] phase=BOUND`、`OK` 递增且显示会话号 -> 发车页变为“视觉: 已开启”。
没有反向线（PC10 -> 香橙派 RX）时才临时使用 `--legacy-hello`，且必须人工确认 F407 在 STOP。
`--publish-curvature` 是发车前提：F407 要求实测曲率有效；`--single-edge-send` 的 bit4
单边估计永远不会被允许发车。
