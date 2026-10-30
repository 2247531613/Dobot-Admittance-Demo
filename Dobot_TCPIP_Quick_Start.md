# Dobot Python 二次开发快速指南

本文面向第一次接触 Dobot TCP/IP 接口的开发者或 Agent，目标是在短时间内建立对本项目、通信方式、常用 API、实时反馈和运动控制流程的整体认识。

> 本文以仓库当前的 `dobot_api.py` 为准。不同机器人型号、控制器固件和 TCP/IP 协议版本可能存在字段长度、命令参数或行为差异，实机使用前应同时核对项目根目录中的官方接口 PDF。

## 1. 先建立整体认知

Dobot Python V4 SDK 本质上是对控制器 TCP/IP 文本命令和二进制反馈流的封装。

程序通常同时维护两条连接：

| 端口 | 项目类 | 数据形式 | 主要用途 |
| --- | --- | --- | --- |
| `29999` | `DobotApiDashboard` | 文本命令，一发一回 | 使能、查询、运动、IO、停止和报警处理 |
| `30004` | `DobotApiFeedBack` | 连续二进制数据流 | 实时读取关节角、位姿、状态、IO、力和指令 ID |

典型程序的数据流如下：

```text
主线程/控制线程
    ├─ 通过29999发送EnableRobot、MovJ、ServoP等命令
    └─ 从共享快照读取最新机器人状态

反馈线程
    └─ 通过30004持续调用feedBackData()
        └─ 解析二进制反馈并更新共享快照
```

关键原则：

1. `29999` 用于主动控制和查询。
2. `30004` 用于持续接收实时状态。
3. `feedBackData()` 是阻塞调用，通常必须放到后台线程。
4. 同一个 Dashboard 连接不要由多个线程并发发送命令。

## 2. 重要文件

| 文件 | 作用 |
| --- | --- |
| `dobot_api.py` | TCP 连接、Dashboard 命令、实时反馈结构和解析逻辑 |
| `feedback_monitor.py` | 当前项目封装的后台反馈线程和状态快照 |
| `DobotDemo.py` | 官方风格的基础连接、反馈和 `MovJ` 示例 |
| `Feedback调试问题总结.md` | 反馈长度、粘包、线程和 Socket 关闭问题总结 |
| `get_error_example.py` | 报警读取示例 |
| `02 Dobot TCP_IP二次开发接口文档_*.pdf` | 官方命令、参数和返回值依据 |

仓库中还有导纳控制、力跟踪和轨迹实验代码。首次接手时应先理解基础 SDK 和 `feedback_monitor.py`，不要直接从复杂控制器反推底层接口。

## 3. 环境和连接条件

基础依赖：

```powershell
python -m pip install numpy requests
```

连接前确认：

- 电脑与机器人位于同一网段；
- 使用实际机器人 IP，示例常见为 `192.168.5.1`；
- 机器人已切换到 TCP/IP 控制模式；
- 急停已释放；
- 工具、负载和现场安全条件正确；
- 端口 `29999` 和 `30004` 未被其他程序独占。

## 4. 三个核心类

### 4.1 `DobotApi`

底层 TCP 基类，负责：

- 建立 Socket；
- 发送字符串；
- 等待文本回复；
- 串行化一发一回操作；
- 关闭和重连。

普通业务代码一般不直接实例化它。

### 4.2 `DobotApiDashboard`

连接 `29999`，提供控制和查询接口：

```python
from dobot_api import DobotApiDashboard

dashboard = DobotApiDashboard("192.168.5.1", 29999)
```

### 4.3 `DobotApiFeedBack`

连接 `30004`，持续读取二进制反馈：

```python
from dobot_api import DobotApiFeedBack

feed = DobotApiFeedBack("192.168.5.1", 30004)
data = feed.feedBackData()
```

项目已经提供 `FeedbackMonitor`，实际业务优先使用它，而不是在主线程直接处理 Socket 和 NumPy 结构。

## 5. 最小只读查询

第一次连接机器人时，先只读取数据，不使能、不运动：

```python
from dobot_api import DobotApiDashboard

robot_ip = "192.168.5.1"
dashboard = DobotApiDashboard(robot_ip, 29999)

try:
    print("模式:", dashboard.RobotMode())
    print("关节角:", dashboard.GetAngle())
    print("TCP位姿:", dashboard.GetPose())
    print("报警ID:", dashboard.GetErrorID())
    print("DI1:", dashboard.DI(1))
    print("DO1:", dashboard.GetDO(1))
finally:
    dashboard.close()
```

这类方法返回协议字符串，而不是直接返回 Python 数值。例如：

```text
0,{300.0,0.0,400.0,180.0,0.0,180.0},GetPose();
```

通常可理解为：

```text
ErrorID,{返回数据},Command(...);
```

- 第一个整数为错误码；
- `0` 通常表示命令被成功接收；
- 花括号中是查询结果或运动指令 ID；
- 非零错误码应结合官方文档和报警信息判断。

不要用简单的“提取全部整数”方法解析浮点位姿，因为小数会被错误拆分。运动回复只需要提取错误码和 ResultID 时，可以针对其固定格式单独解析。

## 6. 基础控制命令

常用生命周期命令：

```python
dashboard.ClearError()       # 原因已排除后，清除可清除报警
dashboard.EnableRobot()      # 使能
dashboard.DisableRobot()     # 下使能
dashboard.Stop()             # 停止运动队列或脚本
dashboard.Pause()            # 暂停
dashboard.Continue()         # 继续
dashboard.SpeedFactor(10)    # 全局速度比例1~100
```

推荐控制流程：

```text
连接
→ 查询RobotMode和报警
→ 排除报警原因
→ ClearError
→ EnableRobot
→ 确认实时反馈中的使能状态
→ 设置运动参数并运动
→ Stop（需要时）
→ DisableRobot
→ 关闭连接
```

`ClearError()` 不能代替排除故障原因。`EmergencyStop()` 也不应当作普通程序停止使用。

## 7. 基础运动接口

### 7.1 `MovJ`

函数签名：

```python
MovJ(a1, b1, c1, d1, e1, f1, coordinateMode,
     user=-1, tool=-1, a=-1, v=-1, cp=-1)
```

`coordinateMode`：

- `0`：前六个值为笛卡尔位姿 `[X,Y,Z,Rx,Ry,Rz]`；
- `1`：前六个值为关节角 `[J1,J2,J3,J4,J5,J6]`。

关节目标示例：

```python
target_joint_deg = [0.0, -20.0, -80.0, 10.0, 90.0, 0.0]
reply = dashboard.MovJ(*target_joint_deg, 1, a=5, v=5)
```

`MovJ` 采用关节运动，TCP 路径不保证是直线。

### 7.2 `MovL`

函数签名：

```python
MovL(a1, b1, c1, d1, e1, f1, coordinateMode,
     user=-1, tool=-1, a=-1, v=-1,
     speed=-1, cp=-1, r=-1)
```

笛卡尔直线运动示例：

```python
target_pose = [300.0, 0.0, 350.0, 180.0, 0.0, 180.0]
reply = dashboard.MovL(
    *target_pose,
    0,
    user=0,
    tool=0,
    a=5,
    speed=20,
)
```

注意：

- `v` 是速度比例，`speed` 是笛卡尔速度参数，二者不要同时使用；
- `cp` 和 `r` 是两种过渡参数，不要同时使用；
- `MovL` 的整条直线路径都必须安全；
- 绝不能直接使用未经现场验证的示例坐标。

### 7.3 `ServoJ` 和 `ServoP`

```python
ServoJ(J1, J2, J3, J4, J5, J6,
       t=-1.0, aheadtime=-1.0, gain=-1.0)

ServoP(X, Y, Z, RX, RY, RZ,
       t=-1.0, aheadtime=-1.0, gain=-1.0)
```

- `ServoJ`：关节空间动态跟随；
- `ServoP`：笛卡尔空间动态跟随；
- 适合外部轨迹、视觉伺服或导纳控制等连续更新目标的场景；
- 官方文档建议循环间隔约 `30 ms`，最高调用频率约 `33 Hz`；
- `ServoP` 不受全局 `SpeedFactor()` 比例影响，但受机器人速度限制；
- `aheadtime` 越小、`gain` 越大，响应通常越快，同时更容易抖动或不稳定。

连续伺服期间只由一个控制线程发送 Servo 命令，不要与 `MovJ/MovL` 混发。程序必须自行限制单周期增量、速度、工作空间、力和反馈超时。

## 8. 数字 IO

主动查询或设置：

```python
dashboard.DI(1)             # 读取DI1
dashboard.GetDO(1)          # 读取DO1
dashboard.DO(1, 1)          # 队列方式打开DO1
dashboard.DOInstant(1, 0)   # 立即关闭DO1
```

实时反馈中的 `DigitalInputs` 和 `DigitalOutputs` 是位掩码。通常可按下面方式读取从 1 开始编号的一路信号：

```python
def read_io_bit(mask, index):
    return (int(mask) >> (index - 1)) & 1
```

控制 DO 前必须确认其没有连接会产生危险动作的夹具、电磁阀或其他执行机构。

## 9. 实时反馈结构

`dobot_api.py` 中的 `MyType` 描述反馈帧字段布局。当前仓库的结构大小应通过以下方式获得：

```python
packet_size = MyType.itemsize
```

当前版本将 `reserve9` 定义为 3 字节，因此 `MyType.itemsize` 为 `1424`。历史版本或其他协议定义可能是 `1440`，不要在业务代码里盲目硬编码帧长。

常用字段：

| 字段 | 含义 |
| --- | --- |
| `RobotMode` | 机器人运行模式 |
| `EnableStatus` | 是否使能 |
| `RunningStatus` | 是否运行 |
| `ErrorStatus` | 是否报警 |
| `CurrentCommandId` | 当前算法队列指令 ID |
| `QActual` | 六关节实际角度 |
| `ToolVectorActual` | TCP 实际位姿 |
| `TCPSpeedActual` | TCP 实际速度 |
| `DigitalInputs` | DI 位掩码 |
| `DigitalOutputs` | DO 位掩码 |
| `ActualTCPForce` | TCP 力/力矩相关反馈 |
| `SixForceValue` | 六维力传感器相关反馈 |
| `ActualQuaternion` | 当前姿态四元数 |
| `TestValue` | 固定校验值，用于检查解析位置 |

`np.frombuffer()` 返回长度为 1 的结构化数组，因此读取字段时通常有 `[0]`：

```python
mode = int(data["RobotMode"][0])
joints = data["QActual"][0].astype(float).tolist()
pose = data["ToolVectorActual"][0].astype(float).tolist()
```

## 10. 反馈有效性检查

不要用 `data["len"][0] == MyType.itemsize` 判断一帧是否有效。调试中该协议字段曾因版本不同表现为 `0` 或 `1424`。

推荐检查：

```python
valid = (
    data is not None
    and data.size > 0
    and data.nbytes == MyType.itemsize
    and int(data["TestValue"][0]) == 0x123456789ABCDEF
)
```

需要区分：

```python
data["len"][0]   # 控制器填写的协议字段
data.nbytes      # NumPy当前记录占用的字节数
MyType.itemsize # 本地结构定义的帧大小
```

## 11. 推荐使用 `FeedbackMonitor`

项目的 `feedback_monitor.py` 已经封装：

- 后台反馈线程；
- `TestValue` 和结构大小检查；
- NumPy 到普通 Python 数据的转换；
- 带锁的最新状态快照；
- 接收时间戳和反馈故障传递；
- 安全关闭阻塞 Socket。

基本使用：

```python
import time
from feedback_monitor import FeedbackMonitor, robot_mode_name

robot_ip = "192.168.5.1"

with FeedbackMonitor(robot_ip) as monitor:
    state = monitor.wait_until_ready(timeout=3.0)

    for _ in range(10):
        monitor.raise_if_failed()
        state = monitor.get_latest()

        if state is not None:
            age_ms = (time.monotonic() - state["recv_time"]) * 1000
            print("模式:", robot_mode_name(state["robot_mode"]))
            print("反馈年龄(ms):", round(age_ms, 1))
            print("关节角:", state["joints"])
            print("TCP位姿:", state["pose"])

        time.sleep(0.5)
```

控制程序不应长期使用过期快照。闭环控制中必须检查：

```python
feedback_age = time.monotonic() - state["recv_time"]
```

超过控制器允许的阈值时停止发送新目标。

## 12. 机器人模式

项目中的常用映射：

| 数值 | 模式 |
| ---: | --- |
| 1 | `INIT` |
| 2 | `BRAKE_OPEN` |
| 4 | `DISABLED` |
| 5 | `ENABLE` |
| 6 | `BACKDRIVE` |
| 7 | `RUNNING` |
| 8 | `RECORDING` |
| 9 | `ERROR` |
| 10 | `PAUSE` |
| 11 | `JOG` |

典型状态变化：

```text
未使能：RobotMode=4，EnableStatus=False
空闲使能：RobotMode=5，EnableStatus=True
运动中：RobotMode=7
报警：RobotMode=9，ErrorStatus=True
```

## 13. 判断运动是否完成

运动命令成功返回时通常包含算法队列 `ResultID`。推荐流程：

1. 发送 `MovJ` 或 `MovL`；
2. 检查返回错误码；
3. 解析返回的 `ResultID`；
4. 持续读取反馈中的 `CurrentCommandId` 和 `RobotMode`；
5. 目标指令 ID 已执行且机器人回到空闲使能状态时判定完成；
6. 设置超时，并在报警、下使能或反馈中断时立即退出等待。

不要只用固定 `sleep()` 猜测运动完成时间。原始 `DobotDemo.RunPoint()` 展示了基本思想，但没有超时和完整故障处理，不宜直接作为生产实现。

## 14. TCP 和线程相关的已知坑

### 14.1 单次 `recv()` 不等于一帧

TCP 是字节流。一次 `recv()` 可能返回半帧、多帧或一帧加下一帧的一部分，不能假设调用边界就是消息边界。

### 14.2 残包不能随意丢弃

若一次收到一个完整帧加部分下一帧，解析完整帧后应保留末尾残包。当前 `feedBackData()` 取最新完整块的实现仍值得在高可靠场景中继续审查。

### 14.3 反馈接收必须只有一个消费者

同一个 `30004` Socket 只能由一个线程读取，否则两个消费者会拆散同一条 TCP 字节流。

### 14.4 Dashboard 命令必须保持一发一回

虽然 `sendRecvMsg()` 内部有锁，但应用仍应让一个控制线程拥有 Dashboard 连接，尤其是在连续发送 `ServoP` 时。

### 14.5 Socket 关闭必须可重复调用

项目的 `close()` 已将 Socket 先置为 `None`，可避免显式 `close()` 与 `__del__()` 重复关闭导致的 Windows `10038` 错误。

关闭反馈线程的正确顺序：

```text
设置停止事件
→ 关闭Socket以解除阻塞recv
→ 等待反馈线程退出
→ 清理对象
```

## 15. 力反馈和导纳控制注意事项

仓库中同时存在：

```python
ActualTCPForce
SixForceValue
```

在闭环控制前必须通过实机实验确认：

- 使用哪组数据；
- 分量顺序是否为 `[Fx,Fy,Fz,Mx,My,Mz]`；
- 数据所在坐标系；
- 力和力矩单位；
- 静态零偏与重力影响；
- `ActualQuaternion` 的分量顺序和旋转方向约定。

导纳控制通常形成：

```text
力反馈
→ 零偏补偿
→ 坐标系转换
→ 滤波和死区
→ 导纳模型
→ 位移/速度/单步/工作空间限幅
→ ServoP目标
```

Windows、Python 和 TCP 组成的是软实时系统。连续伺服应记录实际周期，并在反馈过期、周期连续超限、机器人报警、力超限或目标越界时停止。

## 16. 新 Agent 推荐阅读顺序

接手项目后按以下顺序阅读：

1. 本文：建立项目全局认知；
2. `dobot_api.py` 的 `MyType`、`DobotApi`、`DobotApiDashboard`、`DobotApiFeedBack`；
3. `Feedback调试问题总结.md`：理解反馈调试历史；
4. `feedback_monitor.py`：理解当前反馈线程和快照格式；
5. `DobotDemo.py`：理解最基础的运动完成判断；
6. 官方接口 PDF：核对将要使用的具体命令；
7. 最后再阅读导纳、力跟踪或滑模控制代码。

## 17. 推荐的最小验证顺序

实机验证应逐步进行：

1. `29999` 只读查询 `RobotMode/GetAngle/GetPose`；
2. `30004` 只读反馈，确认 `TestValue`、关节角和位姿；
3. 后台反馈线程启动和干净退出；
4. 清除已排除原因的报警；
5. 使能，并通过反馈确认 `EnableStatus`；
6. 低速运动到示教器确认过的安全点；
7. 使用 `CurrentCommandId` 和模式判断完成；
8. 读取 DI/DO；
9. 最后再进行 `ServoP`、力控或导纳控制。

## 18. Agent 开始改代码前的检查清单

- 当前任务使用哪个机器人型号和固件版本？
- 实际机器人 IP 是什么？
- 当前 `MyType.itemsize` 是多少？
- 用户/工具坐标系编号和单位是什么？
- 目标点是否由现场示教确认？
- 运动接口返回格式是否与当前固件一致？
- 是否只有一个线程读取反馈？
- 是否只有一个线程发送 Dashboard 命令？
- 是否有反馈超时、运动超时和报警退出条件？
- 力数据字段、坐标系和符号是否已经实机确认？
- 无法实机验证的结论是否被明确标注为未验证？

## 19. 一句话总结

Dobot Python 二次开发的核心是：使用 `29999` 串行发送控制和查询命令，使用独立线程从 `30004` 持续获得经过校验的最新状态快照，再由业务层依据状态、指令 ID、超时和安全边界组织运动或闭环控制。
