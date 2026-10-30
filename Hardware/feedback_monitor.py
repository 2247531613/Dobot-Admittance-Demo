"""Dobot 30004 端口实时反馈监视器。

这个模块只负责“读取机器人状态”，不负责使能机器人或发送运动指令。
以后做导纳控制时，控制循环可以调用 ``get_latest()`` 取得最新状态，
而不需要直接处理 Socket、NumPy 结构或线程同步。

线程分工：

    主线程 / 控制线程                  反馈线程
    ----------------                  --------
    monitor.get_latest()              feedBackData()
             |                             |
             +------ latest 快照 <---------+

30004 的 ``feedBackData()`` 是阻塞调用：暂时没有数据时，它会停在
Socket.recv() 等待。因此必须放进后台线程，否则主线程也会被卡住。
"""

from __future__ import annotations

import copy
import numpy as np
import threading
import time
from typing import Any

from Hardware.dobot_api import DobotApiFeedBack, MyType


# Dobot V4 反馈帧中的固定校验值。它可以用来判断数据是否从正确位置解析。
FEEDBACK_TEST_VALUE = 0x123456789ABCDEF


class FeedbackMonitor:
    """在后台线程中持续读取 Dobot 实时反馈。

    一个 ``FeedbackMonitor`` 对象持有：

    1. 一个连接到 30004 端口的 ``DobotApiFeedBack``；
    2. 一个专门执行 ``feedBackData()`` 的后台线程；
    3. 一份由锁保护的最新状态快照。

    参数：
        ip: 机器人的 IP 地址，例如 ``"192.168.5.1"``。
        port: 实时反馈端口，V4 通常使用 30004。
    """

    def __init__(self, ip: str, port: int = 30004) -> None:
        self.ip = ip
        self.port = port

        # 这里先不连接机器人。真正的连接在 start() 中建立，便于控制生命周期。
        self._feed: DobotApiFeedBack | None = None

        # Thread 是后台线程对象。创建对象不等于启动线程，start() 才会启动。
        self._thread: threading.Thread | None = None

        # Event 是线程安全的“开关”。
        # 主线程调用 set() 请求停止；反馈线程调用 is_set() 查看是否应退出。
        self._stop_event = threading.Event()

        # Lock 用来保护多个线程共同访问的 _latest。
        # 同一时刻只允许一个线程进入 ``with self._lock`` 代码块。
        self._lock = threading.Lock()

        # _latest 始终保存“最近一帧”的普通 Python 字典。
        # 不保存所有历史帧，避免内存不断增长和控制器读取旧状态。
        self._latest: dict[str, Any] | None = None

        # 如果反馈线程发生异常，线程不能直接把异常抛给主线程，
        # 因此将异常保存下来，由主线程通过 raise_if_failed() 检查。
        self._last_error: BaseException | None = None

        # 用于观察反馈是否持续更新。
        self._frame_count = 0

    def start(self) -> None:
        """连接 30004 端口并启动反馈线程。

        重复调用 start() 不会重复创建线程。
        """
        if self.is_alive():
            return

        # 每次重新启动前清理上一次运行的状态。
        self._stop_event.clear()
        self._last_error = None
        self._latest = None
        self._frame_count = 0

        # DobotApiFeedBack 的构造函数内部创建 TCP Socket，并连接机器人。
        self._feed = DobotApiFeedBack(self.ip, self.port)

        self._thread = threading.Thread(
            target=self._receive_loop,  # 新线程启动后执行的方法
            name="DobotFeedback",      # 调试器中显示的线程名称
            daemon=True,                # 主程序异常退出时不阻止进程结束
        )
        self._thread.start()

    def _receive_loop(self) -> None:
        """后台反馈线程的入口，只由 Thread 调用。"""
        try:
            while not self._stop_event.is_set():
                # feedBackData() 最终会调用 Socket.recv()。
                # recv() 是阻塞的：没有反馈数据时，线程会在这里等待。
                feed = self._feed
                if feed is None:
                    break

                data = feed.feedBackData()

                # close() 可能在 recv() 返回期间发出了停止请求。
                # 此时不要再发布新快照，直接退出循环。
                if self._stop_event.is_set():
                    break

                if not self._is_valid_frame(data):
                    # 无效帧不覆盖上一份有效状态。
                    continue

                snapshot = self._make_snapshot(data)

                # 先在锁外完成数据转换，再在锁内快速替换引用。
                # 这样主线程等待锁的时间非常短。
                with self._lock:
                    self._frame_count += 1
                    snapshot["frame_count"] = self._frame_count
                    self._latest = snapshot

        except (OSError, ConnectionError) as exc:
            # close() 会关闭 Socket，从而唤醒正在阻塞的 recv()。
            # 如果这是主动关闭，就不把它视为故障。
            if not self._stop_event.is_set():
                self._last_error = exc

        except BaseException as exc:
            # 保存意外异常，让主线程可以发现反馈线程已经失败。
            self._last_error = exc

    @staticmethod
    def _is_valid_frame(data: Any) -> bool:
        """检查 NumPy 反馈结构是否可以安全使用。"""
        if data is None or data.size == 0:
            return False

        # data.nbytes 是 NumPy 实际解析出的字节数。
        # MyType.itemsize 是本地反馈结构定义的大小，当前 V4.6.2 为 1424。
        if data.nbytes != MyType.itemsize:
            return False

        # 同时检查固定 TestValue，防止从错误字节位置解析。
        return int(data["TestValue"][0]) == FEEDBACK_TEST_VALUE

    @staticmethod
    def _make_snapshot(data: Any) -> dict[str, Any]:
        """将 NumPy 反馈帧转换成控制程序易用的 Python 字典。

        ``np.frombuffer`` 返回长度为 1 的结构化数组，所以字段后需要 ``[0]``。
        ``tolist()`` 将 NumPy 数组复制成普通 list，避免业务层依赖底层缓冲区。
        """
        return {
            # 本机收到并解析这帧数据的单调时钟时间。
            # 导纳控制用它判断反馈是否过期，不能使用控制器时间替代。
            "recv_time": time.monotonic(),

            # 协议相关信息。protocol_len 在部分固件上会显示 1424。
            "protocol_len": int(data["len"][0]),
            "controller_timestamp": int(data["TimeStamp"][0]),

            # 机器人整体状态。
            "robot_mode": int(data["RobotMode"][0]),
            "enabled": bool(data["EnableStatus"][0]),
            "running": bool(data["RunningStatus"][0]),
            "error": bool(data["ErrorStatus"][0]),
            "command_id": int(data["CurrentCommandId"][0]),

            # 六关节实际角度，以及 TCP 实际位姿 [X,Y,Z,Rx,Ry,Rz]。
            "joints": data["QActual"][0].astype(float).tolist(),
            "pose": data["ToolVectorActual"][0].astype(float).tolist(),
            "user": data["User"],
            "tool": data["Tool"],
            "actual_quaternion": (
                data["ActualQuaternion"][0]
                .astype(float)
                .tolist()
            ),
            # TCP 实际速度。之后可用于速度监控和安全保护。
            "tcp_speed": data["TCPSpeedActual"][0].astype(float).tolist(),

            # 两组与力相关的反馈暂时都保留。
            # 后续要通过实验确认导纳控制应该采用哪一组，以及它们的坐标系。
            "actual_tcp_force_in_tool": (
                data["ActualTCPForce"][0].astype(float).tolist()
            ),
            "actual_tcp_force_in_base": wrench_tool_to_base(data["ActualTCPForce"][0].astype(float).tolist(), data["ActualQuaternion"][0].astype(float).tolist()),
            "six_force_online": bool(data["SixForceOnline"][0]),
            "six_force": data["SixForceValue"][0].astype(float).tolist(),

            # 数字 IO 是位掩码，例如最低位通常对应 DI1/DO1。
            "digital_inputs": int(data["DigitalInputs"][0]),
            "digital_outputs": int(data["DigitalOutputs"][0]),

        }

    def get_latest(self) -> dict[str, Any] | None:
        """线程安全地返回最新状态的深拷贝。

        返回副本而不是内部字典，是为了防止调用者执行
        ``state["pose"][2] = ...`` 时意外修改监视器保存的状态。
        """
        with self._lock:
            if self._latest is None:
                return None
            return copy.deepcopy(self._latest)

    def wait_until_ready(self, timeout: float = 3.0) -> dict[str, Any]:
        """等待第一帧有效反馈，并返回该状态快照。"""
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            self.raise_if_failed()

            state = self.get_latest()
            if state is not None:
                return state

            # 这里睡眠是为了避免主线程空转占满一个 CPU 核心。
            time.sleep(0.01)

        raise TimeoutError(f"{timeout} 秒内没有收到有效的 Dobot 反馈")

    def raise_if_failed(self) -> None:
        """如果后台线程发生异常，在调用此方法的线程中抛出异常。"""
        if self._last_error is not None:
            raise RuntimeError("Dobot 反馈线程异常退出") from self._last_error

        if self._thread is not None and not self._thread.is_alive():
            if not self._stop_event.is_set():
                raise RuntimeError("Dobot 反馈线程意外停止")

    def is_alive(self) -> bool:
        """反馈线程当前是否仍在运行。"""
        return self._thread is not None and self._thread.is_alive()

    def close(self) -> None:
        """请求反馈线程退出，并关闭 30004 Socket。

        关闭 Socket 很重要：如果反馈线程正阻塞在 recv()，单独设置 Event
        并不能唤醒它；关闭 Socket 后，recv() 会立即返回或抛出 OSError，
        反馈线程才能检查停止请求并退出。
        """
        self._stop_event.set()

        feed = self._feed
        self._feed = None
        if feed is not None:
            feed.close()

        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

        self._thread = None

    def __enter__(self) -> "FeedbackMonitor":
        """支持 ``with FeedbackMonitor(ip) as monitor`` 写法。"""
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        """离开 with 代码块时保证关闭连接。"""
        self.close()


def robot_mode_name(mode: int) -> str:
    """将 RobotMode 数字转换为便于阅读的名称。"""
    names = {
        1: "INIT",
        2: "BRAKE_OPEN",
        4: "DISABLED",
        5: "ENABLE",
        6: "BACKDRIVE",
        7: "RUNNING",
        8: "RECORDING",
        9: "ERROR",
        10: "PAUSE",
        11: "JOG",
    }
    return names.get(mode, f"UNKNOWN({mode})")


def read_io_bit(mask: int, index: int) -> int:
    """从 DI/DO 位掩码中读取一路信号，index 从 1 开始。"""
    if index < 1:
        raise ValueError("IO 编号必须从 1 开始")
    return (int(mask) >> (index - 1)) & 1

def quaternion_to_rotation_tool_to_base(quaternion):
    """
    quaternion顺序：
        [qw, qx, qy, qz]

    返回工具坐标系到基坐标系的旋转矩阵。
    """
    qw, qx, qy, qz = quaternion

    # 防止四元数存在微小数值误差
    norm = np.linalg.norm(quaternion)
    qw, qx, qy, qz = np.array(quaternion) / norm

    return np.array([
        [
            1 - 2 * (qy*qy + qz*qz),
            2 * (qx*qy - qz*qw),
            2 * (qx*qz + qy*qw),
        ],
        [
            2 * (qx*qy + qz*qw),
            1 - 2 * (qx*qx + qz*qz),
            2 * (qy*qz - qx*qw),
        ],
        [
            2 * (qx*qz - qy*qw),
            2 * (qy*qz + qx*qw),
            1 - 2 * (qx*qx + qy*qy),
        ],
    ])

def wrench_tool_to_base(wrench_tool, actual_quaternion):
    """
    wrench_tool:
        Tool Space下的 [Fx, Fy, Fz, Mx, My, Mz]

    actual_quaternion:
        当前TCP姿态 [qw, qx, qy, qz]
    """
    wrench_tool = np.asarray(wrench_tool, dtype=float)

    force_tool = wrench_tool[:3]
    moment_tool = wrench_tool[3:]

    rotation_tool_to_base = quaternion_to_rotation_tool_to_base(
        actual_quaternion
    )

    force_base = rotation_tool_to_base @ force_tool
    moment_base = rotation_tool_to_base @ moment_tool

    return np.concatenate([
        force_base,
        moment_base,
    ])

def main() -> None:
    """只读测试：打印最新位姿和力数据，不使能、不运动机器人。"""
    robot_ip = "192.168.5.1"

    # with 块结束时会自动调用 close()，包括 Ctrl+C 或发生异常的情况。
    with FeedbackMonitor(robot_ip) as monitor:
        # first_state = monitor.wait_until_ready(timeout=3.0)
        # print("已收到第一帧反馈，协议 len 字段:", first_state["protocol_len"])

        for _ in range(20000):
            time.sleep(1)
            monitor.raise_if_failed()

            state = monitor.get_latest()
            if state is None:
                continue

            age_ms = (time.monotonic() - state["recv_time"]) * 1000.0
            # print(
            #     f"frame={state['frame_count']} "
            #     f"mode={robot_mode_name(state['robot_mode'])} "
            #     f"age={age_ms:.1f}ms"
            # )
            # text = (
            #     f"pose: {[round(v, 3) for v in state['pose']]}  |  "
            #     f"ActualTCPForce_in_tool: {[round(v, 3) for v in state['actual_tcp_force_in_tool']]}  |  "
            #     f"ActualTCPForce_in_base: {[round(v, 3) for v in state['actual_tcp_force_in_base']]}  |  "
            #     f"SixForceOnline: {state['six_force_online']}"
            # )
            # print(text, flush=True)
            # print("user:",state["user"])
            # print("tool:",state["tool"])
            print("pose",[round(v, 3) for v in state["pose"]])
            # print("ActualTCPForce_in_tool", [round(v, 8) for v in state['actual_tcp_force_in_tool']])
            # print("ActualTCPForce_in_base", state['actual_tcp_force_in_base'])
            print("ActualQuaternion",[round(v, 3) for v in  state['actual_quaternion']])
if __name__ == "__main__":
    main()
