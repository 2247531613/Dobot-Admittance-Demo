"""Dobot 六维导纳拖动最小示例。

数据链路：30004 实时反馈 -> 力死区/低通 -> 六维导纳 -> 29999 ServoP。

约定：
1. 位姿和 ServoP 平移单位为 mm，导纳控制器内部平移单位为 m；
2. 四元数顺序为 [qw, qx, qy, qz]；
3. 六维力顺序为 [Fx, Fy, Fz, Tx, Ty, Tz]，并在基坐标系中计算。
"""

import sys
import time
from pathlib import Path

import numpy as np


DRAG_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = Path(__file__).resolve().parents[2]
for import_root in (PROJECT_ROOT, DRAG_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from AdmittanceController.admittance_controller import (
    Admittance6D,
    quaternion_multiply,
    quaternion_to_euler_zyx_deg,
    unwrap_euler_deg,
)
from Filters.Filters import FirstOrderLowPassFilter
from Hardware.dobot_api import DobotApiDashboard
from Hardware.feedback_monitor import FeedbackMonitor


ROBOT_IP = "192.168.5.1"
CONTROL_PERIOD_S = 0.008   #控制周期
FEEDBACK_TIMEOUT_S = 0.1   #反馈超时

# 顺序：[X, Y, Z, Rx, Ry, Rz]。
ADMITTANCE_ENABLED = np.array([True, True, True, True, True, True])
INERTIA = np.array([15.0, 15.0, 15.0, 0.5, 0.5, 0.3])
DAMPING = np.array([120.0, 120.0, 120.0, 8.0, 8.0, 3.0])
STIFFNESS = np.zeros(6)

FORCE_DEADBAND_N = 5.0      #死区力
TORQUE_DEADBAND_NM = 1.0    #死区力矩
WRENCH_FILTER_CUTOFF_HZ = 1.0   #滤波器截至频率
STOP_FORCE_N = 500.0        #最大力
STOP_TORQUE_NM = 30.0       #最大力矩
EULER_SINGULARITY_MARGIN_DEG = 5.0   #奇异相关判断


def command_ok(reply):
    """Dobot 命令回复以 0 开头时表示命令已成功接收。"""
    return isinstance(reply, str) and reply.strip().startswith("0,")


def apply_deadband(wrench):
    """分别对力和力矩应用死区。"""
    thresholds = np.array(
        [FORCE_DEADBAND_N] * 3 + [TORQUE_DEADBAND_NM] * 3
    )
    return np.where(np.abs(wrench) < thresholds, 0.0, wrench)


def near_euler_singularity(euler_deg):
    """判断 ZYX 欧拉角的俯仰角是否接近正负 90 度。"""
    return abs(90.0 - abs(euler_deg[1])) < EULER_SINGULARITY_MARGIN_DEG


def main():
    dashboard = None
    monitor = None
    robot_enabled = False

    controller = Admittance6D(
        inertia=INERTIA,
        damping=DAMPING,
        stiffness=STIFFNESS,
        admittance_enabled=ADMITTANCE_ENABLED,
        wrench_filter=FirstOrderLowPassFilter(WRENCH_FILTER_CUTOFF_HZ),
    )

    try:
        dashboard = DobotApiDashboard(ROBOT_IP, 29999)
        monitor = FeedbackMonitor(ROBOT_IP)
        monitor.start()

        state = monitor.wait_until_ready(timeout=3.0)
        if not state["six_force_online"]:
            raise RuntimeError("六维力传感器不在线")

        print("当前位姿:", np.round(state["pose"], 3))
        input("确认现场安全后按 Enter 置零力传感器并使能机器人……")

        reply = dashboard.SixForceHome()
        if not command_ok(reply):
            raise RuntimeError(f"SixForceHome 失败: {reply}")
        time.sleep(0.5)

        reply = dashboard.EnableRobot()
        if not command_ok(reply):
            raise RuntimeError(f"EnableRobot 失败: {reply}")
        robot_enabled = True
        time.sleep(1.0)

        state = monitor.get_latest()
        if state is None:
            raise RuntimeError("使能后未取得机器人反馈")

        initial_pose = np.asarray(state["pose"], dtype=float)
        initial_position_mm = initial_pose[:3].copy()
        initial_quaternion = np.asarray(
            state["actual_quaternion"], dtype=float
        )
        previous_command_euler_deg = initial_pose[3:].copy()
        controller.reset()

        print("导纳拖动已启动，按 Ctrl+C 停止")
        previous_time = None
        next_time = time.monotonic()

        while True:
            monitor.raise_if_failed()
            now = time.monotonic()
            # Windows 的单调时钟在连续两次读取时可能返回相同值。
            # 首周期直接使用目标周期，避免把 dt=0 传给导纳控制器。
            dt = (
                CONTROL_PERIOD_S
                if previous_time is None
                else now - previous_time
            )
            previous_time = now
            if dt <= 0.0:
                continue

            state = monitor.get_latest()
            if state is None or now - state["recv_time"] > FEEDBACK_TIMEOUT_S:
                raise RuntimeError("实时反馈超时")
            if state["error"] or not state["enabled"]:
                raise RuntimeError("机器人报警或已下使能")

            wrench_base = np.asarray(
                state["actual_tcp_force_in_base"], dtype=float
            )
            if np.linalg.norm(wrench_base[:3]) > STOP_FORCE_N:
                raise RuntimeError("外力超过停止阈值")
            if np.linalg.norm(wrench_base[3:]) > STOP_TORQUE_NM:
                raise RuntimeError("外力矩超过停止阈值")

            correction_position_m, correction_quaternion = controller.update(
                apply_deadband(wrench_base), dt
            )
            command_position_mm = (
                initial_position_mm + correction_position_m * 1000.0
            )
            command_quaternion = quaternion_multiply(
                correction_quaternion, initial_quaternion
            )
            command_euler_deg = quaternion_to_euler_zyx_deg(
                command_quaternion
            )
            command_euler_deg = unwrap_euler_deg(
                command_euler_deg, previous_command_euler_deg
            )
            if near_euler_singularity(command_euler_deg):
                raise RuntimeError("目标姿态接近欧拉角奇异区")

            command_pose = np.concatenate(
                (command_position_mm, command_euler_deg)
            )
            reply = dashboard.ServoP(
                *command_pose,
                t=CONTROL_PERIOD_S,
                aheadtime=50,
                gain=500,
            )
            if not command_ok(reply):
                raise RuntimeError(f"ServoP 失败: {reply}")

            previous_command_euler_deg = command_euler_deg
            print(
                "力/力矩(Base):",
                np.round(wrench_base, 2),
                end="\r",
            )

            next_time += CONTROL_PERIOD_S
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()

    except KeyboardInterrupt:
        print("\n用户停止导纳拖动")
    except (ConnectionError, OSError, RuntimeError, TimeoutError) as exc:
        print(f"\n程序停止: {exc}")
    finally:
        if dashboard is not None:
            if robot_enabled:
                try:
                    dashboard.Stop()
                    dashboard.DisableRobot()
                except (ConnectionError, OSError):
                    pass
            dashboard.close()
        if monitor is not None:
            monitor.close()


if __name__ == "__main__":
    main()
