"""采用隐式梯形积分的六维导纳控制器。

约定：

1. 六维顺序为 [X, Y, Z, Rx, Ry, Rz]；
2. 平移内部单位为 m，旋转内部单位为 rad；
3. 力和力矩均在 Base Space 表达，单位分别为 N 和 N*m；
4. 四元数顺序为 [qw, qx, qy, qz]；
5. 控制器只维护相对名义轨迹的导纳修正量。

本文件包含梯形积分控制器及应用层需要的四元数工具。
"""

import math
import numpy as np


def normalize_quaternion(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def quaternion_conjugate(q):
    qw, qx, qy, qz = q
    return np.array([qw, -qx, -qy, -qz])


def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def rotation_vector_to_quaternion(rotation_vector):
    rotation_vector = np.asarray(rotation_vector, dtype=float)
    angle = np.linalg.norm(rotation_vector)
    if angle < 1e-12:
        return normalize_quaternion(
            np.concatenate(([1.0], 0.5 * rotation_vector))
        )

    half_angle = 0.5 * angle
    return np.concatenate((
        [math.cos(half_angle)],
        rotation_vector / angle * math.sin(half_angle),
    ))


def quaternion_to_rotation_vector(q):
    q = normalize_quaternion(q)
    if q[0] < 0.0:
        q = -q

    vector = q[1:]
    vector_norm = np.linalg.norm(vector)
    if vector_norm < 1e-12:
        return 2.0 * vector

    return vector / vector_norm * (
        2.0 * math.atan2(vector_norm, q[0])
    )


def euler_zyx_deg_to_quaternion(euler_deg):
    rx, ry, rz = np.radians(np.asarray(euler_deg, dtype=float))
    qx = rotation_vector_to_quaternion([rx, 0.0, 0.0])
    qy = rotation_vector_to_quaternion([0.0, ry, 0.0])
    qz = rotation_vector_to_quaternion([0.0, 0.0, rz])
    return normalize_quaternion(
        quaternion_multiply(qz, quaternion_multiply(qy, qx))
    )


def quaternion_to_euler_zyx_deg(q):
    qw, qx, qy, qz = normalize_quaternion(q)
    rotation = np.array([
        [
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ],
        [
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ],
        [
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ],
    ])
    horizontal = math.hypot(rotation[0, 0], rotation[1, 0])

    if horizontal > 1e-8:
        rx = math.atan2(rotation[2, 1], rotation[2, 2])
        ry = math.atan2(-rotation[2, 0], horizontal)
        rz = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        rx = math.atan2(-rotation[1, 2], rotation[1, 1])
        ry = math.atan2(-rotation[2, 0], horizontal)
        rz = 0.0

    return np.degrees([rx, ry, rz])


def unwrap_euler_deg(euler_deg, previous_euler_deg):
    euler_deg = np.asarray(euler_deg, dtype=float).copy()
    previous = np.asarray(previous_euler_deg, dtype=float)
    euler_deg += 360.0 * np.round((previous - euler_deg) / 360.0)
    return euler_deg


class Admittance6D:
    """使用隐式梯形法积分的六维导纳控制器。

    连续导纳方程为：

        M * acceleration + B * velocity + K * offset = wrench

    六个方向目前互相解耦，因此 inertia、damping、stiffness 都是六维
    数组。前三项对应平移，后三项对应旋转。
    """

    def __init__(
        self,
        inertia=np.array([5.0, 5.0, 5.0, 1.0, 1.0, 1.0]),
        damping=np.array([500.0, 500.0, 500.0, 10.0, 10.0, 10.0]),
        stiffness=np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0]),
        admittance_enabled=None,
        wrench_filter=None,
    ):
        self.inertia = np.asarray(inertia, dtype=float)
        self.damping = np.asarray(damping, dtype=float)
        self.stiffness = np.asarray(stiffness, dtype=float)
        self.wrench_filter = wrench_filter

        if admittance_enabled is None:
            admittance_enabled = np.ones(6, dtype=bool)
        self.admittance_enabled = np.asarray(admittance_enabled, dtype=bool)

        self.translation_enabled = self.admittance_enabled[:3]
        self.rotation_enabled = self.admittance_enabled[3:]

        # 平移状态。
        self.offset = np.zeros(3)              # m
        self.velocity = np.zeros(3)            # m/s
        self.acceleration = np.zeros(3)        # m/s^2

        # 旋转状态。累计姿态修正始终用四元数维护。
        self.rotation_error = np.zeros(3)      # rad
        self.angular_velocity = np.zeros(3)    # rad/s
        self.angular_acceleration = np.zeros(3)  # rad/s^2
        self.total_delta_quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        # 第一次更新时还没有上一周期的加速度，用当前力初始化它。
        self._initialized = False

    def reset(self):
        """清空相对名义轨迹的全部导纳修正状态。"""
        self.offset = np.zeros(3)
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)

        self.rotation_error = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.angular_acceleration = np.zeros(3)
        self.total_delta_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self._initialized = False

        if self.wrench_filter is not None:
            self.wrench_filter.reset()

    @staticmethod
    def _solve_trapezoidal_velocity(
        mass,
        damping,
        stiffness,
        old_offset,
        old_velocity,
        old_acceleration,
        load,
        dt,
    ):
        """按逐轴隐式梯形公式求周期末速度。"""
        effective_mass = (
            mass
            + 0.5 * dt * damping
            + 0.25 * dt**2 * stiffness
        )

        right_hand_side = (
            mass * old_velocity
            + 0.5 * dt * mass * old_acceleration
            + 0.5 * dt * load
            - 0.5 * dt * stiffness * old_offset
            - 0.25 * dt**2 * stiffness * old_velocity
        )
        return right_hand_side / effective_mass

    def update(self, controller_wrench_base, dt):
        """推进一个周期，返回平移修正和累计姿态修正四元数。

        controller_wrench_base: Base Space 六维力 [Fx, Fy, Fz, Tx, Ty, Tz]。
        dt: 本次实际控制周期，单位 s。

        如果构造时传入 wrench_filter，则先调用其 update(wrench, dt)；
        wrench_filter 为 None 时不滤波。
        """
        controller_wrench_base = np.asarray(controller_wrench_base, dtype=float)
        if controller_wrench_base.shape != (6,):
            raise ValueError(
                "controller_wrench_base 必须为六维数组 "
                "[Fx, Fy, Fz, Tx, Ty, Tz]"
            )
        if dt <= 0.0:
            raise ValueError("dt 必须大于 0")

        if self.wrench_filter is not None:
            controller_wrench_base = np.asarray(
                self.wrench_filter.update(controller_wrench_base, dt),
                dtype=float,
            )
            if controller_wrench_base.shape != (6,):
                raise ValueError("wrench_filter.update() 必须返回六维数组")

        # 关闭的自由度既不接受外力，也不保留内部运动状态。
        controller_wrench_base = controller_wrench_base * self.admittance_enabled

        self.offset *= self.translation_enabled
        self.velocity *= self.translation_enabled
        self.acceleration *= self.translation_enabled
        self.angular_velocity *= self.rotation_enabled
        self.angular_acceleration *= self.rotation_enabled

        self.rotation_error = quaternion_to_rotation_vector(
            self.total_delta_quaternion
        ) * self.rotation_enabled

        # 梯形法需要周期起点加速度。第一次调用时，用当前导纳方程初始化。
        if not self._initialized:
            self.acceleration = (
                controller_wrench_base[:3]
                - self.damping[:3] * self.velocity
                - self.stiffness[:3] * self.offset
            ) / self.inertia[:3]
            self.angular_acceleration = (
                controller_wrench_base[3:]
                - self.damping[3:] * self.angular_velocity
                - self.stiffness[3:] * self.rotation_error
            ) / self.inertia[3:]
            self.acceleration *= self.translation_enabled
            self.angular_acceleration *= self.rotation_enabled
            self._initialized = True

        old_offset = self.offset.copy()
        old_velocity = self.velocity.copy()
        old_acceleration = self.acceleration.copy()
        old_angular_velocity = self.angular_velocity.copy()
        old_angular_acceleration = self.angular_acceleration.copy()

        # 隐式求出周期末线速度。
        new_velocity = self._solve_trapezoidal_velocity(
            self.inertia[:3],
            self.damping[:3],
            self.stiffness[:3],
            old_offset,
            old_velocity,
            old_acceleration,
            controller_wrench_base[:3],
            dt,
        )
        new_velocity *= self.translation_enabled

        # 位移使用周期起点和终点速度的平均值积分。
        new_offset = old_offset + 0.5 * dt * (
            old_velocity + new_velocity
        )
        new_offset *= self.translation_enabled

        # 隐式求出周期末角速度。刚度项使用周期起点的旋转误差，
        # 周期末真实姿态随后通过四元数复合得到。
        new_angular_velocity = self._solve_trapezoidal_velocity(
            self.inertia[3:],
            self.damping[3:],
            self.stiffness[3:],
            self.rotation_error,
            old_angular_velocity,
            old_angular_acceleration,
            controller_wrench_base[3:],
            dt,
        )
        new_angular_velocity *= self.rotation_enabled

        # 梯形积分使用平均角速度。Base Space 增量采用左乘。
        delta_rotation = 0.5 * dt * (
            old_angular_velocity + new_angular_velocity
        )
        delta_rotation *= self.rotation_enabled
        delta_quaternion = rotation_vector_to_quaternion(delta_rotation)
        self.total_delta_quaternion = normalize_quaternion(
            quaternion_multiply(
                delta_quaternion,
                self.total_delta_quaternion,
            )
        )

        # 四元数复合后重新映射并应用自由度开关。
        self.rotation_error = quaternion_to_rotation_vector(
            self.total_delta_quaternion
        ) * self.rotation_enabled
        self.total_delta_quaternion = rotation_vector_to_quaternion(
            self.rotation_error
        )

        self.offset = new_offset
        self.velocity = new_velocity
        self.angular_velocity = new_angular_velocity

        # 根据周期末状态重新计算加速度，供下一周期使用。
        self.acceleration = (
            controller_wrench_base[:3]
            - self.damping[:3] * self.velocity
            - self.stiffness[:3] * self.offset
        ) / self.inertia[:3]
        self.acceleration *= self.translation_enabled

        self.angular_acceleration = (
            controller_wrench_base[3:]
            - self.damping[3:] * self.angular_velocity
            - self.stiffness[3:] * self.rotation_error
        ) / self.inertia[3:]
        self.angular_acceleration *= self.rotation_enabled

        return self.offset.copy(), self.total_delta_quaternion.copy()


def main():
    """离线示例：施加六维恒定载荷，观察梯形积分结果。"""
    controller = Admittance6D()
    controller.reset()
    dt = 0.008

    for step in range(500):
        current_time = step * dt
        wrench = np.array([
            1.0 if current_time < 2.0 else 0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0 if current_time < 2.0 else 0.0,
        ])

        offset, delta_quaternion = controller.update(wrench, dt)

        if step % 50 == 0:
            delta_euler = quaternion_to_euler_zyx_deg(delta_quaternion)
            print(
                f"t={current_time:.2f}s  "
                f"offset_mm={np.round(offset * 1000.0, 3)}  "
                f"delta_euler_deg={np.round(delta_euler, 3)}"
            )


if __name__ == "__main__":
    main()
