"""用于实时六维力数据的滤波器。

输入和输出顺序均为 [Fx, Fy, Fz, Tx, Ty, Tz]。
"""

from collections import deque
import math

import numpy as np


class FirstOrderLowPassFilter:
    """一阶低通滤波器。"""

    def __init__(self, cutoff_hz=5.0):
        self.cutoff_hz = cutoff_hz
        self.value = None

    def reset(self):
        self.value = None

    def update(self, sample, dt):
        sample = np.asarray(sample, dtype=float)
        if self.value is None:
            self.value = sample.copy()
            return self.value.copy()

        time_constant = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (time_constant + dt)
        self.value += alpha * (sample - self.value)
        return self.value.copy()


class FirstOrderLowPass1HzFilter(FirstOrderLowPassFilter):
    """截止频率默认为 1 Hz 的一阶低通滤波器。"""

    def __init__(self, cutoff_hz=1.0):
        super().__init__(cutoff_hz)


class MovingAverageFilter:
    """滑动窗口平均滤波器。"""

    def __init__(self, window_size=5):
        self.samples = deque(maxlen=window_size)

    def reset(self):
        self.samples.clear()

    def update(self, sample, dt=None):
        self.samples.append(np.asarray(sample, dtype=float).copy())
        return np.mean(self.samples, axis=0)


class MovingAverage3LowPass1HzFilter:
    """3 点滑动平均与 1 Hz 一阶低通串联滤波器。"""

    def __init__(self, window_size=3, cutoff_hz=1.0):
        self.moving_average = MovingAverageFilter(window_size)
        self.low_pass = FirstOrderLowPassFilter(cutoff_hz)

    def reset(self):
        self.moving_average.reset()
        self.low_pass.reset()

    def update(self, sample, dt):
        averaged = self.moving_average.update(sample)
        return self.low_pass.update(averaged, dt)


class MovingAverage3OneEuroFilter:
    """3 点滑动平均与 One Euro 串联滤波器。"""

    def __init__(
        self,
        window_size=3,
        min_cutoff_hz=1.0,
        beta=0.1,
        derivative_cutoff_hz=1.0,
    ):
        self.moving_average = MovingAverageFilter(window_size)
        self.one_euro = OneEuroFilter(
            min_cutoff_hz,
            beta,
            derivative_cutoff_hz,
        )

    def reset(self):
        self.moving_average.reset()
        self.one_euro.reset()

    def update(self, sample, dt):
        averaged = self.moving_average.update(sample)
        return self.one_euro.update(averaged, dt)


class MedianFilter:
    """逐通道滑动中值滤波器。"""

    def __init__(self, window_size=5):
        self.samples = deque(maxlen=window_size)

    def reset(self):
        self.samples.clear()

    def update(self, sample, dt=None):
        self.samples.append(np.asarray(sample, dtype=float).copy())
        return np.median(self.samples, axis=0)


class ButterworthLowPassFilter:
    """使用二阶节实时计算的二阶 Butterworth 低通滤波器。"""

    def __init__(self, cutoff_hz=5.0):
        self.cutoff_hz = cutoff_hz
        self.z1 = None
        self.z2 = None

    def reset(self):
        self.z1 = None
        self.z2 = None

    def update(self, sample, dt):
        sample = np.asarray(sample, dtype=float)
        omega = math.tan(math.pi * self.cutoff_hz * dt)
        denominator = 1.0 + math.sqrt(2.0) * omega + omega**2
        b0 = omega**2 / denominator
        b1 = 2.0 * b0
        b2 = b0
        a1 = 2.0 * (omega**2 - 1.0) / denominator
        a2 = (
            1.0 - math.sqrt(2.0) * omega + omega**2
        ) / denominator

        if self.z1 is None:
            self.z1 = (1.0 - b0) * sample
            self.z2 = (b2 - a2) * sample

        value = b0 * sample + self.z1
        self.z1 = b1 * sample - a1 * value + self.z2
        self.z2 = b2 * sample - a2 * value
        return value.copy()


class OneEuroFilter:
    """根据输入变化速度自适应调整截止频率的 One Euro 滤波器。"""

    def __init__(
        self,
        min_cutoff_hz=1.0,
        beta=0.1,
        derivative_cutoff_hz=1.0,
    ):
        self.min_cutoff_hz = min_cutoff_hz
        self.beta = beta
        self.derivative_cutoff_hz = derivative_cutoff_hz
        self.value = None
        self.derivative = None
        self.previous_sample = None

    @staticmethod
    def _alpha(cutoff_hz, dt):
        time_constant = 1.0 / (2.0 * math.pi * cutoff_hz)
        return dt / (time_constant + dt)

    def reset(self):
        self.value = None
        self.derivative = None
        self.previous_sample = None

    def update(self, sample, dt):
        sample = np.asarray(sample, dtype=float)
        if self.value is None:
            self.value = sample.copy()
            self.derivative = np.zeros_like(sample)
            self.previous_sample = sample.copy()
            return self.value.copy()

        raw_derivative = (sample - self.previous_sample) / dt
        derivative_alpha = self._alpha(self.derivative_cutoff_hz, dt)
        self.derivative += derivative_alpha * (
            raw_derivative - self.derivative
        )

        cutoff_hz = (
            self.min_cutoff_hz + self.beta * np.abs(self.derivative)
        )
        signal_alpha = self._alpha(cutoff_hz, dt)
        self.value += signal_alpha * (sample - self.value)
        self.previous_sample = sample.copy()
        return self.value.copy()


class TrackingDifferentiatorFilter:
    """二阶 TD（跟踪微分器）滤波器，输出平滑后的跟踪值。

    tracking_rate 越大，跟随越快但滤波越弱；damping_ratio 默认为
    临界阻尼 1.0。内部同时估计输入的一阶变化率 derivative。
    """

    def __init__(self, tracking_rate=10.0, damping_ratio=1.0):
        if tracking_rate <= 0.0:
            raise ValueError("tracking_rate must be positive")
        if damping_ratio <= 0.0:
            raise ValueError("damping_ratio must be positive")
        self.tracking_rate = tracking_rate
        self.damping_ratio = damping_ratio
        self.value = None
        self.derivative = None

    def reset(self):
        self.value = None
        self.derivative = None

    def update(self, sample, dt):
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        sample = np.asarray(sample, dtype=float)
        if self.value is None:
            self.value = sample.copy()
            self.derivative = np.zeros_like(sample)
            return self.value.copy()

        rate = self.tracking_rate
        acceleration = (
            rate**2 * (sample - self.value)
            - 2.0 * self.damping_ratio * rate * self.derivative
        )
        self.derivative += acceleration * dt
        self.value += self.derivative * dt
        return self.value.copy()
