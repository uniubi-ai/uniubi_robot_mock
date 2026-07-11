from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from sim2sim.robot2simulator.joint_map import MotorHeader


MAX_MOTOR_NUM = 16


@dataclass(frozen=True)
class PowerObserved:
    power: float = 0.0
    health: float = 0.0
    temper: float = 0.0
    charge_current: float = 0.0
    charge_voltage: float = 0.0


@dataclass(frozen=True)
class MotionObserved:
    timestamp_us: int
    quat_wxyz: np.ndarray  # (4,)
    gyro_xyz: np.ndarray  # (3,)
    accel_xyz: np.ndarray  # (3,)
    motor_pos: np.ndarray  # (n,)
    motor_vel: np.ndarray  # (n,)
    motor_tau: np.ndarray  # (n,)
    motor_headers: Sequence[MotorHeader]
    imu_temp: float = 0.0
    motor_enable: np.ndarray | None = None  # (n,), optional
    motor_online: np.ndarray | None = None  # (n,), optional
    motor_error: np.ndarray | None = None  # (n,), optional
    power: PowerObserved = PowerObserved()


@dataclass(frozen=True)
class MotionCtrl:
    timestamp_us: int
    motor_pos: np.ndarray  # (n,)
    motor_vel: np.ndarray  # (n,)
    kp: np.ndarray  # (n,)
    kd: np.ndarray  # (n,)
    tau_ff: np.ndarray  # (n,)
    motor_headers: Sequence[MotorHeader]


@dataclass(frozen=True)
class MotionFaultItem:
    joint_index: int
    start_delay_s: float
    duration_s: float
    error_code: int


@dataclass(frozen=True)
class MotionFault:
    timestamp_us: int
    items: Sequence[MotionFaultItem]


@dataclass(frozen=True)
class MotionRecord:
    action: int
    observed: MotionObserved
