from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ImuSample:
    quat_wxyz: np.ndarray  # (4,)
    gyro_xyz: np.ndarray  # (3,)
    accel_xyz: np.ndarray  # (3,)


@dataclass(frozen=True)
class JointState:
    qpos: np.ndarray  # (n,)
    qvel: np.ndarray  # (n,)
    torque: np.ndarray  # (n,)

