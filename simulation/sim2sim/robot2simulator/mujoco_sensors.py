from __future__ import annotations

from typing import Optional, TYPE_CHECKING

import numpy as np

from sim2sim.robot2simulator.sim_types import ImuSample

if TYPE_CHECKING:  # pragma: no cover
    import mujoco


def _read_sensor(model: "mujoco.MjModel", data: "mujoco.MjData", name: str) -> Optional[np.ndarray]:
    import mujoco

    try:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    except Exception:
        return None
    if sid < 0:
        return None
    adr = int(model.sensor_adr[sid])
    dim = int(model.sensor_dim[sid])
    return np.asarray(data.sensordata[adr : adr + dim], dtype=np.float32).copy()


def _resolve_sensor_slice(model: "mujoco.MjModel", name: str) -> Optional[tuple[int, int]]:
    import mujoco

    try:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    except Exception:
        return None
    if sid < 0:
        return None
    return int(model.sensor_adr[sid]), int(model.sensor_dim[sid])


def read_imu(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    *,
    orientation_slice: Optional[tuple[int, int]] = None,
    gyro_slice: Optional[tuple[int, int]] = None,
    accel_slice: Optional[tuple[int, int]] = None,
    out_quat: Optional[np.ndarray] = None,
    out_gyro: Optional[np.ndarray] = None,
    out_accel: Optional[np.ndarray] = None,
) -> ImuSample:
    quat = out_quat if out_quat is not None else np.empty(4, dtype=np.float32)
    gyro = out_gyro if out_gyro is not None else np.empty(3, dtype=np.float32)
    accel = out_accel if out_accel is not None else np.empty(3, dtype=np.float32)

    if orientation_slice is None:
        orientation_slice = _resolve_sensor_slice(model, "orientation")
    if orientation_slice is not None and int(orientation_slice[1]) == 4:
        adr, dim = orientation_slice
        quat[:] = data.sensordata[adr : adr + dim]
    else:
        quat[:] = data.qpos[3:7]

    if gyro_slice is None:
        gyro_slice = _resolve_sensor_slice(model, "gyro")
    if gyro_slice is not None and int(gyro_slice[1]) == 3:
        adr, dim = gyro_slice
        gyro[:] = data.sensordata[adr : adr + dim]
    else:
        gyro[:] = data.qvel[3:6]

    if accel_slice is None:
        accel_slice = _resolve_sensor_slice(model, "accelerometer")
    if accel_slice is not None and int(accel_slice[1]) == 3:
        adr, dim = accel_slice
        accel[:] = data.sensordata[adr : adr + dim]
    else:
        accel.fill(0.0)

    return ImuSample(quat_wxyz=quat, gyro_xyz=gyro, accel_xyz=accel)
