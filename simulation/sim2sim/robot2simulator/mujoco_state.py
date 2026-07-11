from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Sequence

import numpy as np

from sim2sim.robot2simulator.sim_types import JointState

if TYPE_CHECKING:  # pragma: no cover
    import mujoco


def read_12dof_joint_state(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    actuator_names: Sequence[str],
    *,
    actuator_ids: Optional[np.ndarray] = None,
    qpos_adrs: Optional[np.ndarray] = None,
    dof_adrs: Optional[np.ndarray] = None,
    out_qpos: Optional[np.ndarray] = None,
    out_qvel: Optional[np.ndarray] = None,
    out_tau: Optional[np.ndarray] = None,
) -> JointState:
    import mujoco

    num_actuators = len(actuator_names)
    qpos = out_qpos if out_qpos is not None else np.empty(num_actuators, dtype=np.float32)
    qvel = out_qvel if out_qvel is not None else np.empty(num_actuators, dtype=np.float32)
    tau = out_tau if out_tau is not None else np.empty(num_actuators, dtype=np.float32)

    if actuator_ids is None or qpos_adrs is None or dof_adrs is None:
        actuator_ids = np.empty(num_actuators, dtype=np.int32)
        qpos_adrs = np.empty(num_actuators, dtype=np.int32)
        dof_adrs = np.empty(num_actuators, dtype=np.int32)
        for i, act_name in enumerate(actuator_names):
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if aid < 0:
                raise RuntimeError(f"找不到 actuator: {act_name}")

            trnid = model.actuator_trnid[aid]
            joint_id = int(trnid[0])
            actuator_ids[i] = int(aid)
            qpos_adrs[i] = int(model.jnt_qposadr[joint_id])
            dof_adrs[i] = int(model.jnt_dofadr[joint_id])

    qpos[:] = data.qpos[qpos_adrs]
    qvel[:] = data.qvel[dof_adrs]
    tau[:] = data.actuator_force[actuator_ids]

    return JointState(qpos=qpos, qvel=qvel, torque=tau)
