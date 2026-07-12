from __future__ import annotations

import contextlib
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ContextManager, Optional, Sequence

import numpy as np

from sim2sim.robot2simulator.backends.base import SimBackend
from sim2sim.robot2simulator.mujoco_sensors import read_imu
from sim2sim.robot2simulator.mujoco_state import read_12dof_joint_state
from sim2sim.robot2simulator.sim_types import ImuSample, JointState


@dataclass(frozen=True)
class MujocoBackendConfig:
    xml_path: str
    sim_dt: float = 0.005
    publish_hz: float = 200.0
    control_hz: float = 50.0
    realtime: bool = True
    headless: bool = False
    actuator_names: Sequence[str] = ()

    # init
    initial_joint_pos: Optional[np.ndarray] = None  # (n,)
    initial_base_pos_xyz: Optional[np.ndarray] = None  # (3,)
    settling_steps: int = 0

    # pd
    stiffness: Optional[np.ndarray] = None  # (n,)
    damping: Optional[np.ndarray] = None  # (n,)
    torque_limits: Optional[np.ndarray] = None  # (n,)
    armature: Optional[np.ndarray] = None  # (n,)
    torque_curve_x1: Optional[np.ndarray] = None  # (n,)
    torque_curve_x2: Optional[np.ndarray] = None  # (n,)
    torque_curve_y1: Optional[np.ndarray] = None  # (n,)
    torque_curve_y2: Optional[np.ndarray] = None  # (n,)
    friction_static: Optional[np.ndarray] = None  # (n,)
    friction_dynamic: Optional[np.ndarray] = None  # (n,)
    activation_velocity: Optional[np.ndarray] = None  # (n,)

    # debug
    dump_actuators: bool = False
    mj_debug: bool = False
    mj_debug_hz: float = 2.0


class MujocoBackend(SimBackend):
    def __init__(self, cfg: MujocoBackendConfig) -> None:
        import mujoco
        import mujoco.viewer

        self._mujoco = mujoco
        self._mujoco_viewer = mujoco.viewer

        actuator_names = tuple(cfg.actuator_names)
        super().__init__(
            actuator_names=actuator_names,
            sim_dt=cfg.sim_dt,
            realtime=cfg.realtime,
            headless=cfg.headless,
        )

        self._cfg = cfg
        xml_path = str(Path(cfg.xml_path).expanduser())
        self._model = mujoco.MjModel.from_xml_path(xml_path)
        self._model.opt.timestep = float(cfg.sim_dt)
        self._data = mujoco.MjData(self._model)

        self._actuator_ids = np.empty(len(self.actuator_names), dtype=np.int32)
        self._joint_ids = np.empty(len(self.actuator_names), dtype=np.int32)
        self._qpos_adrs = np.empty(len(self.actuator_names), dtype=np.int32)
        self._dof_adrs = np.empty(len(self.actuator_names), dtype=np.int32)
        for i, act_name in enumerate(self.actuator_names):
            aid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if aid < 0:
                raise RuntimeError(f"找不到 actuator: {act_name}")
            joint_id = int(self._model.actuator_trnid[aid][0])
            self._actuator_ids[i] = int(aid)
            self._joint_ids[i] = joint_id
            self._qpos_adrs[i] = int(self._model.jnt_qposadr[joint_id])
            self._dof_adrs[i] = int(self._model.jnt_dofadr[joint_id])

        def _resolve_sensor_slice(name: str) -> Optional[tuple[int, int]]:
            try:
                sid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            except Exception:
                return None
            if sid < 0:
                return None
            return int(self._model.sensor_adr[sid]), int(self._model.sensor_dim[sid])

        self._orientation_slice = _resolve_sensor_slice("orientation")
        self._gyro_slice = _resolve_sensor_slice("gyro")
        self._accel_slice = _resolve_sensor_slice("accelerometer")

        self._imu_quat = np.empty(4, dtype=np.float32)
        self._imu_gyro = np.empty(3, dtype=np.float32)
        self._imu_accel = np.empty(3, dtype=np.float32)
        self._joint_qpos = np.empty(len(self.actuator_names), dtype=np.float32)
        self._joint_qvel = np.empty(len(self.actuator_names), dtype=np.float32)
        self._joint_tau = np.empty(len(self.actuator_names), dtype=np.float32)
        n_act = len(self.actuator_names)
        self._q_des = np.zeros(n_act, dtype=np.float32)
        self._qd_des = np.zeros(n_act, dtype=np.float32)
        self._kp = np.zeros(n_act, dtype=np.float32)
        self._kd = np.zeros(n_act, dtype=np.float32)
        self._tau_ff = np.zeros(n_act, dtype=np.float32)
        self._tau_applied = np.zeros(n_act, dtype=np.float32)
        self._qfrc_buffer = np.zeros(self._model.nv, dtype=np.float32)
        self._tau_limits = self._resolve_torque_limits(cfg.torque_limits)
        default_effort = self._tau_limits if self._tau_limits is not None else np.full(n_act, np.inf, dtype=np.float32)
        self._armature = self._resolve_actuator_vector(cfg.armature, default=None)
        self._tn_x1 = self._resolve_actuator_vector(cfg.torque_curve_x1, default=np.inf)
        self._tn_x2 = self._resolve_actuator_vector(cfg.torque_curve_x2, default=np.inf)
        self._tn_y1 = self._resolve_actuator_vector(cfg.torque_curve_y1, default=default_effort)
        self._tn_y2 = self._resolve_actuator_vector(cfg.torque_curve_y2, default=self._tn_y1)
        self._friction_static = self._resolve_actuator_vector(cfg.friction_static, default=0.0)
        self._friction_dynamic = self._resolve_actuator_vector(cfg.friction_dynamic, default=0.0)
        self._activation_vel = self._resolve_actuator_vector(cfg.activation_velocity, default=0.01)

        self._next_mj_debug_t = 0.0
        self._last_kp: Optional[np.ndarray] = None
        self._last_kd: Optional[np.ndarray] = None
        self._fault_visual_color = np.asarray([1.0, 0.05, 0.0, 1.0], dtype=np.float32)
        self._fault_visual_geoms_by_joint = self._build_fault_visual_geom_map()
        self._fault_visual_original_rgba: dict[int, np.ndarray] = {}
        self._fault_visual_active_indices: tuple[int, ...] = ()

        self.reset()

    @property
    def publish_hz(self) -> float:
        return float(self._cfg.publish_hz)

    @property
    def control_hz(self) -> float:
        return float(self._cfg.control_hz)

    def reset(self) -> None:
        mujoco = self._mujoco
        mujoco.mj_resetData(self._model, self._data)
        self._disable_builtin_actuator_servos()
        self._apply_armature_if_present()

        stiffness = np.asarray(
            self._cfg.stiffness if self._cfg.stiffness is not None else np.zeros(len(self.actuator_names)),
            dtype=np.float32,
        )
        damping = np.asarray(
            self._cfg.damping if self._cfg.damping is not None else np.zeros(len(self.actuator_names)),
            dtype=np.float32,
        )
        if stiffness.shape[0] != len(self.actuator_names):
            raise RuntimeError(f"stiffness 维度不匹配：{stiffness.shape[0]} != {len(self.actuator_names)}")
        if damping.shape[0] != len(self.actuator_names):
            raise RuntimeError(f"damping 维度不匹配：{damping.shape[0]} != {len(self.actuator_names)}")
        self.set_pd_gains(stiffness, damping)

        if self._cfg.dump_actuators:
            self._dump_actuators()

        init_joint = self._cfg.initial_joint_pos
        if init_joint is None:
            init_joint = np.asarray(
                [
                    0.48,
                    1.10,
                    -2.72,  # FL
                    -0.48,
                    1.10,
                    -2.72,  # FR
                    0.48,
                    1.10,
                    -2.72,  # RL
                    -0.48,
                    1.10,
                    -2.72,  # RR
                ],
                dtype=np.float32,
            )
        init_joint = np.asarray(init_joint, dtype=np.float32)

        if init_joint.shape[0] != len(self.actuator_names):
            raise RuntimeError(
                f"initial_joint_pos 维度不匹配：{init_joint.shape[0]} != {len(self.actuator_names)}"
            )

        self._set_initial_joint_positions(init_joint)
        self.set_position_target(init_joint)
        self.set_velocity_target(np.zeros_like(init_joint))
        self.set_feedforward_torque(np.zeros_like(init_joint))
        self._data.qvel[:] = 0.0
        self._data.ctrl[:] = 0.0
        self._data.qfrc_applied[:] = 0.0

        base_pos = self._cfg.initial_base_pos_xyz
        if base_pos is None:
            base_pos = np.asarray([0.0, 0.0, 0.5], dtype=np.float32)
        base_pos = np.asarray(base_pos, dtype=np.float32)
        if self._data.qpos.shape[0] >= 3 and base_pos.shape == (3,):
            self._data.qpos[0:3] = base_pos

        mujoco.mj_forward(self._model, self._data)

        for _ in range(int(self._cfg.settling_steps)):
            self._apply_torque_control()
            mujoco.mj_step(self._model, self._data)

        self._next_mj_debug_t = 0.0
        self._last_kp = None
        self._last_kd = None

    def step(self) -> None:
        if self._cfg.mj_debug:
            now = time.time()
            if now >= self._next_mj_debug_t:
                self._print_mj_debug()
                self._next_mj_debug_t = now + 1.0 / max(float(self._cfg.mj_debug_hz), 1e-6)
        self._apply_torque_control()
        self._mujoco.mj_step(self._model, self._data)

    def set_position_target(self, target_pos: np.ndarray) -> None:
        target_pos = np.asarray(target_pos, dtype=np.float32)
        if target_pos.shape[0] != len(self.actuator_names):
            raise RuntimeError(f"target_pos 维度不匹配：{target_pos.shape[0]} != {len(self.actuator_names)}")
        self._q_des[:] = target_pos

    def set_velocity_target(self, target_vel: np.ndarray) -> None:
        target_vel = np.asarray(target_vel, dtype=np.float32)
        if target_vel.shape[0] != len(self.actuator_names):
            raise RuntimeError(f"target_vel 维度不匹配：{target_vel.shape[0]} != {len(self.actuator_names)}")
        self._qd_des[:] = target_vel

    def set_feedforward_torque(self, tau_ff: np.ndarray) -> None:
        tau_ff = np.asarray(tau_ff, dtype=np.float32)
        if tau_ff.shape[0] != len(self.actuator_names):
            raise RuntimeError(f"tau_ff 维度不匹配：{tau_ff.shape[0]} != {len(self.actuator_names)}")
        self._tau_ff[:] = tau_ff

    def set_pd_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        kp = np.asarray(kp, dtype=np.float32)
        kd = np.asarray(kd, dtype=np.float32)
        if (self._last_kp is not None) and (self._last_kd is not None):
            if np.array_equal(kp, self._last_kp) and np.array_equal(kd, self._last_kd):
                return
        self._apply_received_pd_gains(kp=kp, kd=kd)
        self._last_kp = kp.copy()
        self._last_kd = kd.copy()

    def get_imu(self) -> ImuSample:
        return read_imu(
            self._model,
            self._data,
            orientation_slice=self._orientation_slice,
            gyro_slice=self._gyro_slice,
            accel_slice=self._accel_slice,
            out_quat=self._imu_quat,
            out_gyro=self._imu_gyro,
            out_accel=self._imu_accel,
        )

    def get_joint_state(self) -> JointState:
        state = read_12dof_joint_state(
            self._model,
            self._data,
            self.actuator_names,
            actuator_ids=self._actuator_ids,
            qpos_adrs=self._qpos_adrs,
            dof_adrs=self._dof_adrs,
            out_qpos=self._joint_qpos,
            out_qvel=self._joint_qvel,
            out_tau=self._joint_tau,
        )
        np.copyto(self._joint_tau, self._tau_applied, casting="unsafe")
        return state

    def viewer_context(self, enabled: bool) -> ContextManager[Optional[Any]]:
        if (not enabled) or self.headless:
            return contextlib.nullcontext(None)
        return self._mujoco_viewer.launch_passive(self._model, self._data)

    def viewer_is_running(self, viewer: Any) -> bool:
        return bool(viewer.is_running())

    def viewer_sync(self, viewer: Any) -> None:
        viewer.sync()

    def set_fault_visual(self, joint_index: Optional[int], active: bool) -> None:
        if active and joint_index is not None:
            self.set_fault_visuals((int(joint_index),))
        elif not active:
            self.set_fault_visuals(())

    def set_fault_visuals(self, joint_indices: Sequence[int]) -> None:
        normalized = tuple(
            sorted(
                {
                    int(idx)
                    for idx in joint_indices
                    if 0 <= int(idx) < len(self._fault_visual_geoms_by_joint)
                }
            )
        )
        if normalized == self._fault_visual_active_indices:
            return

        self._restore_fault_visual()
        for idx in normalized:
            for geom_id in self._fault_visual_geoms_by_joint[idx]:
                if int(geom_id) not in self._fault_visual_original_rgba:
                    self._fault_visual_original_rgba[int(geom_id)] = np.asarray(
                        self._model.geom_rgba[int(geom_id)], dtype=np.float32
                    ).copy()
                self._model.geom_rgba[int(geom_id)] = self._fault_visual_color
        self._fault_visual_active_indices = normalized

    def close(self) -> None:
        self._restore_fault_visual()
        # mujoco viewer/context 会在 with 里自动释放；这里留空即可
        return None

    def _build_fault_visual_geom_map(self) -> list[list[int]]:
        geom_map: list[list[int]] = []
        for joint_id in self._joint_ids:
            body_id = int(self._model.jnt_bodyid[int(joint_id)])
            body_ids = self._body_subtree_ids(body_id)
            geoms = [
                geom_id
                for geom_id in range(int(self._model.ngeom))
                if int(self._model.geom_bodyid[geom_id]) in body_ids
            ]
            geom_map.append(geoms)
        return geom_map

    def _body_subtree_ids(self, root_body_id: int) -> set[int]:
        body_ids = {int(root_body_id)}
        changed = True
        while changed:
            changed = False
            for body_id in range(1, int(self._model.nbody)):
                parent_id = int(self._model.body_parentid[body_id])
                if parent_id in body_ids and body_id not in body_ids:
                    body_ids.add(body_id)
                    changed = True
        return body_ids

    def _restore_fault_visual(self) -> None:
        for geom_id, rgba in self._fault_visual_original_rgba.items():
            self._model.geom_rgba[int(geom_id)] = rgba
        self._fault_visual_original_rgba.clear()
        self._fault_visual_active_indices = ()

    def _set_initial_joint_positions(self, motor_pos: np.ndarray) -> None:
        self._data.qpos[self._qpos_adrs] = motor_pos

    def _resolve_torque_limits(self, cfg_limits: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if cfg_limits is not None:
            limits = np.asarray(cfg_limits, dtype=np.float32)
            if limits.shape[0] != len(self.actuator_names):
                raise RuntimeError(f"torque_limits 维度不匹配：{limits.shape[0]} != {len(self.actuator_names)}")
            return np.abs(limits)

        if not hasattr(self._model, "actuator_forcerange"):
            return None
        force_range = np.asarray(self._model.actuator_forcerange[self._actuator_ids], dtype=np.float32)
        if force_range.shape != (len(self.actuator_names), 2):
            return None
        limits = np.minimum(np.abs(force_range[:, 0]), np.abs(force_range[:, 1]))
        if not np.all(np.isfinite(limits)) or np.any(limits <= 0.0):
            return None
        return limits

    def _resolve_actuator_vector(
        self, value: Optional[np.ndarray], *, default: float | np.ndarray | None
    ) -> Optional[np.ndarray]:
        if value is None:
            if default is None:
                return None
            value_arr = np.asarray(default, dtype=np.float32)
        else:
            value_arr = np.asarray(value, dtype=np.float32)

        if value_arr.ndim == 0 or value_arr.size == 1:
            return np.full(len(self.actuator_names), float(value_arr.reshape(-1)[0]), dtype=np.float32)
        value_arr = value_arr.reshape(-1).astype(np.float32)
        if value_arr.shape[0] != len(self.actuator_names):
            raise RuntimeError(
                f"actuator parameter 维度不匹配：{value_arr.shape[0]} != {len(self.actuator_names)}"
            )
        return value_arr.copy()

    def _apply_armature_if_present(self) -> None:
        if self._armature is None:
            return
        if hasattr(self._model, "dof_armature"):
            self._model.dof_armature[self._dof_adrs] = self._armature

    def _disable_builtin_actuator_servos(self) -> None:
        if hasattr(self._model, "actuator_gainprm"):
            self._model.actuator_gainprm[self._actuator_ids, :] = 0.0
        if hasattr(self._model, "actuator_biasprm"):
            self._model.actuator_biasprm[self._actuator_ids, :] = 0.0
        if self._data.ctrl.size > 0:
            self._data.ctrl[:] = 0.0

    def _apply_received_pd_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        if kp.shape[0] != len(self.actuator_names) or kd.shape[0] != len(self.actuator_names):
            raise RuntimeError(
                f"kp/kd 维度不匹配：kp={kp.shape[0]}, kd={kd.shape[0]} vs actuator_names={len(self.actuator_names)}"
            )
        self._kp[:] = kp
        self._kd[:] = kd

    def _apply_torque_control(self) -> None:
        q = np.asarray(self._data.qpos[self._qpos_adrs], dtype=np.float32)
        qd = np.asarray(self._data.qvel[self._dof_adrs], dtype=np.float32)
        if (
            (not np.all(np.isfinite(q)))
            or (not np.all(np.isfinite(qd)))
            or (not np.all(np.isfinite(self._q_des)))
            or (not np.all(np.isfinite(self._qd_des)))
            or (not np.all(np.isfinite(self._kp)))
            or (not np.all(np.isfinite(self._kd)))
            or (not np.all(np.isfinite(self._tau_ff)))
        ):
            self._tau_applied.fill(0.0)
            self._data.qfrc_applied[:] = 0.0
            return

        tau = self._kp * (self._q_des - q) + self._kd * (self._qd_des - qd) + self._tau_ff
        tau -= self._friction_static * np.tanh(qd / self._activation_vel) + self._friction_dynamic * qd
        tau = self._clip_effort_by_tn_curve(tau, qd)
        if not np.all(np.isfinite(tau)):
            self._tau_applied.fill(0.0)
            self._data.qfrc_applied[:] = 0.0
            return

        self._tau_applied[:] = tau
        self._qfrc_buffer.fill(0.0)
        self._qfrc_buffer[self._dof_adrs] = tau
        self._data.qfrc_applied[:] = self._qfrc_buffer
        if self._data.ctrl.size > 0:
            self._data.ctrl[:] = 0.0

    def _clip_effort_by_tn_curve(self, effort: np.ndarray, joint_vel: np.ndarray) -> np.ndarray:
        same_direction = (joint_vel * effort) > 0.0
        max_effort = np.where(same_direction, self._tn_y1, self._tn_y2).astype(np.float32)

        x1 = self._tn_x1
        x2 = self._tn_x2
        valid_curve = np.isfinite(x1) & np.isfinite(x2) & (x2 > x1)
        motoring_over_x1 = same_direction & valid_curve & (np.abs(joint_vel) >= x1)
        curve_limit = np.zeros_like(max_effort)
        curve_limit[valid_curve] = (
            -max_effort[valid_curve] / (x2[valid_curve] - x1[valid_curve])
        ) * (np.abs(joint_vel[valid_curve]) - x1[valid_curve]) + max_effort[valid_curve]
        curve_limit = np.clip(curve_limit, 0.0, None)
        max_effort = np.where(motoring_over_x1, curve_limit, max_effort)

        if self._tau_limits is not None:
            max_effort = np.minimum(max_effort, self._tau_limits)
        return np.clip(effort, -max_effort, max_effort)

    def _dump_actuators(self) -> None:
        mujoco = self._mujoco
        gravity = getattr(self._model.opt, "gravity", None)
        disableflags = int(getattr(self._model.opt, "disableflags", 0))
        gravity_disabled = False
        try:
            gravity_disabled = bool(disableflags & int(mujoco.mjtDisableBit.mjDSBL_GRAVITY))
        except Exception:
            gravity_disabled = False

        print(
            f"[MuJoCo] nu={int(self._model.nu)} nq={int(self._model.nq)} nv={int(self._model.nv)} "
            f"gravity={None if gravity is None else np.array2string(np.asarray(gravity), precision=3)} "
            f"disableflags={disableflags} gravity_disabled={gravity_disabled} torque_control=True "
            f"tau_limits={None if self._tau_limits is None else np.array2string(self._tau_limits, precision=2)} "
            f"tn_x1={np.array2string(self._tn_x1[:3], precision=2)} "
            f"tn_x2={np.array2string(self._tn_x2[:3], precision=2)}"
        )
        for aid in range(int(self._model.nu)):
            name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            trnid = self._model.actuator_trnid[aid]
            joint_id = int(trnid[0])
            joint_name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            kp = float(self._model.actuator_gainprm[aid, 0]) if hasattr(self._model, "actuator_gainprm") else float("nan")
            fr0 = float(self._model.actuator_forcerange[aid, 0]) if hasattr(self._model, "actuator_forcerange") else float("nan")
            fr1 = float(self._model.actuator_forcerange[aid, 1]) if hasattr(self._model, "actuator_forcerange") else float("nan")
            print(
                f"[MuJoCo] actuator[{aid}] name={name} joint={joint_name} "
                f"builtin_kp={kp:.3f} forcerange=({fr0:.1f},{fr1:.1f})"
            )

    def _print_mj_debug(self) -> None:
        act_names = self.actuator_names
        qpos_named = np.zeros(len(act_names), dtype=np.float32)
        for i, (name, qpos_adr) in enumerate(zip(act_names, self._qpos_adrs)):
            qpos_named[i] = float(self._data.qpos[qpos_adr])

        base_pos = np.asarray(self._data.qpos[:3], dtype=np.float32) if self._data.qpos.shape[0] >= 3 else None
        base_vel = np.asarray(self._data.qvel[:3], dtype=np.float32) if self._data.qvel.shape[0] >= 3 else None
        print(
            "[MuJoCo] state "
            f"t={float(self._data.time):.3f} "
            f"base_pos={None if base_pos is None else np.array2string(base_pos, precision=3)} "
            f"base_vel={None if base_vel is None else np.array2string(base_vel, precision=3)} "
            f"q_des[:3]={np.array2string(self._q_des[:3], precision=3)} "
            f"qpos[:3]={np.array2string(qpos_named[:3], precision=3)} "
            f"tau[:3]={np.array2string(self._tau_applied[:3], precision=3)} "
            f"|q_des-qpos|_inf={float(np.max(np.abs(self._q_des - qpos_named))):.4f} "
            f"|tau|_inf={float(np.max(np.abs(self._tau_applied))):.4f}"
        )
