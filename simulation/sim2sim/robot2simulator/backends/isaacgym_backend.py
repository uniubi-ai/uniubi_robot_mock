from __future__ import annotations

import contextlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ContextManager, Optional, Sequence

import numpy as np

from sim2sim.robot2simulator.backends.base import SimBackend
from sim2sim.robot2simulator.sim_types import ImuSample, JointState


@dataclass(frozen=True)
class IsaacGymBackendConfig:
    urdf_path: str
    sim_dt: float = 0.005
    publish_hz: float = 200.0
    control_hz: float = 50.0
    realtime: bool = True
    headless: bool = False
    actuator_names: Sequence[str] = ()
    dof_names: Optional[Sequence[str]] = None

    # init
    initial_joint_pos: Optional[np.ndarray] = None  # (n,)
    initial_base_pos_xyz: Optional[np.ndarray] = None  # (3,)
    settling_steps: int = 50

    # pd (Isaac Gym 的 DOF properties)
    stiffness: Optional[np.ndarray] = None  # (n,)
    damping: Optional[np.ndarray] = None  # (n,)
    torque_limits: Optional[np.ndarray] = None  # (n,)

    # sim params
    use_gpu: bool = True
    use_gpu_pipeline: bool = True
    device_id: int = 0
    physics_engine: str = "physx"  # physx | flex
    num_substeps: int = 1
    viewer_width: int = 1280
    viewer_height: int = 720


class IsaacGymBackend(SimBackend):
    """老 Isaac Gym (gymapi) 后端。

    说明：
    - 目标是单机器人、position target 控制，与 MujocoBackend 的语义对齐；
    - joint/DOF 名称默认用 `actuator_names` 去匹配 URDF 的 dof names（不匹配时会报错）。
    """

    def __init__(self, cfg: IsaacGymBackendConfig) -> None:
        try:
            from isaacgym import gymapi, gymtorch  # type: ignore
        except Exception as e:  # pragma: no cover
            raise RuntimeError(
                "未检测到老 Isaac Gym (isaacgym/gymapi) 环境。\n"
                "请在 Isaac Gym 自带的 Python 环境中运行，或确认 PYTHONPATH/LD_LIBRARY_PATH 已配置。"
            ) from e

        self._gymapi = gymapi
        self._gymtorch = gymtorch

        actuator_names = tuple(cfg.actuator_names)
        super().__init__(
            actuator_names=actuator_names,
            sim_dt=cfg.sim_dt,
            realtime=cfg.realtime,
            headless=cfg.headless,
        )
        self._cfg = cfg

        self._gym = gymapi.acquire_gym()
        self._sim = None
        self._env = None
        self._actor = None
        self._viewer = None
        self._actor_index = 0

        self._dof_name_to_index: dict[str, int] = {}
        self._dof_indices_in_act_order: list[int] = []
        self._dof_names_in_act_order: list[str] = []

        self._dof_state_tensor = None
        self._dof_force_tensor = None
        self._root_state_tensor = None
        self._dof_target_tensor = None

        self._act_dof_index_tensor = None
        self._dof_force_cmd_tensor = None
        self._q_des = None
        self._qd_des = None
        self._kp = None
        self._kd = None
        self._tau_ff = None
        self._tau_limits = None
        self._effort_limits_act = None
        self._tau_applied = None
        self._sim_step_count = 0
        self._last_root_lin_vel_w: Optional[np.ndarray] = None
        self._last_root_lin_vel_step: Optional[int] = None
        self._last_quat_wxyz: Optional[np.ndarray] = None
        self._last_kp: Optional[np.ndarray] = None
        self._last_kd: Optional[np.ndarray] = None

        self._create_sim_and_actor()
        self.reset()

    @property
    def publish_hz(self) -> float:
        return float(self._cfg.publish_hz)

    @property
    def control_hz(self) -> float:
        return float(self._cfg.control_hz)

    def _create_sim_and_actor(self) -> None:
        import torch

        gymapi = self._gymapi

        compute_id = int(self._cfg.device_id) if self._cfg.use_gpu else -1
        graphics_id = int(self._cfg.device_id) if (not self._cfg.headless) else -1

        sim_params = gymapi.SimParams()
        sim_params.dt = float(self._cfg.sim_dt)
        sim_params.substeps = int(self._cfg.num_substeps)
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.use_gpu_pipeline = bool(self._cfg.use_gpu_pipeline)
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        physics_engine = str(self._cfg.physics_engine).lower()
        if physics_engine == "physx":
            sim_params.physx.use_gpu = bool(self._cfg.use_gpu)
            # 对齐 legged_gym 默认 PhysX 参数（稳定性更好，减少穿透导致的“卡地里”）
            try:
                sim_params.physx.solver_type = 1  # 1: TGS
            except Exception:
                pass
            try:
                sim_params.physx.num_position_iterations = 12
                sim_params.physx.num_velocity_iterations = 2
            except Exception:
                pass
            try:
                sim_params.physx.contact_offset = 0.01
                sim_params.physx.rest_offset = 0.0
                sim_params.physx.bounce_threshold_velocity = 0.5
                sim_params.physx.max_depenetration_velocity = 1.0
            except Exception:
                pass
            try:
                sim_params.physx.contact_collection = 2
            except Exception:
                pass
            sim = self._gym.create_sim(compute_id, graphics_id, gymapi.SIM_PHYSX, sim_params)
        elif physics_engine == "flex":
            sim = self._gym.create_sim(compute_id, graphics_id, gymapi.SIM_FLEX, sim_params)
        else:
            raise ValueError(f"未知 physics_engine: {self._cfg.physics_engine}")

        if sim is None:
            raise RuntimeError("create_sim 失败：请检查 GPU/驱动/Isaac Gym 环境")

        self._sim = sim

        # ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        try:
            plane_params.static_friction = 1.0
            plane_params.dynamic_friction = 1.0
            plane_params.restitution = 0.0
        except Exception:
            pass
        self._gym.add_ground(self._sim, plane_params)

        # env
        env = self._gym.create_env(self._sim, gymapi.Vec3(-1.0, -1.0, 0.0), gymapi.Vec3(1.0, 1.0, 1.0), 1)
        if env is None:
            raise RuntimeError("create_env 失败")
        self._env = env

        # asset
        asset_options = gymapi.AssetOptions()
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_EFFORT
        asset_options.fix_base_link = False
        asset_options.disable_gravity = False
        asset_options.collapse_fixed_joints = True
        asset_options.replace_cylinder_with_capsule = True

        urdf_path = str(Path(self._cfg.urdf_path).expanduser())
        asset_root = str(Path(urdf_path).parent)
        asset_file = str(Path(urdf_path).name)
        asset = self._gym.load_asset(self._sim, asset_root, asset_file, asset_options)
        if asset is None:
            raise RuntimeError(f"load_asset 失败：{urdf_path}")

        # spawn pose
        base_pos = self._cfg.initial_base_pos_xyz
        if base_pos is None:
            base_pos = np.asarray([0.0, 0.0, 0.8], dtype=np.float32)
        base_pos = np.asarray(base_pos, dtype=np.float32)
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(float(base_pos[0]), float(base_pos[1]), float(base_pos[2]))
        pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

        actor = self._gym.create_actor(self._env, asset, pose, "robot", 0, 1)
        if actor is None:
            raise RuntimeError("create_actor 失败")
        self._actor = actor
        try:
            self._actor_index = int(self._gym.get_actor_index(self._env, self._actor, gymapi.DOMAIN_SIM))
        except Exception:
            self._actor_index = 0

        # dof mapping
        dof_names = self._gym.get_asset_dof_names(asset)
        self._dof_name_to_index = {str(n): int(i) for i, n in enumerate(dof_names)}
        self._dof_names_in_act_order = self._resolve_controlled_dof_names()
        self._dof_indices_in_act_order = [self._dof_name_to_index[n] for n in self._dof_names_in_act_order]

        self._gym.prepare_sim(self._sim)

        # tensors
        self._dof_state_tensor = self._gym.acquire_dof_state_tensor(self._sim)
        self._dof_force_tensor = self._gym.acquire_dof_force_tensor(self._sim)
        self._root_state_tensor = self._gym.acquire_actor_root_state_tensor(self._sim)

        # Isaac Gym 不同版本 API 差异：
        # - 有的版本没有 `acquire_dof_target_tensor`，推荐用户自行分配 targets tensor，然后调用
        #   `gym.set_dof_position_target_tensor(sim, gymtorch.unwrap_tensor(targets))`。
        num_dofs = int(self._gym.get_actor_dof_count(self._env, self._actor))
        target_device = (
            torch.device(f"cuda:{self._cfg.device_id}")
            if (self._cfg.use_gpu_pipeline or self._cfg.use_gpu)
            else torch.device("cpu")
        )
        self._dof_target_tensor = torch.zeros(num_dofs, device=target_device, dtype=torch.float32)

        self._act_dof_index_tensor = torch.tensor(
            self._dof_indices_in_act_order, device=target_device, dtype=torch.long
        )
        self._dof_force_cmd_tensor = torch.zeros(num_dofs, device=target_device, dtype=torch.float32)

        n_act = len(self.actuator_names)
        self._q_des = torch.zeros(n_act, device=target_device, dtype=torch.float32)
        self._qd_des = torch.zeros(n_act, device=target_device, dtype=torch.float32)
        self._kp = torch.zeros(n_act, device=target_device, dtype=torch.float32)
        self._kd = torch.zeros(n_act, device=target_device, dtype=torch.float32)
        self._tau_ff = torch.zeros(n_act, device=target_device, dtype=torch.float32)
        self._tau_applied = torch.zeros(n_act, device=target_device, dtype=torch.float32)
        self._tau_limits = None
        if self._cfg.torque_limits is not None:
            tl = np.asarray(self._cfg.torque_limits, dtype=np.float32)
            if tl.shape[0] != n_act:
                raise RuntimeError(f"torque_limits 维度不匹配：{tl.shape[0]} != {n_act}")
            self._tau_limits = torch.as_tensor(tl, device=target_device, dtype=torch.float32)

    def _reset_root_pose(self) -> None:
        """重置根部位姿，避免初始穿透导致“卡地里”。

        说明：不同 Isaac Gym 版本 API 不一致；
        这里优先使用 root state tensor 接口（legged_gym 同款）。
        """
        import torch

        if self._sim is None or self._root_state_tensor is None:
            return

        base_pos = self._cfg.initial_base_pos_xyz
        if base_pos is None:
            base_pos = np.asarray([0.0, 0.0, 0.8], dtype=np.float32)
        base_pos = np.asarray(base_pos, dtype=np.float32)

        self._gym.refresh_actor_root_state_tensor(self._sim)
        root = self._gymtorch.wrap_tensor(self._root_state_tensor).view(-1, 13)
        idx = int(self._actor_index)
        if idx < 0 or idx >= int(root.shape[0]):
            idx = 0

        root[idx, 0:3] = root.new_tensor([float(base_pos[0]), float(base_pos[1]), float(base_pos[2])])
        root[idx, 3:7] = root.new_tensor([0.0, 0.0, 0.0, 1.0])
        root[idx, 7:13] = 0.0

        if hasattr(self._gym, "set_actor_root_state_tensor_indexed"):
            indices = torch.tensor([idx], device=root.device, dtype=torch.int32)
            self._gym.set_actor_root_state_tensor_indexed(
                self._sim, self._gymtorch.unwrap_tensor(root), self._gymtorch.unwrap_tensor(indices), 1
            )
        elif hasattr(self._gym, "set_actor_root_state_tensor"):
            self._gym.set_actor_root_state_tensor(self._sim, self._gymtorch.unwrap_tensor(root))
        else:
            # 没有可用的 root state setter：只能退化为不重置
            return

    def _resolve_controlled_dof_names(self) -> list[str]:
        """解析“受控 DOF 名称列表”（顺序即 action/motor_pos 的顺序）。"""

        # 1) 显式提供 dof_names（推荐）
        if self._cfg.dof_names is not None:
            names = [str(n) for n in self._cfg.dof_names]
            missing = [n for n in names if n not in self._dof_name_to_index]
            if missing:
                avail = sorted(self._dof_name_to_index.keys())
                raise RuntimeError(f"URDF dof_names 缺失：{missing}；可用 dof_names（节选）: {avail[:30]}")
            if len(names) != len(self.actuator_names):
                raise RuntimeError(
                    f"dof_names 长度需与 actuator_names 一致：{len(names)} != {len(self.actuator_names)}"
                )
            return names

        # 2) actuator_names 直接可用（即与 URDF dof names 一致）
        if all(n in self._dof_name_to_index for n in self.actuator_names):
            return list(self.actuator_names)

        # 3) 兼容 Mujoco 的 actuator 命名：FL_hip/FL_thigh/FL_calf -> *_ABAD/HIP/KNEE_JOINT
        mapped: list[str] = []
        suffix_map = {"hip": "ABAD_JOINT", "thigh": "HIP_JOINT", "calf": "KNEE_JOINT"}
        ok = True
        for n in self.actuator_names:
            parts = str(n).split("_", 1)
            if len(parts) != 2:
                ok = False
                break
            leg, seg = parts[0].upper(), parts[1].lower()
            if seg not in suffix_map:
                ok = False
                break
            cand = f"{leg}_{suffix_map[seg]}"
            if cand not in self._dof_name_to_index:
                ok = False
                break
            mapped.append(cand)
        if ok and len(mapped) == len(self.actuator_names):
            return mapped

        # 4) 常见 Zhishen 12DOF leg order 的默认命名
        if len(self.actuator_names) == 12:
            legs = ["FL", "FR", "RL", "RR"]
            joints = ["ABAD_JOINT", "HIP_JOINT", "KNEE_JOINT"]
            names = [f"{leg}_{j}" for leg in legs for j in joints]
            if all(n in self._dof_name_to_index for n in names):
                return names

        avail = sorted(self._dof_name_to_index.keys())
        raise RuntimeError(
            "无法从 actuator_names 推断 URDF dof 名称顺序。\n"
            "建议在 yaml 中添加 `dof_names: [...]`（顺序与 motor_pos 一致），例如：\n"
            "  [FL_ABAD_JOINT, FL_HIP_JOINT, FL_KNEE_JOINT, FR_ABAD_JOINT, ...]\n"
            f"当前 actuator_names={list(self.actuator_names)}\n"
            f"可用 dof_names（节选）={avail[:30]}"
        )

    def reset(self) -> None:
        if self._sim is None:
            raise RuntimeError("sim 未初始化")

        gymapi = self._gymapi
        self._reset_root_pose()

        init_joint = self._cfg.initial_joint_pos
        if init_joint is None:
            init_joint = np.zeros(len(self.actuator_names), dtype=np.float32)
        init_joint = np.asarray(init_joint, dtype=np.float32)
        if init_joint.shape[0] != len(self.actuator_names):
            raise RuntimeError(
                f"initial_joint_pos 维度不匹配：{init_joint.shape[0]} != {len(self.actuator_names)}"
            )

        # 初始化 dof state
        self._gym.refresh_dof_state_tensor(self._sim)
        dof_state = self._gymtorch.wrap_tensor(self._dof_state_tensor).view(-1, 2)
        for act_i, dof_i in enumerate(self._dof_indices_in_act_order):
            dof_state[dof_i, 0] = float(init_joint[act_i])
            dof_state[dof_i, 1] = 0.0
        self._gym.set_dof_state_tensor(self._sim, self._gymtorch.unwrap_tensor(dof_state))

        # 设置 position control + PD
        dof_props = self._gym.get_actor_dof_properties(self._env, self._actor)
        # 力矩控制：driveMode=EFFORT；kp/kd 由我们自己算力矩，不写入 dof_props
        dof_props["driveMode"][:] = gymapi.DOF_MODE_EFFORT
        try:
            dof_props["stiffness"][:] = 0.0
            dof_props["damping"][:] = 0.0
        except Exception:
            pass

        # effort limit：默认来自 URDF；
        # 若 yaml 提供 torque_limits，则覆盖（对齐 MuJoCo 的 forcerange 语义）
        try:
            if self._cfg.torque_limits is not None:
                tl = np.asarray(self._cfg.torque_limits, dtype=np.float32)
                for act_i, dof_i in enumerate(self._dof_indices_in_act_order):
                    dof_props["effort"][dof_i] = float(tl[act_i])
        except Exception:
            pass
        self._gym.set_actor_dof_properties(self._env, self._actor, dof_props)

        # 记录实际 effort limit（用于每步扭矩裁剪 + 发布 motor_tau）
        try:
            import torch

            effort_act = np.zeros(len(self.actuator_names), dtype=np.float32)
            for act_i, dof_i in enumerate(self._dof_indices_in_act_order):
                effort_act[act_i] = float(dof_props["effort"][dof_i])
            effort_act = np.where(np.isfinite(effort_act) & (effort_act > 0.0), effort_act, np.inf).astype(np.float32)
            self._effort_limits_act = torch.as_tensor(effort_act, device=self._kp.device, dtype=torch.float32)
        except Exception:
            self._effort_limits_act = None

        # 初始化内部 PD（来自 yaml）
        if self._cfg.stiffness is not None:
            kp = np.asarray(self._cfg.stiffness, dtype=np.float32)
            if kp.shape[0] != len(self.actuator_names):
                raise RuntimeError(f"stiffness 维度不匹配：{kp.shape[0]} != {len(self.actuator_names)}")
            self.set_pd_gains(
                kp,
                np.asarray(
                    self._cfg.damping if self._cfg.damping is not None else np.zeros_like(kp),
                    dtype=np.float32,
                ),
            )
        elif self._cfg.damping is not None:
            kd = np.asarray(self._cfg.damping, dtype=np.float32)
            if kd.shape[0] != len(self.actuator_names):
                raise RuntimeError(f"damping 维度不匹配：{kd.shape[0]} != {len(self.actuator_names)}")
            self.set_pd_gains(np.zeros_like(kd), kd)

        # 初始期望
        self.set_position_target(init_joint)
        self.set_velocity_target(np.zeros_like(init_joint))
        self.set_feedforward_torque(np.zeros_like(init_joint))

        # 确保状态同步：先应用一次力矩控制，再进行 settling
        # 这有助于确保 DOF 状态和属性设置后的初始一致性
        self._apply_torque_control()
        self._gym.simulate(self._sim)
        self._gym.fetch_results(self._sim, True)

        for _ in range(int(self._cfg.settling_steps)):
            self.step()

        self._last_kp = None
        self._last_kd = None
        self._sim_step_count = 0
        self._last_root_lin_vel_w = None
        self._last_root_lin_vel_step = None
        self._last_quat_wxyz = None

    def step(self) -> None:
        if self._sim is None:
            raise RuntimeError("sim 未初始化")
        self._apply_torque_control()
        self._gym.simulate(self._sim)
        self._gym.fetch_results(self._sim, True)
        self._sim_step_count += 1

    def set_position_target(self, target_pos: np.ndarray) -> None:
        import torch

        if self._sim is None:
            raise RuntimeError("sim 未初始化")
        target_pos = np.asarray(target_pos, dtype=np.float32).reshape(-1)
        n = len(self.actuator_names)
        if target_pos.shape[0] != n:
            out = np.zeros(n, dtype=np.float32)
            m = min(n, target_pos.shape[0])
            if m > 0:
                out[:m] = target_pos[:m]
            target_pos = out

        t = torch.as_tensor(target_pos, device=self._q_des.device, dtype=self._q_des.dtype)
        self._q_des[:] = t

    def set_velocity_target(self, target_vel: np.ndarray) -> None:
        import torch

        target_vel = np.asarray(target_vel, dtype=np.float32).reshape(-1)
        n = len(self.actuator_names)
        if target_vel.shape[0] != n:
            out = np.zeros(n, dtype=np.float32)
            m = min(n, target_vel.shape[0])
            if m > 0:
                out[:m] = target_vel[:m]
            target_vel = out
        t = torch.as_tensor(target_vel, device=self._qd_des.device, dtype=self._qd_des.dtype)
        self._qd_des[:] = t

    def set_feedforward_torque(self, tau_ff: np.ndarray) -> None:
        import torch

        tau_ff = np.asarray(tau_ff, dtype=np.float32).reshape(-1)
        n = len(self.actuator_names)
        if tau_ff.shape[0] != n:
            out = np.zeros(n, dtype=np.float32)
            m = min(n, tau_ff.shape[0])
            if m > 0:
                out[:m] = tau_ff[:m]
            tau_ff = out
        t = torch.as_tensor(tau_ff, device=self._tau_ff.device, dtype=self._tau_ff.dtype)
        self._tau_ff[:] = t

    def set_pd_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        import torch

        kp = np.asarray(kp, dtype=np.float32)
        kd = np.asarray(kd, dtype=np.float32)
        if (self._last_kp is not None) and (self._last_kd is not None):
            if np.array_equal(kp, self._last_kp) and np.array_equal(kd, self._last_kd):
                return
        if kp.shape[0] != len(self.actuator_names) or kd.shape[0] != len(self.actuator_names):
            raise RuntimeError(
                f"kp/kd 维度不匹配：kp={kp.shape[0]} kd={kd.shape[0]} n={len(self.actuator_names)}"
            )

        self._kp[:] = torch.as_tensor(kp, device=self._kp.device, dtype=self._kp.dtype)
        self._kd[:] = torch.as_tensor(kd, device=self._kd.device, dtype=self._kd.dtype)

        self._last_kp = kp.copy()
        self._last_kd = kd.copy()

    def _apply_torque_control(self) -> None:
        import torch

        if self._sim is None:
            return

        # 读取当前状态
        self._gym.refresh_dof_state_tensor(self._sim)
        dof_state = self._gymtorch.wrap_tensor(self._dof_state_tensor).view(-1, 2)

        dof_q = dof_state[:, 0]
        dof_qd = dof_state[:, 1]
        # 确保索引 tensor 和设备一致（虽然通常已经在同一设备，但为了安全起见）
        act_indices = self._act_dof_index_tensor
        if act_indices.device != dof_q.device:
            act_indices = act_indices.to(dof_q.device)
        q = dof_q[act_indices]
        qd = dof_qd[act_indices]

        # 避免把 NaN/Inf 力矩写进仿真，导致 root quat 爆炸
        if not torch.all(torch.isfinite(q)) or not torch.all(torch.isfinite(qd)):
            self._dof_force_cmd_tensor.zero_()
            self._gym.set_dof_actuation_force_tensor(
                self._sim, self._gymtorch.unwrap_tensor(self._dof_force_cmd_tensor)
            )
            self._tau_applied.zero_()
            return
        if (
            (not torch.all(torch.isfinite(self._q_des)))
            or (not torch.all(torch.isfinite(self._qd_des)))
            or (not torch.all(torch.isfinite(self._kp)))
            or (not torch.all(torch.isfinite(self._kd)))
            or (not torch.all(torch.isfinite(self._tau_ff)))
        ):
            self._dof_force_cmd_tensor.zero_()
            self._gym.set_dof_actuation_force_tensor(
                self._sim, self._gymtorch.unwrap_tensor(self._dof_force_cmd_tensor)
            )
            self._tau_applied.zero_()
            return

        # 确保所有 tensor 在同一设备上（虽然通常已经在同一设备，但为了安全起见）
        target_device = self._kp.device
        if q.device != target_device:
            q = q.to(target_device)
        if qd.device != target_device:
            qd = qd.to(target_device)
        
        tau = self._kp * (self._q_des - q) + self._kd * (self._qd_des - qd) + self._tau_ff

        # 对齐 MuJoCo 的 actuator forcerange：每步裁剪，
        # 避免内部饱和导致“发布值”和“实际施加”不一致
        limit = None
        if self._tau_limits is not None:
            limit = self._tau_limits
        if self._effort_limits_act is not None:
            limit = self._effort_limits_act if limit is None else torch.minimum(limit, self._effort_limits_act)
        if limit is not None:
            tau = torch.clamp(tau, -limit, limit)
        if not torch.all(torch.isfinite(tau)):
            self._dof_force_cmd_tensor.zero_()
            self._gym.set_dof_actuation_force_tensor(
                self._sim, self._gymtorch.unwrap_tensor(self._dof_force_cmd_tensor)
            )
            self._tau_applied.zero_()
            return
        self._tau_applied[:] = tau

        self._dof_force_cmd_tensor.zero_()
        # 确保索引 tensor 和设备一致
        act_indices = self._act_dof_index_tensor
        if act_indices.device != self._dof_force_cmd_tensor.device:
            act_indices = act_indices.to(self._dof_force_cmd_tensor.device)
        # 确保 tau 和设备一致
        if tau.device != self._dof_force_cmd_tensor.device:
            tau = tau.to(self._dof_force_cmd_tensor.device)
        self._dof_force_cmd_tensor[act_indices] = tau
        self._gym.set_dof_actuation_force_tensor(self._sim, self._gymtorch.unwrap_tensor(self._dof_force_cmd_tensor))

    def get_imu(self) -> ImuSample:
        if self._sim is None:
            raise RuntimeError("sim 未初始化")
        self._gym.refresh_actor_root_state_tensor(self._sim)
        root = self._gymtorch.wrap_tensor(self._root_state_tensor).view(-1, 13)[int(self._actor_index)]

        # root: [px,py,pz, qx,qy,qz,qw, vx,vy,vz, wx,wy,wz]
        qx, qy, qz, qw = [float(v) for v in root[3:7]]
        quat_wxyz = np.asarray([qw, qx, qy, qz], dtype=np.float32)
        if not np.all(np.isfinite(quat_wxyz)):
            if self._last_quat_wxyz is not None:
                quat_wxyz = self._last_quat_wxyz.copy()
            else:
                quat_wxyz = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        else:
            n = float(np.linalg.norm(quat_wxyz))
            if (not np.isfinite(n)) or n < 1e-6:
                quat_wxyz = np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            else:
                quat_wxyz = (quat_wxyz / n).astype(np.float32)
        self._last_quat_wxyz = quat_wxyz.copy()

        # 对齐 legged_gym / MuJoCo IMU sensor 的语义：输出机体系 gyro/accel
        # accel 为 specific force，静止约为 +9.81z
        omega_w = np.asarray([float(root[10]), float(root[11]), float(root[12])], dtype=np.float32)
        v_w = np.asarray([float(root[7]), float(root[8]), float(root[9])], dtype=np.float32)

        R_bw = self._rotmat_from_quat_wxyz(quat_wxyz)  # body->world
        gyro_xyz = (R_bw.T @ omega_w).astype(np.float32)

        accel_xyz = np.zeros(3, dtype=np.float32)
        if self._last_root_lin_vel_w is not None and self._last_root_lin_vel_step is not None:
            steps = max(1, int(self._sim_step_count - self._last_root_lin_vel_step))
            dt = float(self.sim_dt) * float(steps)
            a_w = (v_w - self._last_root_lin_vel_w) / max(dt, 1e-12)
            g_w = np.asarray([0.0, 0.0, -9.81], dtype=np.float32)
            accel_xyz = (R_bw.T @ (a_w - g_w)).astype(np.float32)
        self._last_root_lin_vel_w = v_w
        self._last_root_lin_vel_step = int(self._sim_step_count)

        return ImuSample(quat_wxyz=quat_wxyz, gyro_xyz=gyro_xyz, accel_xyz=accel_xyz)

    @staticmethod
    def _rotmat_from_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
        """由四元数(wxyz)生成旋转矩阵（body->world）。"""
        qw, qx, qy, qz = [float(x) for x in quat_wxyz]
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz
        return np.asarray(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )

    def get_joint_state(self) -> JointState:
        if self._sim is None:
            raise RuntimeError("sim 未初始化")
        if self._tau_applied is None:
            raise RuntimeError("_tau_applied 未初始化")
        self._gym.refresh_dof_state_tensor(self._sim)

        dof_state = self._gymtorch.wrap_tensor(self._dof_state_tensor).view(-1, 2)

        qpos = np.zeros(len(self.actuator_names), dtype=np.float32)
        qvel = np.zeros(len(self.actuator_names), dtype=np.float32)
        tau = np.zeros(len(self.actuator_names), dtype=np.float32)
        for act_i, dof_i in enumerate(self._dof_indices_in_act_order):
            qpos[act_i] = float(dof_state[dof_i, 0])
            qvel[act_i] = float(dof_state[dof_i, 1])
            tau[act_i] = float(self._tau_applied[act_i])
        return JointState(qpos=qpos, qvel=qvel, torque=tau)

    def viewer_context(self, enabled: bool) -> ContextManager[Optional[Any]]:
        if (not enabled) or self.headless:
            return contextlib.nullcontext(None)

        if self._sim is None:
            raise RuntimeError("sim 未初始化")

        gymapi = self._gymapi
        cam_props = gymapi.CameraProperties()
        try:
            cam_props.width = int(self._cfg.viewer_width)
            cam_props.height = int(self._cfg.viewer_height)
        except Exception:
            pass
        viewer = self._gym.create_viewer(self._sim, cam_props)
        if viewer is None:
            raise RuntimeError("create_viewer 失败")
        self._viewer = viewer

        @contextlib.contextmanager
        def _ctx():
            try:
                yield viewer
            finally:
                try:
                    self._gym.destroy_viewer(viewer)
                except Exception:
                    pass
                self._viewer = None

        return _ctx()

    def viewer_is_running(self, viewer: Any) -> bool:
        return not bool(self._gym.query_viewer_has_closed(viewer))

    def viewer_sync(self, viewer: Any) -> None:
        if self._sim is None:
            return
        self._gym.step_graphics(self._sim)
        self._gym.draw_viewer(viewer, self._sim, True)
        if self.realtime:
            self._gym.sync_frame_time(self._sim)

    def close(self) -> None:
        if self._sim is None:
            return
        try:
            if self._viewer is not None:
                self._gym.destroy_viewer(self._viewer)
        except Exception:
            pass
        self._viewer = None

        try:
            self._gym.destroy_sim(self._sim)
        except Exception:
            pass
        self._sim = None
