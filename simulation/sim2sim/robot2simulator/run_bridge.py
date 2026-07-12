from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Any, Dict

import numpy as np
import yaml

from sim2sim.robot2simulator.bridge_core import BridgeCore, BridgeOptions
from sim2sim.robot2simulator.config import DEFAULT_ACTUATOR_NAMES, DdsConfig
from sim2sim.robot2simulator.joint_map import headers_for_actuator_names
from sim2sim.robot2simulator.transport.cyclonedds_transport import CycloneDdsConfig, CycloneDdsTransport


def _load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    return {} if raw is None else raw


def _resolve_path(path: str, *, config_dir: Path) -> str:
    expanded = Path(path).expanduser()
    if expanded.is_absolute():
        return str(expanded)
    return str((config_dir / expanded).resolve())


def _get_sim_dt(raw: Dict[str, Any]) -> float:
    return float(raw.get("simulation_dt", 0.005))


def _get_publish_hz(raw: Dict[str, Any], sim_dt: float) -> float:
    if "publish_hz" in raw and raw["publish_hz"] is not None:
        return float(raw["publish_hz"])
    return 1.0 / max(float(sim_dt), 1e-12)


def _get_control_hz(raw: Dict[str, Any], sim_dt: float) -> float:
    if "control_hz" in raw and raw["control_hz"] is not None:
        return float(raw["control_hz"])
    if "control_decimation" in raw and raw["control_decimation"] is not None:
        sim_hz = 1.0 / max(float(sim_dt), 1e-12)
        return float(sim_hz) / max(float(raw["control_decimation"]), 1.0)
    return 50.0


def _select_backend(raw: Dict[str, Any], arg_backend: str) -> str:
    if arg_backend != "auto":
        return arg_backend
    key = raw.get("backend", raw.get("simulator", None))
    if key is not None:
        return str(key).strip().lower()
    if "xml_path" in raw:
        return "mujoco"
    if "urdf_path" in raw:
        return "isaacgym"
    raise RuntimeError("无法从 config 推断 backend：请在 yaml 里加 `backend: mujoco|isaacgym`")


def _make_transport(dds: DdsConfig, *, actuator_names: tuple[str, ...], dds_debug: bool) -> CycloneDdsTransport:
    headers = tuple(headers_for_actuator_names(actuator_names))

    qos_obs = dds.qos_motion_observed if dds.qos_motion_observed is not None else dds.qos
    qos_ctrl = dds.qos_motion_control if dds.qos_motion_control is not None else dds.qos
    qos_fault = dds.qos_motion_fault if dds.qos_motion_fault is not None else dds.qos
    cfg = CycloneDdsConfig(
        domain_id=dds.domain_id,
        topic_motion_observed=dds.topic_motion_observed,
        topic_motion_control=dds.topic_motion_control,
        topic_motion_fault=dds.topic_motion_fault,
        topic_motion_record=dds.topic_motion_record,
        observed_reliability=qos_obs.reliability,
        observed_history=qos_obs.history,
        observed_depth=int(qos_obs.depth),
        observed_reliable_max_blocking_time_ns=int(qos_obs.reliable_max_blocking_time_ns),
        observed_data_representation_cdrv0=bool(qos_obs.data_representation_cdrv0),
        observed_data_representation_xcdrv2=bool(qos_obs.data_representation_xcdrv2),
        control_reliability=qos_ctrl.reliability,
        control_history=qos_ctrl.history,
        control_depth=int(qos_ctrl.depth),
        control_reliable_max_blocking_time_ns=int(qos_ctrl.reliable_max_blocking_time_ns),
        control_data_representation_cdrv0=bool(qos_ctrl.data_representation_cdrv0),
        control_data_representation_xcdrv2=bool(qos_ctrl.data_representation_xcdrv2),
        fault_reliability=qos_fault.reliability,
        fault_history=qos_fault.history,
        fault_depth=int(qos_fault.depth),
        fault_reliable_max_blocking_time_ns=int(qos_fault.reliable_max_blocking_time_ns),
        fault_data_representation_cdrv0=bool(qos_fault.data_representation_cdrv0),
        fault_data_representation_xcdrv2=bool(qos_fault.data_representation_xcdrv2),
        debug=bool(dds_debug),
        debug_prefix="DDS",
    )
    return CycloneDdsTransport(cfg, motor_headers=headers)


def _make_backend_mujoco(
    raw: Dict[str, Any],
    *,
    args: argparse.Namespace,
    actuator_names: tuple[str, ...],
    config_dir: Path,
):
    from sim2sim.robot2simulator.backends.mujoco_backend import MujocoBackend, MujocoBackendConfig

    xml_path = _resolve_path(str(raw["xml_path"]), config_dir=config_dir)
    sim_dt = _get_sim_dt(raw)
    publish_hz = _get_publish_hz(raw, sim_dt=sim_dt)
    control_hz = _get_control_hz(raw, sim_dt=sim_dt)

    control_cfg = raw.get("control", {}) or {}
    stiffness = np.asarray(control_cfg.get("stiffness", []), dtype=np.float32)
    damping = np.asarray(control_cfg.get("damping", []), dtype=np.float32)
    torque_limits_arr = np.asarray(raw.get("torque_limits", []), dtype=np.float32)
    torque_limits = None if torque_limits_arr.size == 0 else torque_limits_arr
    actuator_model_cfg = raw.get("actuator_model", {}) or {}

    def _optional_actuator_param(name: str):
        if name not in actuator_model_cfg:
            return None
        return np.asarray(actuator_model_cfg[name], dtype=np.float32)

    init_joint_pos = raw.get("initial_joint_pos", None)
    if init_joint_pos is None:
        init_joint_pos = raw.get("init_angles", None)
    init_joint_pos = None if init_joint_pos is None else np.asarray(init_joint_pos, dtype=np.float32)

    base_pos = raw.get("initial_base_pos_xyz", None)
    base_pos = None if base_pos is None else np.asarray(base_pos, dtype=np.float32)

    settling_steps = int(raw.get("settling_steps", 50))

    cfg = MujocoBackendConfig(
        xml_path=str(Path(xml_path).expanduser()),
        sim_dt=sim_dt,
        publish_hz=publish_hz,
        control_hz=control_hz,
        realtime=bool(raw.get("realtime", True)),
        headless=bool(args.headless or raw.get("headless", False)),
        actuator_names=actuator_names,
        initial_joint_pos=init_joint_pos,
        initial_base_pos_xyz=base_pos,
        settling_steps=settling_steps,
        stiffness=stiffness,
        damping=damping,
        torque_limits=torque_limits,
        armature=_optional_actuator_param("armature"),
        torque_curve_x1=_optional_actuator_param("x1"),
        torque_curve_x2=_optional_actuator_param("x2"),
        torque_curve_y1=_optional_actuator_param("y1"),
        torque_curve_y2=_optional_actuator_param("y2"),
        friction_static=_optional_actuator_param("friction_static"),
        friction_dynamic=_optional_actuator_param("friction_dynamic"),
        activation_velocity=_optional_actuator_param("activation_velocity"),
        dump_actuators=bool(args.dump_actuators),
        mj_debug=bool(args.mj_debug),
        mj_debug_hz=float(args.mj_debug_hz),
    )
    return MujocoBackend(cfg)


def _make_backend_isaacgym(
    raw: Dict[str, Any],
    *,
    args: argparse.Namespace,
    actuator_names: tuple[str, ...],
    config_dir: Path,
):
    from sim2sim.robot2simulator.backends.isaacgym_backend import IsaacGymBackend, IsaacGymBackendConfig

    urdf_path = _resolve_path(str(raw["urdf_path"]), config_dir=config_dir)
    sim_dt = _get_sim_dt(raw)
    publish_hz = _get_publish_hz(raw, sim_dt=sim_dt)
    control_hz = _get_control_hz(raw, sim_dt=sim_dt)

    control_cfg = raw.get("control", {}) or {}
    stiffness_arr = np.asarray(control_cfg.get("stiffness", []), dtype=np.float32)
    damping_arr = np.asarray(control_cfg.get("damping", []), dtype=np.float32)
    stiffness = None if stiffness_arr.size == 0 else stiffness_arr
    damping = None if damping_arr.size == 0 else damping_arr
    torque_limits_arr = np.asarray(raw.get("torque_limits", []), dtype=np.float32)
    torque_limits = None if torque_limits_arr.size == 0 else torque_limits_arr

    init_joint_pos = raw.get("initial_joint_pos", None)
    if init_joint_pos is None:
        init_joint_pos = raw.get("init_angles", None)
    init_joint_pos = None if init_joint_pos is None else np.asarray(init_joint_pos, dtype=np.float32)

    base_pos = raw.get("initial_base_pos_xyz", None)
    base_pos = None if base_pos is None else np.asarray(base_pos, dtype=np.float32)

    gym_cfg = raw.get("isaacgym", {}) or {}
    dof_names = raw.get("dof_names", None)
    if dof_names is None:
        dof_names = gym_cfg.get("dof_names", None)
    settling_steps = raw.get("settling_steps", None)
    if settling_steps is None:
        settling_steps = gym_cfg.get("settling_steps", 50)

    cfg = IsaacGymBackendConfig(
        urdf_path=str(Path(urdf_path).expanduser()),
        sim_dt=sim_dt,
        publish_hz=publish_hz,
        control_hz=control_hz,
        realtime=bool(raw.get("realtime", True)),
        headless=bool(args.headless or raw.get("headless", False)),
        actuator_names=actuator_names,
        dof_names=None if dof_names is None else tuple(str(x) for x in dof_names),
        initial_joint_pos=init_joint_pos,
        initial_base_pos_xyz=base_pos,
        settling_steps=int(settling_steps),
        stiffness=stiffness,
        damping=damping,
        torque_limits=torque_limits,
        use_gpu=bool(gym_cfg.get("use_gpu", True)),
        use_gpu_pipeline=bool(gym_cfg.get("use_gpu_pipeline", True)),
        device_id=int(gym_cfg.get("device_id", 0)),
        physics_engine=str(gym_cfg.get("physics_engine", "physx")),
        num_substeps=int(gym_cfg.get("num_substeps", 2)),
        viewer_width=int(gym_cfg.get("viewer_width", 1280)),
        viewer_height=int(gym_cfg.get("viewer_height", 720)),
    )
    return IsaacGymBackend(cfg)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="yaml path, for example sim2sim/configs/uniubi_cyvet.yaml")
    parser.add_argument("--backend", choices=("auto", "mujoco", "isaacgym"), default="auto")
    parser.add_argument("--viewer", action="store_true", help="打开仿真器 viewer（若后端支持）")
    parser.add_argument("--headless", action="store_true", help="强制 headless")
    parser.add_argument(
        "--viewer-sync-every-n-steps",
        type=int,
        default=2,
        help="viewer.sync() 的步长；1 表示每步同步，2 表示每 2 步同步一次",
    )

    parser.add_argument("--print-ctrl", action="store_true", help="打印接收到的控制（action）")
    parser.add_argument("--print-ctrl-hz", type=float, default=2.0, help="控制打印频率（Hz）")
    parser.add_argument("--timing-log", action="store_true", help="将高频 timing 明细写入 timing.log")
    parser.add_argument("--dds-debug", action="store_true", help="打印 DDS 匹配/类型名信息（用于确认是否与对端匹配）")
    parser.add_argument("--dds-debug-hz", type=float, default=1.0, help="DDS 匹配状态打印频率（Hz）")
    parser.add_argument(
        "--drop-ctrl-after-first-s",
        type=float,
        default=0.0,
        help="首次成功接收 action 后，额外丢弃后续控制的持续时间（秒）；0 表示关闭",
    )
    parser.add_argument(
        "--fault-joint-index",
        type=int,
        default=-1,
        help="发布 observed 时注入故障的关节 index；-1 表示关闭",
    )
    parser.add_argument(
        "--fault-start-s",
        type=float,
        default=0.0,
        help="单关节故障开始时间（相对首次成功接收 action 的秒数）",
    )
    parser.add_argument(
        "--fault-duration-s",
        type=float,
        default=0.0,
        help="单关节故障持续时间（秒）；0 表示不注入",
    )
    parser.add_argument(
        "--fault-error-code",
        type=int,
        default=1,
        help="单关节故障期间发布到 observed.motor[i].error 的错误码",
    )

    # mujoco debug
    parser.add_argument("--mj-debug", action="store_true", help="打印 MuJoCo 侧状态，用于排查为何不动")
    parser.add_argument("--mj-debug-hz", type=float, default=2.0, help="MuJoCo 调试打印频率（Hz）")
    parser.add_argument("--dump-actuators", action="store_true", help="启动时打印执行器参数（kp/forcerange/映射关节）")
    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()
    raw = _load_yaml(str(config_path))

    dds = DdsConfig()
    print(f"DDS Config: {dds}")

    backend_name = _select_backend(raw, args.backend)
    actuator_names = tuple(raw.get("actuator_names", DEFAULT_ACTUATOR_NAMES))

    if backend_name == "mujoco":
        backend = _make_backend_mujoco(raw, args=args, actuator_names=actuator_names, config_dir=config_path.parent)
        publish_hz = float(getattr(backend, "publish_hz"))
        control_hz = float(getattr(backend, "control_hz"))
    elif backend_name == "isaacgym":
        backend = _make_backend_isaacgym(raw, args=args, actuator_names=actuator_names, config_dir=config_path.parent)
        publish_hz = float(getattr(backend, "publish_hz"))
        control_hz = float(getattr(backend, "control_hz"))
    else:
        raise RuntimeError(f"未知 backend: {backend_name}")

    transport = _make_transport(dds, actuator_names=actuator_names, dds_debug=bool(args.dds_debug))

    core = BridgeCore(
        transport=transport,
        backend=backend,
        publish_hz=publish_hz,
        control_hz=control_hz,
        options=BridgeOptions(
            print_ctrl=bool(args.print_ctrl),
            print_ctrl_hz=float(args.print_ctrl_hz),
            dds_debug=bool(args.dds_debug),
            dds_debug_hz=float(args.dds_debug_hz),
            timing_log=bool(args.timing_log),
            drop_ctrl_after_first_s=float(args.drop_ctrl_after_first_s),
            fault_joint_index=int(args.fault_joint_index),
            fault_start_s=float(args.fault_start_s),
            fault_duration_s=float(args.fault_duration_s),
            fault_error_code=int(args.fault_error_code),
        ),
    )
    core.print_startup(
        domain_id=dds.domain_id,
        obs_topic=dds.topic_motion_observed,
        ctrl_topic=dds.topic_motion_control,
        fault_topic=dds.topic_motion_fault,
    )

    viewer_enabled = bool(args.viewer) and (not backend.headless) and (not bool(args.headless))
    viewer_sync_every_n_steps = max(1, int(args.viewer_sync_every_n_steps))
    try:
        with backend.viewer_context(viewer_enabled) as viewer:
            while True:
                if viewer is not None and (not backend.viewer_is_running(viewer)):
                    break
                loop_wall_start = time.time()
                loop_perf_start = time.perf_counter()
                is_control_boundary = core.will_hit_control_boundary()
                core.step_once()
                viewer_sync_ms = 0.0
                if viewer is not None and (core.step_count % viewer_sync_every_n_steps == 0):
                    viewer_sync_start = time.perf_counter()
                    backend.viewer_sync(viewer)
                    viewer_sync_ms = (time.perf_counter() - viewer_sync_start) * 1000.0
                sleep_ms = 0.0
                if core.realtime:
                    sleep_s = float(core.sim_dt) - (time.time() - loop_wall_start)
                    if sleep_s > 0:
                        sleep_start = time.perf_counter()
                        time.sleep(sleep_s)
                        sleep_ms = (time.perf_counter() - sleep_start) * 1000.0
                if is_control_boundary:
                    core.log_loop_timing(
                        viewer_sync_ms=viewer_sync_ms,
                        sleep_ms=sleep_ms,
                        loop_ms=(time.perf_counter() - loop_perf_start) * 1000.0,
                    )
    finally:
        core.close()
        backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
