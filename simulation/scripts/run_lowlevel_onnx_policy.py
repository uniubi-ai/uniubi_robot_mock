#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np


DEFAULT_POS_LEG_MAJOR = np.asarray([0.0, 0.8, -1.58] * 4, dtype=np.float32)
LEG_MAJOR_TO_PER_JOINT = np.asarray([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)
PER_JOINT_TO_LEG_MAJOR = np.asarray([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)
DEFAULT_POS_PER_JOINT = DEFAULT_POS_LEG_MAJOR[LEG_MAJOR_TO_PER_JOINT]


def _quat_rotate_inverse_wxyz(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(a) for a in q]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-6:
        return v.astype(np.float32)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    # Inverse rotation by q is rotation by conjugate(q).
    x, y, z = -x, -y, -z
    qv = np.asarray([x, y, z], dtype=np.float32)
    uv = np.cross(qv, v)
    uuv = np.cross(qv, uv)
    return (v + 2.0 * (w * uv + uuv)).astype(np.float32)


def _obs_to_arrays(obs) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    gyro = np.asarray([obs.imu.gyro.x, obs.imu.gyro.y, obs.imu.gyro.z], dtype=np.float32)
    quat = np.asarray(
        [obs.imu.quaternion.w, obs.imu.quaternion.x, obs.imu.quaternion.y, obs.imu.quaternion.z],
        dtype=np.float32,
    )
    pos_leg = np.asarray([m.position for m in obs.motors[:12]], dtype=np.float32)
    vel_leg = np.asarray([m.velocity for m in obs.motors[:12]], dtype=np.float32)
    return gyro, quat, pos_leg, vel_leg


def _build_policy_obs(obs, command: np.ndarray, last_action_model: np.ndarray) -> np.ndarray:
    gyro, quat, pos_leg, vel_leg = _obs_to_arrays(obs)
    gravity_body = _quat_rotate_inverse_wxyz(quat, np.asarray([0.0, 0.0, -1.0], dtype=np.float32))
    pos_model = pos_leg[LEG_MAJOR_TO_PER_JOINT]
    vel_model = vel_leg[LEG_MAJOR_TO_PER_JOINT]
    parts = (
        gyro * 0.2,
        gravity_body,
        command.astype(np.float32),
        pos_model - DEFAULT_POS_PER_JOINT,
        vel_model * 0.05,
        last_action_model.astype(np.float32),
    )
    return np.concatenate(parts, dtype=np.float32).reshape(1, 45)


def _command_from_trc(obs, fallback: np.ndarray) -> np.ndarray:
    trc = getattr(obs, "trc", None)
    if trc is None or not int(getattr(trc, "valid", 0)):
        return fallback.copy()
    axes = list(getattr(trc, "axes", []))
    if len(axes) < 3:
        return fallback.copy()
    # Match mock motionTRC mapping: yaw=axesLX, lineVelocityX=axesLY, lineVelocityY=axesRX.
    return np.asarray([float(axes[1]), float(axes[2]), float(axes[0])], dtype=np.float32)


def _make_action(sdk, layout, target_leg: np.ndarray, kp: float, kd: float):
    action = sdk.MotorCtrlAction()
    motors = []
    for i, mi in enumerate(layout.motors[:12]):
        m = sdk.MotorCtrl()
        m.limb_no = mi.limb_no
        m.joint_no = mi.joint_no
        m.position = float(target_leg[i])
        m.velocity = 0.0
        m.kp_gain = float(kp)
        m.kd_gain = float(kd)
        m.torque = 0.0
        motors.append(m)
    action.motor_num = len(motors)
    action.motors = motors
    return action


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a 45-dim ONNX walking policy through MotionLowLevelClient.")
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--sdk-python",
        default=os.getenv("ROBOTSDK_PYTHON_PATH", ""),
        help="Path containing the robot_motion_sdk Python package. Can also be set by ROBOTSDK_PYTHON_PATH.",
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--control-topic", default="rt/motion/control")
    parser.add_argument("--observed-topic", default="rt/motion/observed")
    parser.add_argument("--trc-topic", default="motion/trc")
    parser.add_argument("--cmd-x", type=float, default=0.3)
    parser.add_argument("--cmd-y", type=float, default=0.0)
    parser.add_argument("--cmd-yaw", type=float, default=0.0)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument(
        "--warmup-duration",
        type=float,
        default=2.0,
        help="Seconds to send the default standing target before running ONNX.",
    )
    parser.add_argument("--warmup-kp", type=float, default=80.0)
    parser.add_argument("--warmup-kd", type=float, default=1.0)
    args = parser.parse_args()

    if not args.sdk_python:
        print("missing --sdk-python or ROBOTSDK_PYTHON_PATH", flush=True)
        return 2
    sdk_python = Path(args.sdk_python).expanduser().resolve()
    if not (sdk_python / "robot_motion_sdk" / "__init__.py").is_file():
        print(f"invalid sdk python path: {sdk_python}", flush=True)
        print("expected: <path>/robot_motion_sdk/__init__.py", flush=True)
        return 2

    os.environ["UNIUBI_MOTION_LOWLEVEL_BACKEND"] = "simulation"
    os.environ["UNIUBI_MOTION_DDS_DOMAIN"] = str(args.domain)
    os.environ["UNIUBI_MOTION_CONTROL_TOPIC"] = args.control_topic
    os.environ["UNIUBI_MOTION_OBSERVED_TOPIC"] = args.observed_topic
    os.environ["UNIUBI_MOTION_TRC_TOPIC"] = args.trc_topic

    sys.path.insert(0, str(sdk_python))
    import robot_motion_sdk as sdk

    fallback_command = np.asarray([args.cmd_x, args.cmd_y, args.cmd_yaw], dtype=np.float32)

    sdk.service.initial(None, "onnxPolicy")
    client = sdk.MotionLowLevelClient()
    if not client.connect(observed_hz=500, lease_ms=60000):
        print(f"connect failed: {client.get_last_error()}", flush=True)
        return 1
    if not client.set_motion_enable(True):
        print(f"set_motion_enable failed: {client.get_last_error()}", flush=True)
        return 1

    layout = client.get_motor_layout()
    warmup_action = _make_action(sdk, layout, DEFAULT_POS_LEG_MAJOR, args.warmup_kp, args.warmup_kd)
    warmup_period = 1.0 / max(args.rate, 1.0)
    next_warmup_t = time.monotonic()
    warmup_count = 0
    session_state = {}

    def _load_onnx_session() -> None:
        try:
            import onnxruntime as ort

            session = ort.InferenceSession(str(Path(args.model)), providers=["CPUExecutionProvider"])
            session_state["session"] = session
            session_state["input_name"] = session.get_inputs()[0].name
            session_state["output_name"] = session.get_outputs()[0].name
        except Exception as exc:
            session_state["error"] = exc

    loader = threading.Thread(target=_load_onnx_session, daemon=True)
    loader.start()
    warmup_deadline = time.monotonic() + max(args.warmup_duration, 0.0)
    while time.monotonic() < warmup_deadline or loader.is_alive():
        client.send_control(warmup_action)
        warmup_count += 1
        next_warmup_t += warmup_period
        sleep_s = next_warmup_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    loader.join()
    if "error" in session_state:
        print(f"load ONNX failed: {session_state['error']}", flush=True)
        client.set_motion_enable(False)
        client.disconnect()
        sdk.service.shutdown()
        return 1
    session = session_state["session"]
    input_name = session_state["input_name"]
    output_name = session_state["output_name"]
    if warmup_count:
        print(
            f"warmup sent default_target count={warmup_count} "
            f"duration={args.warmup_duration}s kp={args.warmup_kp} kd={args.warmup_kd}",
            flush=True,
        )

    last_action_model = np.zeros(12, dtype=np.float32)
    deadline = time.monotonic() + max(args.duration, 0.0)
    period = 1.0 / max(args.rate, 1.0)
    next_t = time.monotonic()
    obs_count = 0
    print(
        f"ONNX policy running model={args.model} obs_topic={args.observed_topic} "
        f"ctrl_topic={args.control_topic} trc_topic={args.trc_topic}",
        flush=True,
    )
    try:
        while time.monotonic() < deadline:
            obs = client.get_latest_observation(timeout_ms=100)
            if obs is None:
                continue
            command = _command_from_trc(obs, fallback_command)
            policy_obs = _build_policy_obs(obs, command, last_action_model)
            action_model = session.run([output_name], {input_name: policy_obs})[0].reshape(12).astype(np.float32)
            action_model = np.clip(action_model, -100.0, 100.0)
            target_model = DEFAULT_POS_PER_JOINT + 0.25 * action_model
            target_leg = target_model[PER_JOINT_TO_LEG_MAJOR]
            action = _make_action(sdk, layout, target_leg, args.kp, args.kd)
            ok = client.send_control(action)
            obs_count += 1
            if obs_count == 1 or obs_count % int(max(args.rate, 1.0)) == 0:
                print(
                    f"step={obs_count} send={ok} cmd={np.round(command, 3).tolist()} "
                    f"act[:3]={np.round(action_model[:3], 3).tolist()} "
                    f"target_leg[:3]={np.round(target_leg[:3], 3).tolist()} "
                    f"trc_valid={getattr(obs.trc, 'valid', 0)}",
                    flush=True,
                )
            last_action_model = action_model
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        client.set_motion_enable(False)
        client.disconnect()
        sdk.service.shutdown()
    print(f"done obs_count={obs_count}", flush=True)
    return 0 if obs_count > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
