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
CROUCH_POS_LEG_MAJOR = np.asarray(
    [
        0.48,
        1.10,
        -2.72,
        -0.48,
        1.10,
        -2.72,
        0.48,
        1.10,
        -2.72,
        -0.48,
        1.10,
        -2.72,
    ],
    dtype=np.float32,
)
POSTURE_KP_LEG_MAJOR = np.asarray(
    [90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 130.0, 130.0, 140.0, 130.0, 130.0, 140.0],
    dtype=np.float32,
)
POSTURE_KD_LEG_MAJOR = np.asarray(
    [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5],
    dtype=np.float32,
)
LEG_MAJOR_TO_PER_JOINT = np.asarray([0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11], dtype=np.int64)
PER_JOINT_TO_LEG_MAJOR = np.asarray([0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11], dtype=np.int64)
DEFAULT_POS_PER_JOINT = DEFAULT_POS_LEG_MAJOR[LEG_MAJOR_TO_PER_JOINT]
POLICY_PATH = Path(__file__).resolve().parents[1] / "models" / "policy.onnx"
CONTROL_RATE_HZ = 50.0


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
    kp_values = np.broadcast_to(np.asarray(kp, dtype=np.float32), (12,))
    kd_values = np.broadcast_to(np.asarray(kd, dtype=np.float32), (12,))
    action = sdk.MotorCtrlAction()
    motors = []
    for i, mi in enumerate(layout.motors[:12]):
        m = sdk.MotorCtrl()
        m.limb_no = mi.limb_no
        m.joint_no = mi.joint_no
        m.position = float(target_leg[i])
        m.velocity = 0.0
        m.kp_gain = float(kp_values[i])
        m.kd_gain = float(kd_values[i])
        m.torque = 0.0
        motors.append(m)
    action.motor_num = len(motors)
    action.motors = motors
    return action


def _latest_joint_pos_leg_major(client, timeout_ms: int, fallback: np.ndarray) -> np.ndarray:
    obs = client.get_latest_observation(timeout_ms=timeout_ms)
    if obs is None or len(getattr(obs, "motors", [])) < 12:
        return fallback.astype(np.float32).copy()
    return np.asarray([m.position for m in obs.motors[:12]], dtype=np.float32)


def _send_pose(client, sdk, layout, pose: np.ndarray, kp: float, kd: float) -> bool:
    return bool(client.send_control(_make_action(sdk, layout, pose.astype(np.float32), kp, kd)))


def _run_pose_transition(
    client,
    sdk,
    layout,
    start_pose: np.ndarray,
    target_pose: np.ndarray,
    duration_s: float,
    rate_hz: float,
    kp: float,
    kd: float,
    name: str,
) -> tuple[np.ndarray, int]:
    duration_s = max(float(duration_s), 0.0)
    period = 1.0 / max(float(rate_hz), 1.0)
    steps = max(1, int(math.ceil(duration_s / period))) if duration_s > 0.0 else 1
    start_pose = start_pose.astype(np.float32).copy()
    target_pose = target_pose.astype(np.float32).copy()
    next_t = time.monotonic()
    sent_count = 0
    for step in range(steps):
        ratio = 1.0 if steps <= 1 else float(step + 1) / float(steps)
        pose = (1.0 - ratio) * start_pose + ratio * target_pose
        _send_pose(client, sdk, layout, pose, kp, kd)
        sent_count += 1
        next_t += period
        sleep_s = next_t - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    print(
        f"{name} transition sent count={sent_count} duration={duration_s:.2f}s "
        f"kp={kp} kd={kd} target[:3]={np.round(target_pose[:3], 3).tolist()}",
        flush=True,
    )
    return target_pose, sent_count


def _wait_lowlevel_state(client, target, timeout_s: float, state_event: threading.Event) -> bool:
    deadline = time.monotonic() + max(timeout_s, 0.0)
    while client.get_state() != target:
        remain = deadline - time.monotonic()
        if remain <= 0:
            return False
        state_event.clear()
        state_event.wait(min(remain, 0.5))
    return True


class PolicyControlLoop:
    def __init__(self, client, sdk, layout, session, rate_hz: float) -> None:
        self.client = client
        self.sdk = sdk
        self.layout = layout
        self.session = session
        self.input_name = session.get_inputs()[0].name
        self.output_name = session.get_outputs()[0].name
        self.period = 1.0 / max(rate_hz, 1.0)
        self.lock = threading.Lock()
        self.mode = "idle"
        self.command = np.zeros(3, dtype=np.float32)
        self.hold_pose = DEFAULT_POS_LEG_MAJOR.copy()
        self.hold_kp = POSTURE_KP_LEG_MAJOR.copy()
        self.hold_kd = POSTURE_KD_LEG_MAJOR.copy()
        self.last_action = np.zeros(12, dtype=np.float32)
        self.running = True
        self.sent = 0
        self.failed = 0
        self.thread = threading.Thread(target=self._run, name="onnx-policy-control", daemon=True)
        self.thread.start()

    def pause(self) -> None:
        with self.lock:
            self.mode = "idle"
        time.sleep(self.period * 2.0)

    def hold(self, pose: np.ndarray, kp=POSTURE_KP_LEG_MAJOR, kd=POSTURE_KD_LEG_MAJOR) -> None:
        with self.lock:
            self.hold_pose = pose.astype(np.float32).copy()
            self.hold_kp = np.broadcast_to(np.asarray(kp, dtype=np.float32), (12,)).copy()
            self.hold_kd = np.broadcast_to(np.asarray(kd, dtype=np.float32), (12,)).copy()
            self.mode = "stand"

    def walk(self, command: np.ndarray) -> None:
        with self.lock:
            self.command = command.astype(np.float32).copy()
            self.last_action.fill(0.0)
            self.mode = "walk"

    def state(self) -> tuple[str, np.ndarray, int, int]:
        with self.lock:
            return self.mode, self.command.copy(), self.sent, self.failed

    def close(self) -> None:
        self.running = False
        self.thread.join(timeout=2.0)

    def _run(self) -> None:
        next_cycle = time.monotonic()
        while self.running:
            with self.lock:
                mode = self.mode
                command = self.command.copy()
                hold_pose = self.hold_pose.copy()
                hold_kp = self.hold_kp.copy()
                hold_kd = self.hold_kd.copy()
                last_action = self.last_action.copy()

            ok = None
            if mode == "stand":
                ok = self.client.send_control(_make_action(self.sdk, self.layout, hold_pose, hold_kp, hold_kd))
            elif mode == "walk":
                obs = self.client.get_latest_observation(timeout_ms=10)
                if obs is not None:
                    policy_obs = _build_policy_obs(obs, command, last_action)
                    action_model = self.session.run([self.output_name], {self.input_name: policy_obs})[0]
                    action_model = np.clip(action_model.reshape(12), -100.0, 100.0).astype(np.float32)
                    target_model = DEFAULT_POS_PER_JOINT + 0.25 * action_model
                    target_leg = target_model[PER_JOINT_TO_LEG_MAJOR]
                    ok = self.client.send_control(_make_action(self.sdk, self.layout, target_leg, 35.0, 1.0))
                    with self.lock:
                        self.last_action = action_model

            if ok is not None:
                with self.lock:
                    if ok:
                        self.sent += 1
                    else:
                        self.failed += 1
            next_cycle += self.period
            delay = next_cycle - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_cycle = time.monotonic()


def _print_help() -> None:
    print(
        """commands:
  stand                 smoothly stand up from the current measured pose
  walk [VX VY YAW]      stand first if needed, then run policy.onnx; defaults to 0.5 0 0
  stop                  stop the policy and return to the standing target
  lay                   smoothly return to the laying pose
  obs                   print the latest observation
  state                 print client and control-loop state
  help                  show this help
  quit                  stop sending, disable LowLevel control, and exit
"""
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Interactive LowLevel demo for the bundled policy.onnx.")
    parser.add_argument(
        "--sdk-python",
        default=os.getenv("ROBOTSDK_PYTHON_PATH", ""),
        help="Path containing the robot_motion_sdk Python package.",
    )
    args = parser.parse_args()

    if not args.sdk_python:
        print("missing --sdk-python or ROBOTSDK_PYTHON_PATH", flush=True)
        return 2
    sdk_python = Path(args.sdk_python).expanduser().resolve()
    sys.path.insert(0, str(sdk_python))
    import robot_motion_sdk as sdk
    import onnxruntime as ort

    os.environ["UNIUBI_MOTION_DDS_DOMAIN"] = "42"
    os.environ["UNIUBI_MOTION_CONTROL_TOPIC"] = "rt/motion/control"
    os.environ["UNIUBI_MOTION_OBSERVED_TOPIC"] = "rt/motion/observed"
    os.environ["UNIUBI_MOTION_TRC_TOPIC"] = "rt/motion/trc"

    session = ort.InferenceSession(str(POLICY_PATH), providers=["CPUExecutionProvider"])
    if session.get_inputs()[0].shape != [1, 45] or session.get_outputs()[0].shape != [1, 12]:
        print("unsupported policy.onnx shape; expected input [1,45] and output [1,12]", flush=True)
        return 2
    if not sdk.service.initial(None, "onnxPolicyCli"):
        print("sdk.service.initial failed", flush=True)
        return 1

    client = sdk.MotionLowLevelClient()
    state_event = threading.Event()

    @client.on_connect
    def _on_connect(state, err):
        print(f"[callback] state={state} error={err}", flush=True)
        state_event.set()

    loop = None
    try:
        if not client.connect(observed_hz=500, lease_ms=60000):
            print(f"connect rejected: {client.get_last_error()}", flush=True)
            return 1
        if not _wait_lowlevel_state(client, sdk.LowLevelState.kConnected, 5.0, state_event):
            print(f"connect timeout: {client.get_last_error()}", flush=True)
            return 1
        layout = client.get_motor_layout()
        if layout is None or layout.motor_num != 12:
            print(f"invalid motor layout: {client.get_last_error()}", flush=True)
            return 1

        loop = PolicyControlLoop(client, sdk, layout, session, CONTROL_RATE_HZ)
        posture = "laying"

        def ensure_prepared() -> None:
            if client.get_state() == sdk.LowLevelState.kConnected:
                if not client.set_motion_enable(True):
                    raise RuntimeError(f"enable rejected: {client.get_last_error()}")
                if not _wait_lowlevel_state(client, sdk.LowLevelState.kPrepared, 10.0, state_event):
                    raise RuntimeError(f"enable timeout: {client.get_last_error()}")
            if client.get_state() != sdk.LowLevelState.kPrepared:
                raise RuntimeError(f"LowLevel control is not prepared: {client.get_state()}")

        def transition_to_stand(name: str = "stand") -> None:
            nonlocal posture
            ensure_prepared()
            loop.pause()
            start = _latest_joint_pos_leg_major(client, 500, CROUCH_POS_LEG_MAJOR)
            _run_pose_transition(
                client, sdk, layout, start, DEFAULT_POS_LEG_MAJOR, 2.0, CONTROL_RATE_HZ,
                POSTURE_KP_LEG_MAJOR, POSTURE_KD_LEG_MAJOR, name,
            )
            loop.hold(DEFAULT_POS_LEG_MAJOR)
            posture = "standing"

        print(f"[PASS] connected; model={POLICY_PATH}; robot starts in laying pose", flush=True)
        _print_help()

        while True:
            try:
                line = input("lowlevel> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            command_name, _, raw_args = line.partition(" ")
            command_name = command_name.lower()
            raw_args = raw_args.strip()
            try:
                if command_name in ("quit", "exit"):
                    break
                if command_name == "help":
                    _print_help()
                elif command_name == "stand":
                    transition_to_stand()
                    print("[PASS] standing", flush=True)
                elif command_name == "walk":
                    values = [float(value) for value in raw_args.split()] if raw_args else [0.5, 0.0, 0.0]
                    if len(values) != 3:
                        raise ValueError("usage: walk [VX VY YAW]")
                    if posture not in ("standing", "walking"):
                        print(f"[INFO] current posture={posture}; standing before walk", flush=True)
                        transition_to_stand("walk prepare")
                    else:
                        ensure_prepared()
                    loop.walk(np.asarray(values, dtype=np.float32))
                    posture = "walking"
                    print(f"[PASS] policy running command={values}", flush=True)
                elif command_name == "stop":
                    if client.get_state() != sdk.LowLevelState.kPrepared:
                        raise RuntimeError("LowLevel control is not enabled")
                    loop.pause()
                    start = _latest_joint_pos_leg_major(client, 500, DEFAULT_POS_LEG_MAJOR)
                    _run_pose_transition(
                        client, sdk, layout, start, DEFAULT_POS_LEG_MAJOR, 1.0, CONTROL_RATE_HZ,
                        POSTURE_KP_LEG_MAJOR, POSTURE_KD_LEG_MAJOR, "stop"
                    )
                    loop.hold(DEFAULT_POS_LEG_MAJOR)
                    posture = "standing"
                    print("[PASS] policy stopped; standing", flush=True)
                elif command_name == "lay":
                    if client.get_state() != sdk.LowLevelState.kPrepared:
                        raise RuntimeError("run stand before lay")
                    loop.pause()
                    start = _latest_joint_pos_leg_major(client, 500, DEFAULT_POS_LEG_MAJOR)
                    _run_pose_transition(
                        client, sdk, layout, start, CROUCH_POS_LEG_MAJOR, 2.0, CONTROL_RATE_HZ,
                        POSTURE_KP_LEG_MAJOR, POSTURE_KD_LEG_MAJOR, "lay"
                    )
                    loop.hold(CROUCH_POS_LEG_MAJOR)
                    posture = "laying"
                    print("[PASS] laying", flush=True)
                elif command_name == "obs":
                    obs = client.get_latest_observation(timeout_ms=500)
                    if obs is None:
                        print("[WAIT] no observation", flush=True)
                    else:
                        q = [round(m.position, 3) for m in obs.motors[:12]]
                        print(f"motors={obs.motor_num} q={q}", flush=True)
                elif command_name == "state":
                    mode, command, sent, failed = loop.state()
                    print(
                        f"client={client.get_state()} error={client.get_last_error()} "
                        f"posture={posture} mode={mode} command={command.tolist()} sent={sent} failed={failed}",
                        flush=True,
                    )
                else:
                    print(f"unknown command: {command_name!r}; type help", flush=True)
            except (RuntimeError, ValueError) as exc:
                print(f"[ERROR] {exc}", flush=True)
        return 0
    finally:
        if loop is not None:
            loop.pause()
            loop.close()
        if client.get_state() == sdk.LowLevelState.kPrepared:
            client.emergency_stop()
            client.set_motion_enable(False)
        client.disconnect()
        sdk.service.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
