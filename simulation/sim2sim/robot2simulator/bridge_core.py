from __future__ import annotations

import atexit
import os
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from sim2sim.robot2simulator.backends.base import SimBackend
from sim2sim.robot2simulator.joint_map import headers_for_actuator_names
from sim2sim.robot2simulator.motion_messages import MotionCtrl, MotionFault, MotionFaultItem, MotionObserved
from sim2sim.robot2simulator.transport.base import MotionTransport


@dataclass(frozen=True)
class BridgeOptions:
    print_ctrl: bool
    print_ctrl_hz: float
    dds_debug: bool
    dds_debug_hz: float
    timing_log: bool
    drop_ctrl_after_first_s: float
    fault_joint_index: int
    fault_start_s: float
    fault_duration_s: float
    fault_error_code: int


@dataclass(frozen=True)
class _ScheduledFault:
    token: int
    joint_index: int
    start_at: float
    end_at: float
    error_code: int
    source: str
    start_label_s: float
    end_label_s: float


@dataclass(frozen=True)
class _ActiveFault:
    token: int
    joint_index: int
    error_code: int
    source: str
    start_label_s: float
    end_label_s: float


class _AsyncLineWriter:
    def __init__(self, path: str) -> None:
        self._path = str(path)
        parent = os.path.dirname(self._path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._fp = open(self._path, "a", encoding="utf-8", buffering=1)
        self._queue: queue.Queue[str | None] = queue.Queue()
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="bridge-timing-log", daemon=True)
        self._thread.start()
        atexit.register(self.close)

    @property
    def path(self) -> str:
        return self._path

    def write(self, line: str) -> None:
        if self._closed:
            return
        self._queue.put(line.rstrip("\n") + "\n")

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                break
            self._fp.write(item)
        self._fp.flush()
        self._fp.close()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._queue.put(None)
        self._thread.join(timeout=2.0)


class BridgeCore:
    def __init__(
        self,
        *,
        transport: MotionTransport,
        backend: SimBackend,
        publish_hz: float,
        control_hz: float,
        options: BridgeOptions,
    ) -> None:
        self._transport = transport
        self._backend = backend
        self._options = options

        sim_hz = 1.0 / max(float(self._backend.sim_dt), 1e-12)
        self._publish_decimation = max(1, int(round(sim_hz / float(publish_hz))))
        self._control_decimation = max(1, int(round(sim_hz / float(control_hz))))
        self._control_period_s = float(self._backend.sim_dt) * float(self._control_decimation)

        self._step_count = 0
        self._next_ctrl_print_t = 0.0
        self._first_ctrl_recv_t: Optional[float] = None
        self._last_ctrl_recv_t: Optional[float] = None
        self._last_ctrl_msg_ts_us: Optional[int] = None
        self._wait_print_decimation = max(1, int(round(sim_hz)))
        self._next_dds_debug_t = 0.0
        self._last_ctrl: Optional[MotionCtrl] = None
        self._next_obs_print_t = 0.0
        self._observed_pub_count = 0
        self._next_ctrl_none_log_t = 0.0
        self._ctrl_drop_window_started = False
        self._ctrl_drop_until_t: Optional[float] = None

        self._headers = tuple(headers_for_actuator_names(self._backend.actuator_names))
        motor_num = len(self._backend.actuator_names)
        self._pub_quat = np.empty(4, dtype=np.float32)
        self._pub_gyro = np.empty(3, dtype=np.float32)
        self._pub_accel = np.empty(3, dtype=np.float32)
        self._pub_motor_pos = np.empty(motor_num, dtype=np.float32)
        self._pub_motor_vel = np.empty(motor_num, dtype=np.float32)
        self._pub_motor_tau = np.empty(motor_num, dtype=np.float32)
        self._last_good_quat = np.empty(4, dtype=np.float32)
        self._last_good_gyro = np.empty(3, dtype=np.float32)
        self._last_good_accel = np.empty(3, dtype=np.float32)
        self._last_good_motor_pos = np.empty(motor_num, dtype=np.float32)
        self._last_good_motor_vel = np.empty(motor_num, dtype=np.float32)
        self._last_good_motor_tau = np.empty(motor_num, dtype=np.float32)
        self._last_good_valid = False
        self._next_bad_obs_log_t = 0.0
        self._next_bad_ctrl_shape_log_t = 0.0
        timing_log_path = os.environ.get("BRIDGE_TIMING_LOG_PATH", "timing.log")
        self._timing_writer: Optional[_AsyncLineWriter] = _AsyncLineWriter(timing_log_path) if options.timing_log else None
        self._fault_active = False
        self._fault_hold_ctrl: Optional[MotionCtrl] = None
        self._fault_visual_indices: tuple[int, ...] = ()
        self._scheduled_faults: dict[int, _ScheduledFault] = {}
        self._active_faults: dict[int, _ActiveFault] = {}
        self._next_fault_token = 1

    def _legacy_fault(self, now: float) -> Optional[_ActiveFault]:
        fault_idx = int(self._options.fault_joint_index)
        start_s = max(float(self._options.fault_start_s), 0.0)
        duration_s = max(float(self._options.fault_duration_s), 0.0)
        if self._first_ctrl_recv_t is None:
            return None
        elapsed_s = max(float(now - self._first_ctrl_recv_t), 0.0)
        if not (
            (0 <= fault_idx < len(self._backend.actuator_names))
            and (duration_s > 0.0)
            and (start_s <= elapsed_s < (start_s + duration_s))
        ):
            return None
        return _ActiveFault(
            token=0,
            joint_index=fault_idx,
            error_code=int(self._options.fault_error_code) & 0xFF,
            source="cli",
            start_label_s=start_s,
            end_label_s=start_s + duration_s,
        )

    def _accept_fault_command(self, item: MotionFaultItem, now: float) -> None:
        joint_index = int(item.joint_index)
        if not (0 <= joint_index < len(self._backend.actuator_names)):
            if self._options.print_ctrl:
                print(f"[Bridge] WARN ignore fault command: invalid joint_index={joint_index}")
            return

        duration_s = max(float(item.duration_s), 0.0)
        if duration_s <= 0.0:
            if self._options.print_ctrl:
                print(
                    f"[Bridge] WARN ignore fault command: joint={self._backend.actuator_names[joint_index]} "
                    f"index={joint_index} duration_s={duration_s:.3f}"
                )
            return

        start_delay_s = max(float(item.start_delay_s), 0.0)
        token = self._next_fault_token
        self._next_fault_token += 1
        self._scheduled_faults[token] = _ScheduledFault(
            token=token,
            joint_index=joint_index,
            start_at=now + start_delay_s,
            end_at=now + start_delay_s + duration_s,
            error_code=int(item.error_code) & 0xFF,
            source="topic",
            start_label_s=start_delay_s,
            end_label_s=start_delay_s + duration_s,
        )
        if self._options.print_ctrl:
            print(
                "[Bridge] schedule joint fault "
                f"joint={self._backend.actuator_names[joint_index]} index={joint_index} "
                f"offline=1 error={int(item.error_code) & 0xFF} "
                f"delay_s={start_delay_s:.3f} duration_s={duration_s:.3f}"
            )

    def _poll_fault_commands(self, now: float) -> None:
        for cmd in self._transport.try_recv_faults(timeout_s=0.0):
            for item in cmd.items:
                self._accept_fault_command(item, now)

    def _collect_active_faults(self, now: float) -> dict[int, _ActiveFault]:
        active: dict[int, _ActiveFault] = {}

        legacy = self._legacy_fault(now)
        if legacy is not None:
            active[legacy.joint_index] = legacy

        expired_tokens = [token for token, fault in self._scheduled_faults.items() if now >= fault.end_at]
        for token in expired_tokens:
            self._scheduled_faults.pop(token, None)

        for fault in self._scheduled_faults.values():
            if now < fault.start_at:
                continue
            current = active.get(fault.joint_index)
            if current is None or fault.token >= current.token:
                active[fault.joint_index] = _ActiveFault(
                    token=fault.token,
                    joint_index=fault.joint_index,
                    error_code=fault.error_code,
                    source=fault.source,
                    start_label_s=fault.start_label_s,
                    end_label_s=fault.end_label_s,
                )
        return active

    def _sync_fault_state(self, now: float) -> dict[int, _ActiveFault]:
        next_faults = self._collect_active_faults(now)
        prev_faults = self._active_faults

        prev_joints = set(prev_faults.keys())
        next_joints = set(next_faults.keys())

        if (not prev_joints) and next_joints:
            self._fault_hold_ctrl = self._last_ctrl
            if self._options.print_ctrl and self._fault_hold_ctrl is None:
                print("[Bridge] WARN joint fault entered before any action was accepted; no control to hold")
        elif prev_joints and (not next_joints):
            self._fault_hold_ctrl = None

        if self._options.print_ctrl:
            for joint_index in sorted(next_joints - prev_joints):
                fault = next_faults[joint_index]
                print(
                    "[Bridge] inject joint fault "
                    f"joint={self._backend.actuator_names[joint_index]} index={joint_index} "
                    f"offline=1 error={fault.error_code} "
                    f"source={fault.source} "
                    f"start_s={fault.start_label_s:.3f} end_s={fault.end_label_s:.3f}"
                )
            for joint_index in sorted(prev_joints - next_joints):
                fault = prev_faults[joint_index]
                print(
                    "[Bridge] recover joint fault "
                    f"joint={self._backend.actuator_names[joint_index]} index={joint_index} "
                    f"source={fault.source} "
                    f"recover_s={fault.end_label_s:.3f}"
                )

        next_visual_indices = tuple(sorted(next_joints))
        if next_visual_indices != self._fault_visual_indices:
            self._backend.set_fault_visuals(next_visual_indices)
            self._fault_visual_indices = next_visual_indices

        self._active_faults = next_faults
        self._fault_active = bool(next_faults)
        return next_faults

    @property
    def sim_dt(self) -> float:
        return float(self._backend.sim_dt)

    @property
    def realtime(self) -> bool:
        return bool(self._backend.realtime)

    @property
    def step_count(self) -> int:
        return int(self._step_count)

    def will_hit_control_boundary(self) -> bool:
        return (self._step_count % self._control_decimation) == 0

    def print_startup(self, *, domain_id: int, obs_topic: str, ctrl_topic: str, fault_topic: str) -> None:
        if not self._options.print_ctrl:
            return

        sim_hz = 1.0 / max(float(self._backend.sim_dt), 1e-12)
        print(
            "[Bridge] started "
            f"domain_id={domain_id} "
            f"obs_topic={obs_topic} "
            f"ctrl_topic={ctrl_topic} "
            f"fault_topic={fault_topic} "
            f"sim_dt={self._backend.sim_dt:.6f}s sim_hz={sim_hz:.1f} "
            f"publish_decim={self._publish_decimation} control_decim={self._control_decimation}"
        )
        if self._timing_writer is not None:
            print(f"[Bridge] timing_log={self._timing_writer.path}")
        import sys

        sys.stdout.flush()

    def close(self) -> None:
        if self._timing_writer is not None:
            self._timing_writer.close()

    def _timing_log(self, message: str) -> None:
        if self._timing_writer is None:
            return
        wall_us = int(time.time() * 1_000_000)
        self._timing_writer.write(f"[TIMING] step={self._step_count} wall_us={wall_us} {message}")

    def log_loop_timing(self, *, viewer_sync_ms: float, sleep_ms: float, loop_ms: float) -> None:
        if not self._options.print_ctrl:
            return
        self._timing_log(
            "loop_cost "
            f"viewer_sync_ms={viewer_sync_ms:.3f} "
            f"sleep_ms={sleep_ms:.3f} "
            f"loop_ms={loop_ms:.3f}"
        )

    def _maybe_publish_observed(self, now: float, *, force: bool = False) -> None:
        if (not force) and (self._step_count % self._publish_decimation != 0):
            return
        active_faults = self._sync_fault_state(now)
        imu = self._backend.get_imu()
        js = self._backend.get_joint_state()

        np.copyto(self._pub_quat, np.asarray(imu.quat_wxyz, dtype=np.float32), casting="unsafe")
        np.copyto(self._pub_gyro, np.asarray(imu.gyro_xyz, dtype=np.float32), casting="unsafe")
        np.copyto(self._pub_accel, np.asarray(imu.accel_xyz, dtype=np.float32), casting="unsafe")
        np.copyto(self._pub_motor_pos, np.asarray(js.qpos, dtype=np.float32), casting="unsafe")
        np.copyto(self._pub_motor_vel, np.asarray(js.qvel, dtype=np.float32), casting="unsafe")
        np.copyto(self._pub_motor_tau, np.asarray(js.torque, dtype=np.float32), casting="unsafe")

        def _sanitize(name: str, x: np.ndarray, prev: np.ndarray) -> np.ndarray:
            if np.all(np.isfinite(x)):
                return x
            if now >= self._next_bad_obs_log_t:
                bad = np.logical_not(np.isfinite(x))
                print(f"[Bridge] WARN observed {name} non-finite: count={int(np.sum(bad))} shape={tuple(x.shape)}")
                self._next_bad_obs_log_t = now + 1.0
            bad = np.logical_not(np.isfinite(x))
            if self._last_good_valid:
                x[bad] = prev[bad]
            else:
                x[bad] = 0.0
            return x

        quat = _sanitize("quat_wxyz", self._pub_quat, self._last_good_quat)
        gyro = _sanitize("gyro_xyz", self._pub_gyro, self._last_good_gyro)
        accel = _sanitize("accel_xyz", self._pub_accel, self._last_good_accel)
        qpos = _sanitize("motor_pos", self._pub_motor_pos, self._last_good_motor_pos)
        qvel = _sanitize("motor_vel", self._pub_motor_vel, self._last_good_motor_vel)
        tau = _sanitize("motor_tau", self._pub_motor_tau, self._last_good_motor_tau)

        if quat.shape == (4,):
            norm = float(np.linalg.norm(quat))
            if (not np.isfinite(norm)) or norm < 1e-6:
                quat[:] = (1.0, 0.0, 0.0, 0.0)
            else:
                quat *= 1.0 / norm

        motor_num = int(qpos.shape[0])
        motor_enable = np.ones(motor_num, dtype=np.uint8)
        motor_online = np.ones(motor_num, dtype=np.uint8)
        motor_error = np.zeros(motor_num, dtype=np.uint8)

        for joint_index, fault in active_faults.items():
            if 0 <= joint_index < motor_num:
                motor_enable[joint_index] = 0
                motor_online[joint_index] = 0
                motor_error[joint_index] = np.uint8(int(fault.error_code) & 0xFF)

        obs = MotionObserved(
            timestamp_us=int(now * 1_000_000),
            quat_wxyz=quat,
            gyro_xyz=gyro,
            accel_xyz=accel,
            motor_pos=qpos,
            motor_vel=qvel,
            motor_tau=tau,
            motor_headers=self._headers,
            motor_enable=motor_enable,
            motor_online=motor_online,
            motor_error=motor_error,
        )
        self._transport.publish_observed(obs)
        self._observed_pub_count += 1
        np.copyto(self._last_good_quat, quat)
        np.copyto(self._last_good_gyro, gyro)
        np.copyto(self._last_good_accel, accel)
        np.copyto(self._last_good_motor_pos, qpos)
        np.copyto(self._last_good_motor_vel, qvel)
        np.copyto(self._last_good_motor_tau, tau)
        self._last_good_valid = True
        if self._options.print_ctrl and now >= self._next_obs_print_t:
            n = int(obs.motor_pos.shape[0])
            q_preview = np.array2string(obs.motor_pos[: min(4, n)], precision=3, suppress_small=False)
            qd_preview = np.array2string(obs.motor_vel[: min(4, n)], precision=3, suppress_small=False)
            print(
                f"[DDS] pub observed: count={self._observed_pub_count} ts_us={obs.timestamp_us} "
                f"n={n} q[:4]={q_preview} qd[:4]={qd_preview}"
            )
            self._next_obs_print_t = now + 1.0

    def _discard_pending_controls(self) -> int:
        dropped = 0
        while True:
            ctrl = self._transport.try_recv_control(timeout_s=0.0)
            if ctrl is None:
                return dropped
            dropped += 1

    def _take_latest_pending_control(self, first_ctrl: MotionCtrl) -> tuple[MotionCtrl, int]:
        latest = first_ctrl
        extra = 0
        while True:
            ctrl = self._transport.try_recv_control(timeout_s=0.0)
            if ctrl is None:
                return latest, extra
            latest = ctrl
            extra += 1

    def _accept_control(self, ctrl: MotionCtrl, now: float, *, source: str, wait_ms: float, queue_extra: int) -> None:
        if ctrl is not None:
            drop_after_first_s = max(float(self._options.drop_ctrl_after_first_s), 0.0)
            if drop_after_first_s > 0.0 and self._ctrl_drop_window_started and self._ctrl_drop_until_t is not None:
                if now < self._ctrl_drop_until_t:
                    if self._options.print_ctrl:
                        remain_ms = (self._ctrl_drop_until_t - now) * 1000.0
                        self._timing_log(
                            f"ctrl_drop active remain_ms={remain_ms:.1f} "
                            f"src={source} wait_ms={wait_ms:.1f} extra={queue_extra}"
                        )
                    return

            # 若对端已经输出 NaN/Inf，这里不要让它继续污染仿真
            motor_pos = np.asarray(ctrl.motor_pos, dtype=np.float32)
            motor_vel = np.asarray(ctrl.motor_vel, dtype=np.float32)
            kp = np.asarray(ctrl.kp, dtype=np.float32)
            kd = np.asarray(ctrl.kd, dtype=np.float32)
            tau_ff = np.asarray(ctrl.tau_ff, dtype=np.float32)

            # 部分对端不会填充 motor_vel / tau_ff（未初始化会是 NaN）；
            # 这里直接置 0，避免丢包导致停控。
            changed = False
            if not np.all(np.isfinite(motor_vel)):
                if self._options.print_ctrl:
                    n_bad = int(np.sum(~np.isfinite(motor_vel)))
                    print(f"[DDS] WARN recv action motor_vel has non-finite; replace with 0: n={n_bad}")
                motor_vel = np.where(np.isfinite(motor_vel), motor_vel, 0.0).astype(np.float32)
                changed = True
            if not np.all(np.isfinite(tau_ff)):
                if self._options.print_ctrl:
                    n_bad = int(np.sum(~np.isfinite(tau_ff)))
                    print(f"[DDS] WARN recv action tau_ff has non-finite; replace with 0: n={n_bad}")
                tau_ff = np.where(np.isfinite(tau_ff), tau_ff, 0.0).astype(np.float32)
                changed = True
            if changed:
                ctrl = MotionCtrl(
                    timestamp_us=int(ctrl.timestamp_us),
                    motor_pos=motor_pos,
                    motor_vel=motor_vel,
                    kp=kp,
                    kd=kd,
                    tau_ff=tau_ff,
                    motor_headers=ctrl.motor_headers,
                )

            bad = (
                (not np.all(np.isfinite(motor_pos)))
                or (not np.all(np.isfinite(kp)))
                or (not np.all(np.isfinite(kd)))
                or (not np.all(np.isfinite(tau_ff)))
            )
            if bad:
                if self._options.print_ctrl:
                    counts = {
                        "motor_pos": int(np.sum(~np.isfinite(motor_pos))),
                        "motor_vel": int(np.sum(~np.isfinite(motor_vel))),
                        "kp": int(np.sum(~np.isfinite(kp))),
                        "kd": int(np.sum(~np.isfinite(kd))),
                        "tau_ff": int(np.sum(~np.isfinite(tau_ff))),
                    }
                    print(f"[DDS] WARN recv action has non-finite fields; drop this update: {counts}")
                return

            self._last_ctrl = ctrl
            if self._first_ctrl_recv_t is None:
                self._first_ctrl_recv_t = now
            self._last_ctrl_recv_t = now
            if drop_after_first_s > 0.0 and not self._ctrl_drop_window_started:
                self._ctrl_drop_window_started = True
                self._ctrl_drop_until_t = now + drop_after_first_s
                if self._options.print_ctrl:
                    print(
                        "[DDS] start dropping controls after first accepted action: "
                        f"duration_s={drop_after_first_s:.3f}"
                    )
            msg_delta_ms = None
            if self._last_ctrl_msg_ts_us is not None and int(ctrl.timestamp_us) > int(self._last_ctrl_msg_ts_us):
                msg_delta_ms = (int(ctrl.timestamp_us) - int(self._last_ctrl_msg_ts_us)) / 1000.0

            if self._options.print_ctrl:
                timing_parts = [
                    f"src={source}",
                    f"wait_ms={wait_ms:.1f}",
                ]
                if msg_delta_ms is not None:
                    timing_parts.append(f"delta_ms={msg_delta_ms:.1f}")
                if queue_extra > 0:
                    timing_parts.append(f"extra={queue_extra}")
                self._timing_log("ctrl " + " ".join(timing_parts))

            if self._options.print_ctrl and now >= self._next_ctrl_print_t:
                n = int(ctrl.motor_pos.shape[0])
                preview = np.array2string(ctrl.motor_pos[: min(12, n)], precision=3, suppress_small=False)
                kp_preview = np.array2string(ctrl.kp[: min(3, n)], precision=2, suppress_small=False)
                kd_preview = np.array2string(ctrl.kd[: min(3, n)], precision=2, suppress_small=False)
                print(
                    f"[DDS] recv action: n={n} ts_us={ctrl.timestamp_us} pos[:12]={preview} "
                    f"kp[:3]={kp_preview} kd[:3]={kd_preview}"
                )
                if msg_delta_ms is not None:
                    print(f"[DDS] recv action delta_ms={msg_delta_ms:.1f}")
                if self._last_good_valid:
                    gmax = float(np.max(np.abs(self._last_good_gyro)))
                    amax = float(np.max(np.abs(self._last_good_accel)))
                    qmin = float(np.min(self._last_good_motor_pos))
                    qmax = float(np.max(self._last_good_motor_pos))
                    vmin = float(np.min(self._last_good_motor_vel))
                    vmax = float(np.max(self._last_good_motor_vel))
                    tmin = float(np.min(self._last_good_motor_tau))
                    tmax = float(np.max(self._last_good_motor_tau))
                    print(
                        "[Bridge] obs_stats "
                        f"gyro|max={gmax:.3f} accel|max={amax:.3f} "
                        f"q=[{qmin:.3f},{qmax:.3f}] qd=[{vmin:.3f},{vmax:.3f}] "
                        f"tau=[{tmin:.3f},{tmax:.3f}]"
                    )
                self._next_ctrl_print_t = now + 1.0 / max(self._options.print_ctrl_hz, 1e-6)
            self._last_ctrl_msg_ts_us = int(ctrl.timestamp_us)

    def _maybe_recv_control(self, now: float, *, wait_for_new: bool) -> None:
        active_faults = self._sync_fault_state(now)
        if active_faults:
            dropped = self._discard_pending_controls()
            if self._options.print_ctrl and dropped > 0:
                joints = ",".join(
                    self._backend.actuator_names[joint_index]
                    for joint_index in sorted(active_faults.keys())
                )
                self._timing_log(
                    f"ctrl_drop_fault joints={joints} count={len(active_faults)} dropped={dropped}"
                )
            return

        boundary_start = time.time()
        ctrl = self._transport.try_recv_control(timeout_s=0.0)
        if ctrl is not None:
            latest_ctrl, extra = self._take_latest_pending_control(ctrl)
            if self._options.print_ctrl and extra > 0:
                self._timing_log(f"queue_drain phase=pre extra={extra}")
            self._accept_control(
                latest_ctrl,
                now,
                source="queue",
                wait_ms=(time.time() - boundary_start) * 1000.0,
                queue_extra=extra,
            )
            return
        if wait_for_new:
            ctrl = self._transport.try_recv_control(timeout_s=self._control_period_s)
            if ctrl is not None:
                latest_ctrl, extra = self._take_latest_pending_control(ctrl)
                if self._options.print_ctrl and extra > 0:
                    self._timing_log(f"queue_drain phase=post_wait extra={extra}")
                self._accept_control(
                    latest_ctrl,
                    time.time(),
                    source="wait",
                    wait_ms=(time.time() - boundary_start) * 1000.0,
                    queue_extra=extra,
                )
                return

        if self._options.print_ctrl and self._last_ctrl_recv_t is None and (
            self._step_count % self._wait_print_decimation == 0
        ):
            obs_m, ctrl_m = self._transport.debug_match_counts()
            print(
                "[DDS] waiting for action ... "
                f"matched_obs_readers={obs_m} matched_ctrl_writers={ctrl_m}"
            )
        elif self._options.print_ctrl and self._last_ctrl_recv_t is not None and now >= self._next_ctrl_none_log_t:
            hold_age_ms = (now - float(self._last_ctrl_recv_t)) * 1000.0
            self._timing_log(f"ctrl_miss hold_age_ms={hold_age_ms:.1f}")
            self._next_ctrl_none_log_t = now + 1.0 / max(self._options.print_ctrl_hz, 1e-6)

    def _maybe_print_dds_debug(self, now: float) -> None:
        if (not self._options.dds_debug) or (now < self._next_dds_debug_t):
            return
        obs_typename, ctrl_typename = self._transport.debug_topic_typenames()
        obs_tid, ctrl_tid = self._transport.debug_local_type_ids()
        obs_m, ctrl_m = self._transport.debug_match_counts()
        print(
            "[DDS] match_state "
            f"obs_typename={obs_typename} type_id={obs_tid} matched_readers={obs_m} | "
            f"ctrl_typename={ctrl_typename} type_id={ctrl_tid} matched_writers={ctrl_m}"
        )
        peer = self._transport.debug_peer_qos_summary()
        if peer.get("peer_subscriptions_for_observed"):
            for line in peer["peer_subscriptions_for_observed"]:
                print(f"[DDS] peer(observed subscriber) {line}")
        if peer.get("peer_publications_for_control"):
            for line in peer["peer_publications_for_control"]:
                print(f"[DDS] peer(control publisher) {line}")
        self._next_dds_debug_t = now + 1.0 / max(self._options.dds_debug_hz, 1e-6)

    def _maybe_apply_control(self) -> None:
        ctrl = self._fault_hold_ctrl if (self._fault_active and self._fault_hold_ctrl is not None) else self._last_ctrl
        if ctrl is None:
            return
        now = time.time()
        active_faults = self._sync_fault_state(now)

        def _coerce_ctrl_vec(name: str, x: np.ndarray, n: int, *, fill: float) -> np.ndarray:
            arr = np.asarray(x, dtype=np.float32).reshape(-1)
            if arr.shape[0] == n:
                return arr

            out = np.full(n, fill, dtype=np.float32)
            m = min(n, arr.shape[0])
            if m > 0:
                out[:m] = arr[:m]

            if self._options.print_ctrl and now >= self._next_bad_ctrl_shape_log_t:
                print(
                    f"[DDS] WARN ctrl field `{name}` size mismatch: got={arr.shape[0]} expect={n}; "
                    f"use {'pad' if arr.shape[0] < n else 'truncate'} with fill={fill:.1f}"
                )
                self._next_bad_ctrl_shape_log_t = now + 1.0
            return out

        kp = np.asarray(ctrl.kp, dtype=np.float32)
        kd = np.asarray(ctrl.kd, dtype=np.float32)
        n = len(self._backend.actuator_names)
        if kp.shape[0] >= n and kd.shape[0] >= n:
            kp_apply = kp[:n].copy()
            kd_apply = kd[:n].copy()
            for joint_index in active_faults.keys():
                if 0 <= joint_index < n:
                    kp_apply[joint_index] = 0.0
                    kd_apply[joint_index] = 0.0
            self._backend.set_pd_gains(kp_apply, kd_apply)

        incoming = _coerce_ctrl_vec("motor_pos", ctrl.motor_pos, n, fill=0.0)

        incoming_vel = _coerce_ctrl_vec("motor_vel", ctrl.motor_vel, n, fill=0.0)
        for joint_index in active_faults.keys():
            if 0 <= joint_index < n:
                incoming_vel[joint_index] = 0.0
        self._backend.set_velocity_target(incoming_vel)

        incoming_tau_ff = _coerce_ctrl_vec("tau_ff", ctrl.tau_ff, n, fill=0.0)
        for joint_index in active_faults.keys():
            if 0 <= joint_index < n:
                incoming_tau_ff[joint_index] = 0.0
        self._backend.set_feedforward_torque(incoming_tau_ff)

        self._backend.set_position_target(incoming)

    def step_once(self) -> None:
        now = time.time()
        perf_start = time.perf_counter()
        self._poll_fault_commands(now)
        self._sync_fault_state(now)
        self._maybe_print_dds_debug(now)
        is_control_boundary = self.will_hit_control_boundary()
        t_apply_start = time.perf_counter()
        self._maybe_apply_control()
        t_apply_end = time.perf_counter()
        self._backend.step()
        t_step_end = time.perf_counter()
        step_end = time.time()
        self._maybe_publish_observed(step_end, force=is_control_boundary)
        t_publish_end = time.perf_counter()
        if is_control_boundary:
            self._maybe_recv_control(step_end, wait_for_new=not self._fault_active)
        t_recv_end = time.perf_counter()
        if self._options.print_ctrl and is_control_boundary:
            self._timing_log(
                "phase_cost "
                f"apply_ms={(t_apply_end - t_apply_start) * 1000.0:.3f} "
                f"step_ms={(t_step_end - t_apply_end) * 1000.0:.3f} "
                f"publish_ms={(t_publish_end - t_step_end) * 1000.0:.3f} "
                f"recv_ms={(t_recv_end - t_publish_end) * 1000.0:.3f} "
                f"core_ms={(t_recv_end - perf_start) * 1000.0:.3f}"
            )
        self._step_count += 1
