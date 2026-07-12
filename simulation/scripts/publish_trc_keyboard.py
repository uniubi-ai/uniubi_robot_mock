#!/usr/bin/env python3
from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager

DIGITAL_FIELDS = (
    "back",
    "start",
    "lb",
    "rb",
    "f1",
    "f2",
    "a",
    "b",
    "x",
    "y",
    "up",
    "down",
    "left",
    "right",
    "ls",
    "rs",
)
ANALOG_FIELDS = ("stickLX", "stickLY", "stickRX", "stickRY", "triggerL", "triggerR")
PRESETS = {
    "zero": {},
    "emergency": {"lb": 1, "rb": 1},
    "bipedStand": {"lb": 1, "y": 1},
    "handstand": {"lb": 1, "a": 1},
    "laying": {"start": 1, "a": 1},
    "walking": {"start": 1, "y": 1},
    "standing": {"back": 1},
    "waveBody": {"lb": 1, "start": 1},
    "peakLoadStand": {"y": 1, "triggerL": 1.0},
    "jumpFrontflip": {"rb": 1, "y": 1},
    "jumpSideflip": {"rb": 1, "b": 1, "triggerR": -1.0},
    "jumpBackflip": {"rb": 1, "a": 1, "triggerR": -1.0},
    "jumpDoubleBackflip": {"rb": 1, "a": 1, "triggerR": 1.0},
    "jumpDoubleSideflip": {"rb": 1, "b": 1, "triggerR": 1.0},
}


def _build_payload(controller: int) -> dict[str, float | int]:
    payload: dict[str, float | int] = {"controller": int(controller), "timestamp": 0}
    payload.update({field: 0 for field in DIGITAL_FIELDS})
    payload.update({field: 0.0 for field in ANALOG_FIELDS})
    return payload


def _make_message(payload: dict[str, float | int]):
    from sim2sim.robot2simulator.transport.cyclonedds_transport import RemoteControl_, _idl_make

    return _idl_make(
        RemoteControl_,
        controller=int(payload.get("controller") or 0),
        timestamp=int(time.time() * 1_000),
        back=int(payload.get("back", 0)),
        start=int(payload.get("start", 0)),
        lb=int(payload.get("lb", 0)),
        rb=int(payload.get("rb", 0)),
        f1=int(payload.get("f1", 0)),
        f2=int(payload.get("f2", 0)),
        a=int(payload.get("a", 0)),
        b=int(payload.get("b", 0)),
        x=int(payload.get("x", 0)),
        y=int(payload.get("y", 0)),
        up=int(payload.get("up", 0)),
        down=int(payload.get("down", 0)),
        left=int(payload.get("left", 0)),
        right=int(payload.get("right", 0)),
        ls=int(payload.get("ls", 0)),
        rs=int(payload.get("rs", 0)),
        stickLX=float(payload.get("stickLX", 0.0)),
        stickLY=float(payload.get("stickLY", 0.0)),
        stickRX=float(payload.get("stickRX", 0.0)),
        stickRY=float(payload.get("stickRY", 0.0)),
        triggerL=float(payload.get("triggerL", 0.0)),
        triggerR=float(payload.get("triggerR", 0.0)),
    )


class TrcPublisher:
    def __init__(self, *, domain: int, topic: str, reliability: str, depth: int) -> None:
        from cyclonedds.domain import DomainParticipant
        from cyclonedds.topic import Topic
        from sim2sim.robot2simulator.transport.cyclonedds_transport import (
            RemoteControl_,
            _build_qos,
            _make_writer,
            _require_cyclonedds,
        )

        _require_cyclonedds()
        qos = _build_qos(
            reliability,
            "KEEP_LAST",
            depth,
            0,
            True,
            True,
        )
        self._participant = DomainParticipant(domain)
        self._topic = Topic(self._participant, topic, RemoteControl_, qos=qos)
        self._writer = _make_writer(self._participant, self._topic, qos)

    def publish(self, payload: dict[str, float | int]) -> None:
        self._writer.write(_make_message(payload))


@contextmanager
def _raw_terminal(enabled: bool):
    if not enabled or not sys.stdin.isatty():
        yield
        return
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        yield
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def _read_key() -> str | None:
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return None
    return sys.stdin.read(1)


def _print_help() -> None:
    print(
        "TRC keyboard controls:\n"
        "  w/s: forward/back (stickLY)    a/d: lateral (stickRX)\n"
        "  q/e: yaw (stickLX)             space: zero axes/buttons\n"
        "  1: handstand(LB+A)  2: standing(Back)  3: walking(Start+Y)\n"
        "  4: laying(Start+A)  5: waveBody(LB+Start)\n"
        "  z: emergency(LB+RB) x: zero frame\n"
        "  Ctrl-C: exit\n",
        flush=True,
    )


def _apply_key(payload: dict[str, float | int], key: str, controller: int) -> float:
    release_at = 0.0
    if key in {" ", "x"}:
        payload.clear()
        payload.update(_build_payload(controller))
    elif key == "w":
        payload["stickLY"] = 1.0
    elif key == "s":
        payload["stickLY"] = -1.0
    elif key == "a":
        payload["stickRX"] = 1.0
    elif key == "d":
        payload["stickRX"] = -1.0
    elif key == "q":
        payload["stickLX"] = 1.0
    elif key == "e":
        payload["stickLX"] = -1.0
    elif key == "1":
        payload.clear()
        payload.update(_build_payload(controller))
        payload.update(PRESETS["handstand"])
        release_at = time.monotonic() + 0.2
    elif key == "2":
        payload.clear()
        payload.update(_build_payload(controller))
        payload.update(PRESETS["standing"])
        release_at = time.monotonic() + 0.2
    elif key == "3":
        payload.clear()
        payload.update(_build_payload(controller))
        payload.update(PRESETS["walking"])
        release_at = time.monotonic() + 0.2
    elif key == "4":
        payload.clear()
        payload.update(_build_payload(controller))
        payload.update(PRESETS["laying"])
        release_at = time.monotonic() + 0.2
    elif key == "5":
        payload.clear()
        payload.update(_build_payload(controller))
        payload.update(PRESETS["waveBody"])
        release_at = time.monotonic() + 0.2
    elif key == "z":
        payload.clear()
        payload.update(_build_payload(controller))
        payload.update(PRESETS["emergency"])
        release_at = time.monotonic() + 0.2
    return release_at


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish virtual RemoteControl_ frames to rt/motion/trc.")
    parser.add_argument("--domain", type=int, default=42)
    parser.add_argument("--topic", default="rt/motion/trc")
    parser.add_argument(
        "--raw-action-id",
        type=int,
        default=None,
        help="Override the local simulation action id. Alias of --controller.",
    )
    parser.add_argument(
        "--controller",
        type=int,
        default=1,
        help="RemoteControl_.controller value for local simulation.",
    )
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--reliability", choices=["BEST_EFFORT", "RELIABLE"], default="BEST_EFFORT")
    parser.add_argument("--depth", type=int, default=1)
    parser.add_argument("--preset", choices=sorted(PRESETS), default=None)
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to publish; 0 means until Ctrl-C.")
    args = parser.parse_args()

    raw_action_id = args.raw_action_id if args.raw_action_id is not None else args.controller
    if int(raw_action_id) <= 0:
        parser.error("--controller/--raw-action-id must be positive.")

    publisher = TrcPublisher(
        domain=args.domain,
        topic=args.topic,
        reliability=args.reliability,
        depth=args.depth,
    )
    payload = _build_payload(int(raw_action_id))
    if args.preset:
        payload.update(PRESETS[args.preset])

    period = 1.0 / max(float(args.rate), 1.0)
    stop_at = time.monotonic() + float(args.duration) if args.duration > 0 else None
    interactive = args.preset is None
    release_at = 0.0
    print(f"Publishing RemoteControl_ domain={args.domain} topic={args.topic} rate={args.rate:.1f}Hz")
    if interactive:
        _print_help()
    try:
        with _raw_terminal(interactive):
            while stop_at is None or time.monotonic() < stop_at:
                if interactive:
                    key = _read_key()
                    if key:
                        release_at = _apply_key(payload, key, int(raw_action_id)) or release_at
                if release_at and time.monotonic() >= release_at:
                    payload.clear()
                    payload.update(_build_payload(int(raw_action_id)))
                    release_at = 0.0
                publisher.publish(payload)
                time.sleep(period)
    except KeyboardInterrupt:
        pass

    zero = _build_payload(int(raw_action_id))
    for _ in range(3):
        publisher.publish(zero)
        time.sleep(period)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
