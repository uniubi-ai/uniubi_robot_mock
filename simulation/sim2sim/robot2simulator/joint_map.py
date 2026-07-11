from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class MotorHeader:
    limbs_no: int
    joint_no: int


_LEG_INDEX = {
    "FL": 0,
    "FR": 1,
    "RL": 2,
    "RR": 3,
}

_JOINT_INDEX = {
    "hip": 0,
    "abad": 0,
    "thigh": 1,
    "calf": 2,
    "knee": 2,
}


def default_headers_for_12dof_leg_order() -> Sequence[MotorHeader]:
    headers: list[MotorHeader] = []
    for i in range(12):
        headers.append(MotorHeader(limbs_no=i // 3, joint_no=i % 3))
    return tuple(headers)


def headers_for_actuator_names(actuator_names: Sequence[str]) -> Sequence[MotorHeader]:
    headers: list[MotorHeader] = []
    for i, name in enumerate(actuator_names):
        parts = str(name).split("_")
        if len(parts) >= 2:
            leg = _LEG_INDEX.get(parts[0].upper())
            joint = _JOINT_INDEX.get(parts[-1].lower())
            if leg is not None and joint is not None:
                headers.append(MotorHeader(limbs_no=leg, joint_no=joint))
                continue
        headers.append(MotorHeader(limbs_no=i // 3, joint_no=i % 3))
    return tuple(headers)
