from __future__ import annotations

import abc
from typing import Optional

from sim2sim.robot2simulator.motion_messages import MotionCtrl, MotionFault, MotionObserved


class MotionTransport(abc.ABC):
    @abc.abstractmethod
    def publish_observed(self, msg: MotionObserved) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def try_recv_control(self, timeout_s: float) -> Optional[MotionCtrl]:
        raise NotImplementedError

    def try_recv_faults(self, timeout_s: float) -> list[MotionFault]:
        _ = timeout_s
        return []

    # ---- 调试接口（可选）----
    def debug_match_counts(self) -> tuple[int, int]:
        return (0, 0)

    def debug_topic_typenames(self) -> tuple[str, str]:
        return ("", "")

    def debug_local_type_ids(self) -> tuple[str, str]:
        return ("", "")

    def debug_peer_qos_summary(self) -> dict[str, list[str]]:
        return {}
