from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence


DEFAULT_ACTUATOR_NAMES: tuple[str, ...] = (
    "FL_hip",
    "FL_thigh",
    "FL_calf",
    "FR_hip",
    "FR_thigh",
    "FR_calf",
    "RL_hip",
    "RL_thigh",
    "RL_calf",
    "RR_hip",
    "RR_thigh",
    "RR_calf",
)


@dataclass(frozen=True)
class DdsQosConfig:
    """DDS QoS 配置（与项目默认一致）"""

    reliability: str = "BEST_EFFORT"  # BEST_EFFORT | RELIABLE
    history: str = "KEEP_LAST"  # KEEP_LAST | KEEP_ALL
    depth: int = 1
    reliable_max_blocking_time_ns: int = 100000000
    # 与对端（robotservice）保持一致：同时允许 XCDR1(v0) 与 XCDR2
    data_representation_cdrv0: bool = True
    data_representation_xcdrv2: bool = True


@dataclass(frozen=True)
class DdsConfig:
    domain_id: int = 42
    topic_motion_observed: str = "rt/motion/observed"
    topic_motion_control: str = "rt/motion/control"
    topic_motion_fault: str = "rt/motion/fault"
    topic_motion_record: str = "rt/motion/record"
    # 分 topic 的 QoS：control -> RELIABLE，observed -> BEST_EFFORT
    qos_motion_observed: DdsQosConfig = field(default_factory=DdsQosConfig)
    qos_motion_control: DdsQosConfig = field(default_factory=lambda: DdsQosConfig())
    qos_motion_fault: DdsQosConfig = field(default_factory=lambda: DdsQosConfig())
    # 兼容旧字段：若外部代码仍在用，会回退到它（建议迁移到 per-topic 字段）
    qos: DdsQosConfig = field(default_factory=DdsQosConfig)
