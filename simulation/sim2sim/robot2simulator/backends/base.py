from __future__ import annotations

import abc
import contextlib
from typing import Any, ContextManager, Optional, Sequence, Protocol

import numpy as np

from sim2sim.robot2simulator.sim_types import ImuSample, JointState


class SimBackend(abc.ABC):
    """仿真后端抽象：BridgeCore 只依赖这里的最小能力集。"""

    def __init__(self, *, actuator_names: tuple[str, ...], sim_dt: float, realtime: bool, headless: bool) -> None:
        self.actuator_names = actuator_names
        self.sim_dt = float(sim_dt)
        self.realtime = bool(realtime)
        self.headless = bool(headless)

    @abc.abstractmethod
    def reset(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def step(self) -> None:
        raise NotImplementedError

    @abc.abstractmethod
    def set_position_target(self, target_pos: np.ndarray) -> None:
        raise NotImplementedError

    def set_velocity_target(self, target_vel: np.ndarray) -> None:
        """可选：设置目标关节速度（用于力矩 PD）。"""

    def set_feedforward_torque(self, tau_ff: np.ndarray) -> None:
        """可选：设置前馈力矩（用于力矩控制）。"""

    def set_pd_gains(self, kp: np.ndarray, kd: np.ndarray) -> None:
        """可选：设置 PD 参数（不同后端可能不支持或不需要每步设置）。"""

    @abc.abstractmethod
    def get_imu(self) -> ImuSample:
        raise NotImplementedError

    @abc.abstractmethod
    def get_joint_state(self) -> JointState:
        raise NotImplementedError

    def viewer_context(self, enabled: bool) -> ContextManager[Optional[Any]]:
        """返回一个可选 viewer 的上下文管理器；enabled=False 时应返回 None。"""
        _ = enabled
        return contextlib.nullcontext(None)

    def viewer_is_running(self, viewer: Any) -> bool:
        _ = viewer
        return True

    def viewer_sync(self, viewer: Any) -> None:
        _ = viewer

    def set_fault_visual(self, joint_index: Optional[int], active: bool) -> None:
        """可选：在 viewer 中高亮/恢复故障关节对应的机器人部件。"""
        _ = joint_index
        _ = active

    def set_fault_visuals(self, joint_indices: Sequence[int]) -> None:
        """可选：在 viewer 中高亮一组故障关节对应的机器人部件。"""
        _ = joint_indices

    def close(self) -> None:
        """可选：释放仿真资源。"""
