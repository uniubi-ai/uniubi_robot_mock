# 支持矩阵

[English](support_matrix.md) | **简体中文**

| 维度 | 支持范围 |
|---|---|
| Mock 运行平台 | Linux x86_64 |
| 推荐操作系统 | Ubuntu 22.04 LTS |
| 运行时根目录 | `/uniubi_mock` |
| DDS | Cyclone DDS 0.10.5 |
| Host DDS Domain | 42 |
| Motion DDS Domain | 1 |
| 仿真后端 | MuJoCo |
| 默认验证后端 | MuJoCo |
| MuJoCo Python | Python 3.11 环境 |
| 支持动作 | `laying`、`standing`、`walking`、`emergencyStop`、`waveHand`、`bipedStand`、`handstand`、`leftSideStand`、`rightSideStand` |
| 内部安全动作 | `recovery`（检测到跌倒后自动调度） |
| 非目标 | 真机安全验证、强化学习训练、高保真生产级物理仿真的替代方案 |
