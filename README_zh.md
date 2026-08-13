# uniubi_robot_mock

[English](README.md) | **简体中文**

用于在没有真机的情况下进行 SDK 集成开发的 RobotService mock 运行包和仿真 bridge。

## 当前包含内容

| 内容 | 路径 | 说明 |
|---|---|---|
| mock 运行包 | [mockService/uniubi_mock/](mockService/uniubi_mock/) | 部署到 x86_64 Linux 主机 `/uniubi_mock` 的自包含运行环境 |
| 仿真 bridge | [simulation/sim2sim/](simulation/sim2sim/) | MuJoCo 后端，与 mock runtime 交换运控控制和机器人状态 |
| DDS 网卡脚本 | [simulation/scripts/setup_dds.sh](simulation/scripts/setup_dds.sh) | 为当前 shell 绑定 Cyclone DDS 网卡 |
| mock 服务说明 | [docs/mock_service.md](docs/mock_service.md) | 部署、启动、校验和排障 |
| 仿真环境说明 | [docs/simulation_setup.md](docs/simulation_setup.md) | MuJoCo 环境准备和 bridge 启动 |

## 安装

1. **mock 服务（仅 HighLevel）：**按照 [mock 服务说明](docs/mock_service.md)部署
   [mockService/uniubi_mock/](mockService/uniubi_mock/)。LowLevel 跳过这一步。
2. **公共环境：**按照 [SDK Sim2Sim 指南](docs/sim2sim_sdk_zh.md#安装)安装 MuJoCo
   仿真依赖、公开 Python SDK 和同版本 SDK 动态库。

## HighLevel

HighLevel 需要 mock 服务和 MuJoCo bridge。使用
[highlevel_console.py](simulation/scripts/highlevel_console.py)验证已配置动作的调度和
动作参数。启动及测试步骤见
[HighLevel SDK 验证](docs/sim2sim_sdk_zh.md#highlevel-sdk-验证)。

### 支持动作

当前 mock runtime 支持：

- `laying`
- `standing`
- `walking`
- `emergencyStop`
- `waveHand`
- `bipedStand`
- `handstand`
- `leftSideStand`
- `rightSideStand`

当已支持动作上报跌倒时，运行时还会自动使用内部 `recovery` 动作；该动作不作为用户可主动启动的动作暴露。

双足站类动作是持续动作。通过 HighLevel console 返回 Walking 时，应先执行 `stop`，
并用 `state` 确认当前动作已经是 `walking`，再下发 Walking 速度参数。

## LowLevel

LowLevel 不需要 mock 服务。在 x86_64 Linux 上，它通过 DDS 直接与 MuJoCo bridge
通信。使用
[run_lowlevel_onnx_policy.py](simulation/scripts/run_lowlevel_onnx_policy.py)验证直接
观测、内置 ONNX 推理和 12 关节 PD 目标。启动及测试步骤见
[LowLevel SDK 验证](docs/sim2sim_sdk_zh.md#lowlevel-sdk-验证)。

## 兼容性说明

- 目标运行平台：Linux `x86_64`。
- 推荐系统：Ubuntu 22.04 LTS。
- DDS：Cyclone DDS 0.10.5。
- 仿真 bridge：MuJoCo 是唯一支持的仿真后端。
- 本仓用于 SDK 集成和仿真闭环验证，不替代真机安全验证。

## 许可证

本仓库中的 UniUbi 原创代码和文档使用 Apache License 2.0。详见 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。
