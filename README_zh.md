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

## SDK 验证入口

| 接口 | 入口 | 验证内容 |
|---|---|---|
| HighLevel | [simulation/scripts/highlevel_console.py](simulation/scripts/highlevel_console.py) | 已配置动作的调度和动作参数 |
| LowLevel | [simulation/scripts/run_lowlevel_onnx_policy.py](simulation/scripts/run_lowlevel_onnx_policy.py) | 直接观测、ONNX 推理和关节 PD 目标 |

HighLevel 和 LowLevel 的独立操作流程见 [SDK Sim2Sim 指南](docs/sim2sim_sdk_zh.md)。

## 最小闭环

1. 将 [mockService/uniubi_mock/](mockService/uniubi_mock/) 部署到 x86_64 Linux 主机的 `/uniubi_mock`。
2. 使用 `sudo` 和 `LD_LIBRARY_PATH=/uniubi_mock/vendor/usr/lib` 依次启动 `robotMonitorServer`、`motionServer`、`robotServer`。
3. 如果主机网卡不在默认列表中，修改 `/uniubi_mock/etc/dds/host_config.xml` 的 host DDS 网卡。
4. 在 `simulation/` 下设置 `PYTHONPATH=$(pwd)` 并启动仿真 bridge。
5. 使用 SDK 客户端连接 mock 服务，验证 HighLevel 动作或交互式 LowLevel 策略闭环。

完整命令见上方链接的各项指南。

## 支持动作

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

## 兼容性说明

- 目标运行平台：Linux `x86_64`。
- 推荐系统：Ubuntu 22.04 LTS。
- DDS：Cyclone DDS 0.10.5。
- 仿真 bridge：MuJoCo 是唯一支持的仿真后端。
- 本仓用于 SDK 集成和仿真闭环验证，不替代真机安全验证。

## 许可证

本仓库中的 UniUbi 原创代码和文档使用 Apache License 2.0。详见 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。
