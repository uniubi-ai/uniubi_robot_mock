# uniubi_robot_mock

用于在没有真机的情况下进行 SDK 集成开发的 RobotService mock 运行包和仿真 bridge。

## 当前包含内容

| 内容 | 路径 | 说明 |
|---|---|---|
| mock 运行包 | `mockService/uniubi_mock/` | 部署到 Linux VM `/uniubi_mock` 的 x86_64 自包含运行环境 |
| 仿真 bridge | `simulation/sim2sim/` | MuJoCo / Isaac Gym 后端，与 mock runtime 交换运控控制和机器人状态 |
| DDS 网卡脚本 | `simulation/scripts/setup_dds.sh` | 为当前 shell 绑定 Cyclone DDS 网卡 |
| mock 服务说明 | `docs/mock_service.md` | 部署、启动、校验和排障 |
| 仿真环境说明 | `docs/simulation_setup.md` | MuJoCo / Isaac Gym 环境准备和 bridge 启动 |

## 最小闭环

1. 将 `mockService/uniubi_mock/` 部署到 x86_64 Ubuntu VM 的 `/uniubi_mock`。
2. 使用 `LD_LIBRARY_PATH=/uniubi_mock/vendor/usr/lib` 启动 `robotMonitorServer`、`motionServer`、`robotServer`。
3. 如果 VM 网卡不在默认列表中，修改 `/uniubi_mock/etc/dds/host_config.xml` 的 host DDS 网卡。
4. 在 `simulation/` 下设置 `PYTHONPATH=$(pwd)` 并启动仿真 bridge。
5. 使用 SDK 客户端连接 mock 服务，验证 `standing`、`walking`、`laying` 等高级动作。

完整命令见 [docs/mock_service.md](docs/mock_service.md) 和 [docs/simulation_setup.md](docs/simulation_setup.md)。

## 支持动作

当前 mock runtime 支持：

- `laying`
- `standing`
- `walking`
- `emergencyStop`
- `jumpFrontflip`
- `jumpSideflip`
- `jumpBackflip`

## 兼容性说明

- 目标运行平台：Linux `x86_64`。
- 推荐系统：Ubuntu 22.04 LTS。
- DDS：Cyclone DDS 0.10.5。
- 仿真 bridge：MuJoCo 是默认验证后端；Isaac Gym 需要 NVIDIA GPU 和独立 Python 3.8 环境。
- 本仓用于 SDK 集成和仿真闭环验证，不替代真机安全验证。

## 许可证

本仓库中的 UniUbi 原创代码和文档使用 Apache License 2.0。详见 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。
