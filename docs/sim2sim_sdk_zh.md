# SDK Sim2Sim

[English](sim2sim_sdk.md) | **简体中文**

仓库提供两条相互独立的验证链路：

| 接口 | mock 服务 | MuJoCo bridge | 入口 |
|---|---|---|---|
| LowLevel | 不需要 | 需要 | [`simulation/scripts/run_lowlevel_onnx_policy.py`](../simulation/scripts/run_lowlevel_onnx_policy.py) |
| HighLevel | 需要 | 需要 | [`simulation/scripts/highlevel_console.py`](../simulation/scripts/highlevel_console.py) |

在 x86_64 Linux 上，LowLevel SDK 会选择 external-simulation backend，直接通过 DDS
与 MuJoCo bridge 交换控制和观测，不经过 `robotMonitorServer`、`motionServer` 或
`robotServer`。HighLevel 使用 RobotService 中配置的动作，因此需要 mock 运行包和
三个服务。

## 安装

### 1. mock 服务（仅 HighLevel）

HighLevel 需要 RobotService mock 运行环境。按照
[Mock Service 开发指南](mock_service_zh.md) 将 `mockService/uniubi_mock/` 部署到
`/uniubi_mock`。

**LowLevel 跳过这一步。** x86 LowLevel SDK 直接与 MuJoCo bridge 通信，不使用任何
mock 服务进程。

### 2. 公共仿真环境和 SDK 库

HighLevel 和 LowLevel 都需要本仓库、仿真依赖及公开 Python SDK。下面将三个仓库
放在同一级目录：

```bash
cd ~
git clone https://github.com/uniubi-ai/uniubi_robot_mock.git
git clone https://github.com/uniubi-ai/uniubi_robot_sdk.git
git clone https://github.com/uniubi-ai/uniubi_robot_sdk_py.git

python3 -m pip install -r ~/uniubi_robot_mock/simulation/requirements.txt

export UNIUBI_SDK_ROOT=~/uniubi_robot_sdk
cd ~/uniubi_robot_sdk_py
env UNIUBI_SDK_ROOT="$UNIUBI_SDK_ROOT" python3 -m pip install .
```

运行任一 CLI 前，配置同版本 SDK 动态库：

```bash
export UNIUBI_SDK_ROOT=~/uniubi_robot_sdk
export LD_LIBRARY_PATH="$UNIUBI_SDK_ROOT/lib/$(uname -m):${LD_LIBRARY_PATH}"
```

完成公共安装后可直接验证 LowLevel。HighLevel 还需完成第 1 步的 mock 服务安装。

## LowLevel SDK 验证

### 1. 启动 MuJoCo bridge

终端 1：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --viewer
```

无显示环境将 `--viewer` 改为 `--headless`。收到 LowLevel 指令前，机器人以稳定趴姿
出生并持续保持。

### 2. 启动 LowLevel CLI

终端 2：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python scripts/run_lowlevel_onnx_policy.py
```

仅在不安装 Python binding、直接使用源码调试时，才传入
`--sdk-python /path/containing/robot_motion_sdk`。

CLI 只使用仓库内置的
[`simulation/models/policy.onnx`](../simulation/models/policy.onnx)：一个以 50 Hz
运行、45 维输入、12 维输出的 Cyvet 速度策略。

```text
stand
walk 0.5 0 0
state
stop
lay
quit
```

| 命令 | 行为 |
|---|---|
| `stand` | 使能 LowLevel 控制，并用 2 秒从实测姿态平滑切到站立 |
| `walk [VX VY YAW]` | 运行策略，默认参数为 `0.5 0 0` |
| `stop` | 停止策略推理并回到站立目标 |
| `lay` | 平滑回到趴下目标 |
| `state` | 显示 SDK 状态、当前姿态、速度指令和控制帧计数 |
| `obs` | 显示最新的 12 个关节实测位置 |
| `quit` | 执行安全清理、退出 LowLevel 控制、断开连接并结束程序 |

如果输入 `walk` 时不是站立姿态，CLI 会先执行 2 秒站立过渡。趴下和站立使用与
mock MotionServer 配置一致的分关节位控增益。

LowLevel 数据链路为：

```text
MotionLowLevelClient（external simulation backend）
  -> rt/motion/control
  -> MuJoCo bridge
  -> rt/motion/observed
  -> MotionLowLevelClient
```

## HighLevel SDK 验证

### 1. 启动 mock 服务

完成安装第 1 步后，按照 [Mock Service 开发指南](mock_service_zh.md) 启动
`robotMonitorServer`、`motionServer` 和 `robotServer`。MotionServer 会创建实时
控制线程，因此三个服务均需使用 `sudo` 启动。

### 2. 启动 MuJoCo bridge

按照 LowLevel 章节中的命令启动同一个 bridge。如果 bridge 已运行，不要重复启动。

### 3. 启动 HighLevel CLI

终端 2：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd):$ROBOTSDK_PYTHON_PATH

python scripts/highlevel_console.py --iface <网卡名>
```

Walking 的典型测试流程为：

```text
start walking
set {"lineVelocityX":0.5,"lineVelocityY":0,"velocity":0}
state
zero
stop
quit
```

- `set` 持续保持参数。
- `send 3 {...}` 保持参数 3 秒后自动清零。
- `zero` 只清除参数，不停止当前动作。
- `start` 成功仅表示请求已被接受，实际执行动作应通过 `state` 确认。

`bipedStand`、`handstand`、`leftSideStand` 和 `rightSideStand` 是持续动作。使用
`stop` 返回 Walking，等待 `state` 显示 `walking` 后再发送行走参数。输入 `help`
查看完整命令列表。

## DDS 网卡选择

主机存在多张网卡时：

```bash
cd /path/to/uniubi_robot_mock/simulation
source scripts/setup_dds.sh <网卡名>
```

使用 `ip -br addr` 查找承载 DDS 流量的网卡。HighLevel 还需在 mock 运行包的 host
DDS 配置中使用同一网卡，详见
[Mock Service 开发指南](mock_service_zh.md#host-dds-网卡配置)。

## 常见问题

- 如果无法导入 `robot_motion_sdk`，确认 `ROBOTSDK_PYTHON_PATH` 包含
  `robot_motion_sdk/__init__.py`。
- 如果 LowLevel 收不到观测，确认 MuJoCo bridge 与 client 都使用 DDS domain `42`，
  并匹配 `rt/motion/control` 和 `rt/motion/observed` topic。此链路不涉及 mock 服务。
- 如果 HighLevel RPC 可以连接但动作不运行，确认三个 mock 服务都使用 `sudo` 启动，
  并在服务 ready 后再启动 HighLevel client。

ONNXRuntime CLI 用于 x86 仿真和 SDK 接口联调，不用于板端策略部署。
