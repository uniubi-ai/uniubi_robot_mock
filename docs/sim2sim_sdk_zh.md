# SDK Sim2Sim

[English](sim2sim_sdk.md) | **简体中文**

本文档说明两条相互独立的 SDK 仿真验证链路。两者共用 RobotService mock 运行环境和
MuJoCo bridge：

| 接口 | 入口 | 控制层级 |
|---|---|---|
| HighLevel | [`simulation/scripts/highlevel_console.py`](../simulation/scripts/highlevel_console.py) | 启停已配置的运控动作，并更新动作参数 |
| LowLevel | [`simulation/scripts/run_lowlevel_onnx_policy.py`](../simulation/scripts/run_lowlevel_onnx_policy.py) | 运行内置 ONNX 策略，直接发送 12 关节 PD 目标 |

运行 mock 服务的 x86_64 Linux 主机代表机器狗本体。SDK client 可以运行在同一主机，
理论上也可运行在 DDS 网络可达的其他 Linux 主机。以下命令采用已完成验证的同机方式。

## 公共准备

### 1. 启动 mock 服务

按照 [Mock Service 开发指南](mock_service.md) 部署并启动 `robotMonitorServer`、
`motionServer` 和 `robotServer`。MotionServer 会创建实时控制线程，因此三个服务均需
使用 `sudo` 启动。

### 2. 安装仿真依赖

```bash
python -m pip install -r simulation/requirements.txt
```

设置公开 Python SDK 所在目录：

```bash
export ROBOTSDK_PYTHON_PATH=/path/to/robotsdk/Sdk/Python
```

### 3. 启动 MuJoCo bridge

终端 1：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --viewer
```

无显示环境将 `--viewer` 改为 `--headless`。Bridge 发布
`rt/motion/observed`，订阅 `rt/motion/control`。机器人以稳定趴姿出生，在 client
使能或启动运控前不会收到 SDK 控制指令。

### 可选：绑定 DDS 网卡

主机存在多张网卡时：

```bash
cd /path/to/uniubi_robot_mock/simulation
source scripts/setup_dds.sh <网卡名>
```

使用 `ip -br addr` 查找承载 DDS 流量的网卡。

## HighLevel SDK 验证

HighLevel 用于验证 `walking`、`waveHand`、双足站等已配置动作，覆盖动作调度和动作
参数，不直接发送关节目标。

终端 2：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd):$ROBOTSDK_PYTHON_PATH

python scripts/highlevel_console.py --iface <网卡名>
```

控制台不模拟遥控器组合键。Walking 的典型测试流程为：

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
- `zero` 只清除动作参数，不停止当前动作。
- `start` 成功仅表示请求已被接受；实际执行动作应通过 `state` 确认。

`bipedStand`、`handstand`、`leftSideStand`、`rightSideStand` 是持续动作。使用
`stop` 返回 Walking，等待 `state` 显示 `walking` 后再发送行走参数：

```text
start bipedStand
stop
state
set {"lineVelocityX":0.5,"lineVelocityY":0,"velocity":0}
```

输入 `help` 查看完整 HighLevel 命令列表。

## LowLevel SDK 验证

LowLevel 用于验证从观测到关节控制的直接闭环。此示例只支持仓库内置的
[`simulation/models/policy.onnx`](../simulation/models/policy.onnx)：一个以 50 Hz
运行、45 维输入、12 维输出的 Cyvet 速度策略。

终端 2：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)
export LD_LIBRARY_PATH=/uniubi_mock/vendor/usr/lib:$LD_LIBRARY_PATH

python scripts/run_lowlevel_onnx_policy.py \
  --sdk-python "$ROBOTSDK_PYTHON_PATH"
```

Client 连接后不会立即发送关节指令。典型操作流程为：

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

如果输入 `walk` 时记录的当前姿态不是站立，CLI 会先执行同一套 2 秒站立过渡，完成
后才启动策略。趴下和站立使用与 mock MotionServer 配置一致的分关节位控增益。

这个 ONNXRuntime CLI 用于 x86 仿真和 LowLevel SDK 接口联调，不用于板端策略部署。

## 常见问题

如果无法导入 `robot_motion_sdk`，请确认 `ROBOTSDK_PYTHON_PATH` 指向包含
`robot_motion_sdk/__init__.py` 的目录。

如果 DDS endpoint 无法匹配，先确认两端保持以下默认 topic：

- control：`rt/motion/control`
- observed：`rt/motion/observed`
- TRC：`rt/motion/trc`

如果 RPC 可以连接但动作不运行，确认三个 mock 服务都使用 `sudo` 启动，并在服务
ready 后再启动 MuJoCo bridge 和 SDK client。
