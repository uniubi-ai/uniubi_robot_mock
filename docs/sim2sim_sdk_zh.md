# SDK Sim2Sim

这条链路通过 DDS topic 将 UniUbi 低级 SDK client 接到 MuJoCo 仿真，用于在不上真机的情况下验证 SDK 侧低级控制代码。

数据流：

```text
ONNX policy 或 SDK client
  -> MotionLowLevelClient(simulation)
  -> rt/motion/control
  -> MuJoCo bridge
  -> rt/motion/observed
  -> MotionLowLevelClient(simulation)
```

## 安装依赖

在你的 Python 环境中安装仿真依赖：

```bash
python -m pip install -r simulation/requirements.txt
```

Policy client 还需要 UniUbi Python SDK。可以通过参数传入包含 `robot_motion_sdk` 的目录，也可以先设置环境变量：

```bash
export ROBOTSDK_PYTHON_PATH=/path/to/robotsdk/Sdk/Python
```

## 启动 MuJoCo Bridge

终端 1：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --viewer
```

无界面机器：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --headless
```

Bridge 会发布 `rt/motion/observed`，并订阅 `rt/motion/control`。

## 启动 ONNX Policy Client

终端 2：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)
export LD_LIBRARY_PATH=/uniubi_mock/vendor/usr/lib:$LD_LIBRARY_PATH

python scripts/run_lowlevel_onnx_policy.py \
  --sdk-python "$ROBOTSDK_PYTHON_PATH"
```

机器人以趴下姿态出生；没有 LowLevel 指令时会持续保持趴姿。CLI 固定运行仓库内置的
`simulation/models/policy.onnx`，典型操作顺序如下：

```text
stand
walk 0.5 0 0
state
stop
lay
quit
```

`stand` 从实测趴下姿态平滑站立；`walk` 以 50 Hz 运行固定的 45 维输入策略。如果
当前不是站立姿态，`walk` 会自动先执行同一套 2 秒站立过渡，再启动策略。`stop`
停止策略推理并回到站立；`lay` 平滑回到趴下姿态。`state` 会显示 SDK 状态、当前
姿态、速度指令及控制帧成功/失败计数，`obs` 会打印最新的 12 个关节位置。

趴下和站立过渡使用与 mock MotionServer 配置一致的分关节位控增益。`quit` 会执行
LowLevel 安全清理、退出控制并断开客户端。

这个 ONNXRuntime CLI 仅用于 x86 仿真和 LowLevel SDK 接口联调。

## 可选：绑定 DDS 网卡

如果机器上有多张网卡，可以在当前 shell 中绑定 CycloneDDS：

```bash
cd /path/to/uniubi_robot_mock/simulation
source scripts/setup_dds.sh <iface>
```

用 `ip -br addr` 查看网卡，选择承载 DDS 流量的那一张。

## HighLevel 交互控制台

HighLevel mock 与公开 Python SDK 使用相同的命令式控制台，不再模拟遥控器组合键。先安装当前 Python SDK，并启动 MuJoCo bridge，然后运行：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python scripts/highlevel_console.py --iface <网卡名>
```

运行服务的 x86 主机本身就是 mock 设备，不需要发现或选择设备。

进入 `highlevel>` 后可按以下方式验证 Walking：

```text
start walking
set {"lineVelocityX":0.5,"lineVelocityY":0,"velocity":0}
zero
set {"lineVelocityX":-1.0,"lineVelocityY":0,"velocity":0}
zero
state
```

`set` 会持续保持指令，`send 3 {...}` 会保持 3 秒后自动清零；`zero` 只清除
当前动作参数，不会结束动作。`start` 成功表示切换请求已被服务接受，实际执行动作
应通过 `state` 确认。

`bipedStand`、`handstand`、`leftSideStand`、`rightSideStand` 是持续动作。从这些
动作回到 Walking 时，先执行 `stop`，等待 `state` 返回 `walking`，再发送 Walking
速度参数：

```text
start bipedStand
stop
state
set {"lineVelocityX":-1.0,"lineVelocityY":0,"velocity":0}
```

不要把 `start walking` 的 RPC 成功响应当作持续动作已经退出。输入 `help` 可查看
完整命令。

## 常见问题

如果报 `robot_motion_sdk` 无法导入，检查 `ROBOTSDK_PYTHON_PATH` 是否指向包含 `robot_motion_sdk/__init__.py` 的目录。

如果 DDS topic 无法匹配，先保持两侧默认 topic：

- control: `rt/motion/control`
- observed: `rt/motion/observed`
- TRC: `rt/motion/trc`

三个 mock 服务需要按 `docs/mock_service.md` 使用 `sudo` 启动，否则 MotionServer
可能因缺少实时调度权限而无法创建控制线程。建议先启动三个服务，确认 ready 后再
启动 MuJoCo bridge。
