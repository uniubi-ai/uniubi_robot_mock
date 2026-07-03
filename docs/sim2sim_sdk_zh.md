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
  --backend mujoco \
  --viewer
```

无界面机器：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)
export MUJOCO_GL=osmesa

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --backend mujoco \
  --headless
```

Bridge 会发布 `rt/motion/observed`，并订阅 `rt/motion/control`。

## 启动 ONNX Policy Client

终端 2：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python scripts/run_lowlevel_onnx_policy.py \
  --sdk-python "$ROBOTSDK_PYTHON_PATH" \
  --model /path/to/policy.onnx \
  --duration 30 \
  --rate 50 \
  --cmd-x 0.5
```

这个 helper 会构造 Cyvet 速度策略使用的 45 维 observation，并通过 `MotionLowLevelClient` 发送关节位置 target。

板端部署建议使用 TensorRT engine 进行策略推理。上面的 ONNXRuntime helper 主要用于 x86 仿真验证和 SDK 接口联调。

## 可选：绑定 DDS 网卡

如果机器上有多张网卡，可以在当前 shell 中绑定 CycloneDDS：

```bash
cd /path/to/uniubi_robot_mock/simulation
source scripts/setup_dds.sh <iface>
```

用 `ip -br addr` 查看网卡，选择承载 DDS 流量的那一张。

## 可选：虚拟遥控

ONNX helper 默认使用 `--cmd-x/--cmd-y/--cmd-yaw` 作为控制指令。也可以读取 `motion/trc`：

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python scripts/publish_trc_keyboard.py \
  --domain 42 \
  --topic motion/trc \
  --rate 50
```

按键：

- `w/s`：前进/后退
- `a/d`：横移
- `q/e`：转向
- `space` 或 `x`：归零

## 常见问题

如果报 `robot_motion_sdk` 无法导入，检查 `ROBOTSDK_PYTHON_PATH` 是否指向包含 `robot_motion_sdk/__init__.py` 的目录。

如果 DDS topic 无法匹配，先保持两侧默认 topic：

- control: `rt/motion/control`
- observed: `rt/motion/observed`
- TRC: `motion/trc`
