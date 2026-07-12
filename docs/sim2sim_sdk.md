# SDK Sim2Sim

This workflow connects a UniUbi low-level SDK client to a MuJoCo simulation through DDS topics. It is useful for validating SDK-side low-level control code before running on hardware.

Data flow:

```text
ONNX policy or SDK client
  -> MotionLowLevelClient(simulation)
  -> rt/motion/control
  -> MuJoCo bridge
  -> rt/motion/observed
  -> MotionLowLevelClient(simulation)
```

## Install

Install the simulation dependencies in your Python environment:

```bash
python -m pip install -r simulation/requirements.txt
```

The policy client also needs the UniUbi Python SDK package. Pass the directory that contains `robot_motion_sdk`, or set it once:

```bash
export ROBOTSDK_PYTHON_PATH=/path/to/robotsdk/Sdk/Python
```

## Start MuJoCo Bridge

Terminal 1:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --backend mujoco \
  --viewer
```

For headless machines:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)
export MUJOCO_GL=osmesa

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --backend mujoco \
  --headless
```

The bridge publishes `rt/motion/observed` and subscribes to `rt/motion/control`.

## Run ONNX Policy Client

Terminal 2:

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

The helper builds the same 45-dimensional observation used by the Cyvet velocity policy and sends joint position targets through `MotionLowLevelClient`.

For on-board deployment, use a TensorRT engine for policy inference. The ONNXRuntime helper above is intended for x86 simulation and SDK integration checks.

## Optional DDS Interface Binding

When multiple network interfaces are available, bind CycloneDDS in the current shell:

```bash
cd /path/to/uniubi_robot_mock/simulation
source scripts/setup_dds.sh <iface>
```

Use `ip -br addr` to find the interface that should carry DDS traffic.

## Optional Virtual Remote Control

The ONNX helper uses `--cmd-x/--cmd-y/--cmd-yaw` by default. This is the normal path for low-level simulation. It can also read `rt/motion/trc` frames if you want to drive the command with a virtual remote control:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python scripts/publish_trc_keyboard.py \
  --domain 42 \
  --topic rt/motion/trc \
  --rate 50
```

The local simulation uses action id `1` by default. You can override it with `--controller <id>` if needed.

Keyboard mapping:

- `w/s`: forward/backward
- `a/d`: lateral
- `q/e`: yaw
- `space` or `x`: zero axes and buttons
- `1`: handstand (`LB+A`)
- `2`: standing (`Back`)
- `3`: walking (`Start+Y`)
- `4`: laying (`Start+A`)
- `5`: waveBody (`LB+Start`)
- `z`: emergencyStop (`LB+RB`)

## Troubleshooting

If `robot_motion_sdk` cannot be imported, check that `ROBOTSDK_PYTHON_PATH` points to the directory containing `robot_motion_sdk/__init__.py`.

If DDS topics do not match, keep the defaults on both sides:

- control: `rt/motion/control`
- observed: `rt/motion/observed`
- TRC: `rt/motion/trc`
