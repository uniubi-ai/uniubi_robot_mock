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
  --viewer
```

For headless machines:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
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

## HighLevel Interactive Console

HighLevel mock control uses the same command-oriented console as the public Python SDK. It does not emulate remote-control key combinations. Install the current Python SDK first, start the MuJoCo bridge, then run:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python scripts/highlevel_console.py --iface <iface>
```

The x86 host running the runtime is the mock device; no discovery or device
selection is required.

Use the following sequence to validate Walking:

```text
start walking
set {"lineVelocityX":0.5,"lineVelocityY":0,"velocity":0}
zero
set {"lineVelocityX":-1.0,"lineVelocityY":0,"velocity":0}
zero
state
```

`set` keeps a command active, while `send 3 {...}` clears it automatically
after three seconds. `zero` clears the current parameters without stopping the
action. A successful `start` response means that the switch request was
accepted; use `state` to confirm the action that is actually executing.

`bipedStand`, `handstand`, `leftSideStand`, and `rightSideStand` are persistent
actions. To return from one of them to Walking, call `stop`, wait until `state`
reports `walking`, and only then send Walking velocity parameters:

```text
start bipedStand
stop
state
set {"lineVelocityX":-1.0,"lineVelocityY":0,"velocity":0}
```

Do not treat an accepted `start walking` RPC as proof that a persistent action
has already exited. Use `help` for the complete command list. LowLevel ONNX
testing continues to use `--cmd-x/--cmd-y/--cmd-yaw` and is unaffected.

## Troubleshooting

If `robot_motion_sdk` cannot be imported, check that `ROBOTSDK_PYTHON_PATH` points to the directory containing `robot_motion_sdk/__init__.py`.

If DDS topics do not match, keep the defaults on both sides:

- control: `rt/motion/control`
- observed: `rt/motion/observed`
- TRC: `rt/motion/trc`

Start all three mock services with `sudo` as described in
`docs/mock_service.md`. Without real-time scheduling permission, MotionServer
may fail to create its control thread. Start the MuJoCo bridge after the three
services are ready.
