# SDK Sim2Sim

**English** | [简体中文](sim2sim_sdk_zh.md)

This guide covers two independent SDK validation paths against the same
RobotService mock runtime and MuJoCo bridge:

| Interface | Entry point | Control level |
|---|---|---|
| HighLevel | [`simulation/scripts/highlevel_console.py`](../simulation/scripts/highlevel_console.py) | Start and stop configured motion actions and update action parameters |
| LowLevel | [`simulation/scripts/run_lowlevel_onnx_policy.py`](../simulation/scripts/run_lowlevel_onnx_policy.py) | Run the bundled ONNX policy and send 12-joint PD targets |

The x86_64 Linux host running the mock runtime represents the robot body. The
SDK client may run on that host or on another DDS-reachable Linux host. The
commands below use the same host, which is the validated setup.

## Shared Setup

### 1. Start the mock services

Deploy and start `robotMonitorServer`, `motionServer`, and `robotServer` by
following [Mock Service Development Guide](mock_service.md). The services must
run with `sudo` because MotionServer creates a real-time control thread.

### 2. Install the simulation dependencies

```bash
python -m pip install -r simulation/requirements.txt
```

Set the directory containing the public Python SDK package:

```bash
export ROBOTSDK_PYTHON_PATH=/path/to/robotsdk/Sdk/Python
```

### 3. Start the MuJoCo bridge

In terminal 1:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --viewer
```

Use `--headless` instead of `--viewer` on a machine without a display. The
bridge publishes `rt/motion/observed` and subscribes to `rt/motion/control`.
The robot starts in a stable laying pose and receives no SDK control until a
client enables or starts motion.

### Optional: bind the DDS interface

When the host has multiple network interfaces:

```bash
cd /path/to/uniubi_robot_mock/simulation
source scripts/setup_dds.sh <iface>
```

Use `ip -br addr` to find the interface that carries DDS traffic.

## HighLevel SDK Validation

Use HighLevel when testing configured actions such as `walking`, `waveHand`, or
the biped-stand actions. It exercises action scheduling and action parameters;
it does not send joint targets directly.

In terminal 2:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd):$ROBOTSDK_PYTHON_PATH

python scripts/highlevel_console.py --iface <iface>
```

The console does not emulate remote-control button combinations. A typical
Walking session is:

```text
start walking
set {"lineVelocityX":0.5,"lineVelocityY":0,"velocity":0}
state
zero
stop
quit
```

- `set` keeps the parameters active.
- `send 3 {...}` applies parameters for three seconds and then clears them.
- `zero` clears action parameters without stopping the current action.
- `start` reports that the request was accepted; use `state` to confirm the
  action that is actually executing.

`bipedStand`, `handstand`, `leftSideStand`, and `rightSideStand` are persistent
actions. Return from them with `stop`, wait until `state` reports `walking`, and
then send Walking parameters:

```text
start bipedStand
stop
state
set {"lineVelocityX":0.5,"lineVelocityY":0,"velocity":0}
```

Use `help` for the complete HighLevel command list.

## LowLevel SDK Validation

Use LowLevel when testing the direct observation-to-joint-control loop. This
example supports only the bundled
[`simulation/models/policy.onnx`](../simulation/models/policy.onnx): a 45-input,
12-output Cyvet velocity policy running at 50 Hz.

In terminal 2:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)
export LD_LIBRARY_PATH=/uniubi_mock/vendor/usr/lib:$LD_LIBRARY_PATH

python scripts/run_lowlevel_onnx_policy.py \
  --sdk-python "$ROBOTSDK_PYTHON_PATH"
```

The client connects without sending joint commands. A typical session is:

```text
stand
walk 0.5 0 0
state
stop
lay
quit
```

| Command | Behavior |
|---|---|
| `stand` | Enable LowLevel control and smoothly move from the measured pose to standing in two seconds |
| `walk [VX VY YAW]` | Run the policy; defaults to `0.5 0 0` |
| `stop` | Stop inference and return to the standing target |
| `lay` | Smoothly return to the laying target |
| `state` | Show SDK state, tracked posture, command, and control-frame counters |
| `obs` | Show the latest 12 observed joint positions |
| `quit` | Send safety cleanup, disable LowLevel control, disconnect, and exit |

If `walk` is entered while the tracked posture is not standing, the CLI first
performs the same two-second standing transition and starts the policy only
afterward. Laying and standing use the same per-joint posture gains as the mock
MotionServer configuration.

This ONNXRuntime CLI is intended for x86 simulation and LowLevel SDK integration
checks, not on-board policy deployment.

## Troubleshooting

If `robot_motion_sdk` cannot be imported, verify that
`ROBOTSDK_PYTHON_PATH` points to the directory containing
`robot_motion_sdk/__init__.py`.

If DDS endpoints do not match, keep these defaults on both sides:

- control: `rt/motion/control`
- observed: `rt/motion/observed`
- TRC: `rt/motion/trc`

If RPC connects but motion does not run, confirm that all three mock services
were started with `sudo` and are ready before starting the MuJoCo bridge and SDK
client.
