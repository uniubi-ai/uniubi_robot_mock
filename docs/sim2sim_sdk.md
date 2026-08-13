# SDK Sim2Sim

**English** | [简体中文](sim2sim_sdk_zh.md)

The repository provides two independent validation paths:

| Interface | Mock services | MuJoCo bridge | Entry point |
|---|---|---|---|
| LowLevel | Not required | Required | [`simulation/scripts/run_lowlevel_onnx_policy.py`](../simulation/scripts/run_lowlevel_onnx_policy.py) |
| HighLevel | Required | Required | [`simulation/scripts/highlevel_console.py`](../simulation/scripts/highlevel_console.py) |

On x86_64 Linux, the LowLevel SDK selects its external-simulation backend. It
exchanges control and observation directly with the MuJoCo bridge over DDS, so
it does not use `robotMonitorServer`, `motionServer`, or `robotServer`.
HighLevel uses configured RobotService actions and therefore requires the mock
runtime and all three services.

## Install

### 1. Mock services (HighLevel only)

HighLevel requires the RobotService mock runtime. Deploy
`mockService/uniubi_mock/` to `/uniubi_mock` by following the
[Mock Service Development Guide](mock_service.md).

**Skip this step for LowLevel.** The x86 LowLevel SDK communicates directly
with the MuJoCo bridge and does not use any mock service process.

### 2. Shared simulation environment and SDK libraries

Both interfaces require the repository, simulation dependencies, and public
Python SDK. The following layout keeps the three repositories side by side:

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

Before running either CLI, expose the matching SDK native libraries:

```bash
export UNIUBI_SDK_ROOT=~/uniubi_robot_sdk
export LD_LIBRARY_PATH="$UNIUBI_SDK_ROOT/lib/$(uname -m):${LD_LIBRARY_PATH}"
```

After this common installation, LowLevel can be started immediately. HighLevel
also requires the mock service installation from step 1.

## LowLevel SDK Validation

### 1. Start the MuJoCo bridge

Terminal 1:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python -m sim2sim.robot2simulator.run_bridge \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --viewer
```

Use `--headless` instead of `--viewer` without a display. The robot starts and
remains in a stable laying pose before any LowLevel command arrives.

### 2. Start the LowLevel CLI

Terminal 2:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd)

python scripts/run_lowlevel_onnx_policy.py
```

When developing the Python binding directly from source instead of installing
it, pass `--sdk-python /path/containing/robot_motion_sdk`.

The CLI uses only the bundled
[`simulation/models/policy.onnx`](../simulation/models/policy.onnx): a 45-input,
12-output Cyvet velocity policy running at 50 Hz.

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

If `walk` is entered from a posture other than standing, the CLI first performs
the two-second standing transition. Laying and standing use the same per-joint
posture gains as the mock MotionServer configuration.

The LowLevel data path is:

```text
MotionLowLevelClient (external simulation backend)
  -> rt/motion/control
  -> MuJoCo bridge
  -> rt/motion/observed
  -> MotionLowLevelClient
```

## HighLevel SDK Validation

### 1. Start the mock services

After completing mock service installation step 1, start `robotMonitorServer`,
`motionServer`, and `robotServer` by following the
[Mock Service Development Guide](mock_service.md). The services must run with
`sudo` because MotionServer creates a real-time control thread.

### 2. Start the MuJoCo bridge

Start the same bridge shown in the LowLevel section. Do not start a second
bridge if one is already running.

### 3. Start the HighLevel CLI

Terminal 2:

```bash
cd /path/to/uniubi_robot_mock/simulation
export PYTHONPATH=$(pwd):$ROBOTSDK_PYTHON_PATH

python scripts/highlevel_console.py --iface <iface>
```

A typical Walking session is:

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
- `zero` clears parameters without stopping the current action.
- `start` means the request was accepted; use `state` to confirm the action
  that is actually executing.

`bipedStand`, `handstand`, `leftSideStand`, and `rightSideStand` are persistent
actions. Return from them with `stop`, wait until `state` reports `walking`, and
then send Walking parameters. Use `help` for the complete command list.

## DDS Interface Selection

When the host has multiple network interfaces:

```bash
cd /path/to/uniubi_robot_mock/simulation
source scripts/setup_dds.sh <iface>
```

Use `ip -br addr` to identify the interface carrying DDS traffic. HighLevel
also requires the same interface in the mock runtime host DDS configuration;
see [Mock Service Development Guide](mock_service.md#host-dds-网卡配置).

## Troubleshooting

- If `robot_motion_sdk` cannot be imported, verify that
  `ROBOTSDK_PYTHON_PATH` contains `robot_motion_sdk/__init__.py`.
- If LowLevel receives no observation, verify that the MuJoCo bridge and client
  use DDS domain `42` and matching `rt/motion/control` and
  `rt/motion/observed` topics. Mock services are not part of this path.
- If HighLevel RPC connects but motion does not run, verify that all three mock
  services were started with `sudo` and became ready before starting the
  HighLevel client.

The ONNXRuntime CLI is intended for x86 simulation and SDK integration checks,
not on-board policy deployment.
