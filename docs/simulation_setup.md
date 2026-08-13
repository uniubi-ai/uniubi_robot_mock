# Robot Simulation Environment Setup

**English** | [简体中文](simulation_setup_zh.md)

Perform the following steps on Ubuntu.

## Recommended Simulation Environment

| Item | Recommended configuration | Notes |
|---|---|---|
| Operating system | Ubuntu 22.04 LTS | Other versions have not been fully validated |
| CPU | x86, 8 cores or more | MuJoCo physics benefits from multiple CPU cores |
| Memory | 32 GB | 16 GB can be tight when running the viewer, bridge, and recording together |
| Disk | 20 GB free | For the Conda environment, MuJoCo, and runtime logs |
| GPU | Optional | OSMesa software rendering can be used without a discrete GPU |
| Python | 3.11 | Use a dedicated `mujoco_env` environment |
| Network | Gigabit wired connection on the same subnet as the robot | DDS uses multicast discovery by default; routing across subnets requires additional configuration |

> **Note:** An undersized system may show a low simulation frame rate, unstable
> control periods, physics steps that cannot keep up with the configured rate,
> or significant packet loss and latency between the bridge and the robot or
> controller.
>
> MuJoCo and the device services were barely usable on a tested i5-7400
> (4 cores at 3.0 GHz), 16 GB of memory, and Intel HD Graphics 630 because CPU
> utilization remained saturated.

## 1. System Dependencies

These steps install the system and DDS dependencies required by the MuJoCo bridge.

### 1.1 Install Miniconda

```bash
# Download the current Miniconda installer.
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# Run the installer and follow its prompts.
bash ./Miniconda3-latest-Linux-x86_64.sh

# Activate Conda. Adjust the path if Miniconda was installed elsewhere.
source /root/miniconda3/bin/activate
```

See the [official Miniconda installation guide](https://www.anaconda.com/docs/getting-started/miniconda/install/linux-install#how-do-i-verify-my-installers-integrity) for installer verification and installation details.

### 1.2 Build the Cyclone DDS C Library

The device services currently use Cyclone DDS 0.10.5. Use the same version in
the simulation environment.

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
git checkout 0.10.5

mkdir build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXAMPLES=OFF
cmake --build . --target install

echo "export CYCLONEDDS_HOME=$(cd ../install && pwd)" >> ~/.bashrc
source ~/.bashrc
```

### 1.3 Bind DDS to a Network Interface

Cyclone DDS normally selects the first active, non-loopback, multicast-capable
interface. On hosts with multiple interfaces, such as a corporate network,
test-bench LAN, Docker bridge, or virtual adapter, this selection may be
unstable and prevent the bridge from reaching the intended DDS domain.

Use `simulation/scripts/setup_dds.sh` to bind Cyclone DDS in the current shell
without modifying repository files or `~/.bashrc`:

```bash
ip -br addr
source simulation/scripts/setup_dds.sh enp3s0
```

Without an argument, the script lists active non-loopback candidates. To apply
the selection automatically in new shells, add an absolute-path invocation to
`~/.bashrc` only after confirming the correct interface:

```bash
source /abs/path/to/simulation/scripts/setup_dds.sh enp3s0
```

## 2. Install the Simulator

MuJoCo is the only supported simulator. The bridge uses CPU physics and can use
OSMesa for headless software rendering.

### 2.1 Install MuJoCo

```bash
sudo apt-get update
sudo apt-get install -y libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf

conda create -n mujoco_env python=3.11
conda activate mujoco_env

pip install mujoco numpy
pip install cyclonedds==0.10.5 pyyaml
```

The Cyclone DDS Python package uses the C library installed in section 1.2.

Start the MuJoCo bridge from the repository root:

```bash
cd simulation
export PYTHONPATH=$(pwd)
PYTHONUNBUFFERED=1 python sim2sim/robot2simulator/run_bridge.py \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --print-ctrl --print-ctrl-hz 10 --viewer
```

## 3. Configuration Notes

The simulation contains default configuration values that must remain
compatible with the service-side contract. When changing a shared DDS,
observation, control, or model setting, verify both sides together before
publishing the change.
