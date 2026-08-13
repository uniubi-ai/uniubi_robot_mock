# 机器人仿真环境配置

[English](simulation_setup.md) | **简体中文**

以下步骤均在 Ubuntu 上执行。

## 推荐仿真环境

| 项目 | 推荐配置 | 说明 |
|---|---|---|
| 操作系统 | Ubuntu 22.04 LTS | 其他版本尚未充分验证 |
| CPU | x86，8 核或更多 | MuJoCo 物理计算可利用多核 CPU |
| 内存 | 32 GB | 同时运行 viewer、bridge 和录制时，16 GB 可能不足 |
| 磁盘 | 20 GB 可用空间 | 用于 Conda 环境、MuJoCo 和运行日志 |
| GPU | 可选 | 没有独立显卡时可使用 OSMesa 软件渲染 |
| Python | 3.11 | 建议使用独立的 `mujoco_env` 环境 |
| 网络 | 与机器人同网段的千兆有线网络 | DDS 默认使用多播发现；跨网段需要额外配置 |

> **说明：**配置不足时，可能出现仿真画面卡顿、控制周期不稳定、physics step 无法
> 跟上设定频率，或者 bridge 与机器人/控制端之间丢包和延迟明显等问题。
>
> 已测试的 i5-7400（4 核 @ 3.0 GHz）、16 GB 内存和 Intel HD Graphics 630 环境在
> 同时运行 MuJoCo 和设备服务时 CPU 长时间满载，只能勉强运行。

## 1. 系统依赖

以下步骤用于安装 MuJoCo bridge 所需的系统依赖和 DDS 依赖。

### 1.1 安装 Miniconda

```bash
# 下载当前 Miniconda 安装程序。
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# 运行安装程序并按照提示完成安装。
bash ./Miniconda3-latest-Linux-x86_64.sh

# 激活 Conda；如果安装到了其他目录，请调整路径。
source /root/miniconda3/bin/activate
```

安装程序校验和详细步骤见 [Miniconda 官方安装指南](https://www.anaconda.com/docs/getting-started/miniconda/install/linux-install#how-do-i-verify-my-installers-integrity)。

### 1.2 编译 Cyclone DDS C 库

当前设备服务使用 Cyclone DDS 0.10.5，仿真环境应使用相同版本。

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

### 1.3 绑定 DDS 网卡

Cyclone DDS 通常选择第一个处于启用状态、非 loopback 且支持多播的网卡。在公司内网、
实验台局域网、Docker 网桥或虚拟网卡并存的主机上，自动选择可能不稳定，导致 bridge
无法进入预期的 DDS 网络。

使用 `simulation/scripts/setup_dds.sh` 只在当前 shell 中绑定网卡，不修改仓库文件或
`~/.bashrc`：

```bash
ip -br addr
source simulation/scripts/setup_dds.sh enp3s0
```

不传参数时，脚本会列出处于启用状态的非 loopback 候选网卡。确认网卡无误后，如果
希望新 shell 自动生效，可以将绝对路径调用写入 `~/.bashrc`：

```bash
source /abs/path/to/simulation/scripts/setup_dds.sh enp3s0
```

## 2. 安装仿真器

当前只支持 MuJoCo。bridge 使用 CPU 进行物理计算，并支持通过 OSMesa 进行无界面软件
渲染。

### 2.1 安装 MuJoCo

```bash
sudo apt-get update
sudo apt-get install -y libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf

conda create -n mujoco_env python=3.11
conda activate mujoco_env

pip install mujoco numpy
pip install cyclonedds==0.10.5 pyyaml
```

Cyclone DDS Python 包会使用第 1.2 节安装的 C 库。

从仓库根目录启动 MuJoCo bridge：

```bash
cd simulation
export PYTHONPATH=$(pwd)
PYTHONUNBUFFERED=1 python sim2sim/robot2simulator/run_bridge.py \
  --config sim2sim/configs/uniubi_cyvet.yaml \
  --print-ctrl --print-ctrl-hz 10 --viewer
```

## 3. 配置说明

仿真中包含必须与服务端契约保持兼容的默认配置。修改双方共享的 DDS、observation、
控制或模型配置时，必须同时验证两端，再发布变更。
