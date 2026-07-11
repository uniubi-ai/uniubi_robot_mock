# 机器人仿真环境配置

以下步骤均在 ubuntu 上进行。

## 仿真环境配置推荐

| 项目 | 推荐配置 | 说明 |
| --- | --- | --- |
| 操作系统 | Ubuntu 22.04 LTS | 其他版本未充分验证 |
| CPU | x86，≥ 8 核 | MuJoCo 物理计算依赖 CPU 多核 |
| 内存 | 32 GB | 同时跑 viewer + bridge + 录制时 16 GB 偏紧 |
| 磁盘 | 50 GB 可用 | conda 环境 + isaacgym 资源占用较大 |
| GPU | NVIDIA GPU + CUDA 11.x | 仅 mujoco 后端时可无独显，使用 OSMesa 软件渲染；isaacgym 必须 NVIDIA GPU |
| Python | mujoco_env 用 3.11；gym_env 用 3.8 | 两个环境独立，按后端切换 |
| 网络 | 仿真机与真机同一网段，千兆有线 | DDS 默认通过多播发现，跨网段需额外配置 |

> **备注**：配置过低时，可能出现以下问题：
> - 仿真画面卡顿、控制周期不稳，physics step 跟不上设定频率
> - bridge 与真机/控制端 数据丢包严重或通信延迟较大
>
> 实测在 i5-7400（4 核 @ 3.0 GHz）+ 16 GB + Intel HD Graphics 630 上运行 mujoco 以及设备服务，CPU 占用吃满，仅能勉强运行。

## 1. 系统依赖

无论使用哪种仿真后端，以下步骤都需要先完成。

### 1.1 安装 miniconda3
```bash
# 1. 打开终端并运行以下命令，下载最新版本的 Miniconda：
curl -O https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# 2. 运行以下命令安装 Miniconda，期间权限请求一路通过即可完成：
bash ./Miniconda3-latest-Linux-x86_64.sh

# 3. 将conda加入环境变量
source /root/miniconda3/bin/activate

# ps：官方流程详情请查阅
https://www.anaconda.com/docs/getting-started/miniconda/install/linux-install#how-do-i-verify-my-installers-integrity
```

### 1.2 编译 cyclonedds c 库

DDS 通信库，当前设备服务采用版本 cyclonedds = 0.10.5，仿真环境需要保证版本一致。

```bash
# 1. 克隆官方仓库
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds

# 2. 切换到与你要安装的 python 版本对应的分支/标签 (0.10.5)
git checkout 0.10.5

# 3. 创建构建目录
mkdir build install
cd build

# 4. 编译并安装到刚才创建的 install 目录 (不污染系统路径)
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DBUILD_EXAMPLES=OFF
cmake --build . --target install

# 5. 写入 CYCLONEDDS_HOME 到 ~/.bashrc，后续在 mujoco_env / gym_env 中安装 cyclonedds python 包时会自动用到
echo "export CYCLONEDDS_HOME=$(cd ../install && pwd)" >> ~/.bashrc
source ~/.bashrc
```

### 1.3 绑定 DDS 网卡（如遇到启动后设备交互，仿真器无响应情况）

仿真代码依赖 CycloneDDS 默认发现：自动挑第一个 up、非 loopback、支持多播的接口。多网卡环境（公司内网 + 实验台局域网、docker0、虚拟网卡等）下发现顺序不稳定，可能选错网卡导致 bridge 与真机不在同一广播域

解决方案
显式把 CycloneDDS 绑定到与真机连通的那张网卡。仓内提供脚本 `simulation/scripts/setup_dds.sh`，会把内联 XML 写到 `CYCLONEDDS_URI` 环境变量里，**仅对当前 shell 生效**，不污染 `~/.bashrc`、也不需要修改仓内文件。

```bash
# 1. 看一眼可选网卡，挑与真机同段的那张（比如 enp3s0）
ip -br addr

# 2. source 脚本绑定网卡（每开一个新 shell 都要重新 source 一次）
source simulation/scripts/setup_dds.sh enp3s0
```

不传参数时，脚本会自动列出 up、非 loopback 的候选网卡供你参考。

> 如果想要每个新 shell 自动生效，可以在 `~/.bashrc` 末尾加一行（用绝对路径），自行选择是否持久化：
>
> ```bash
> source /abs/path/to/simulation/scripts/setup_dds.sh enp3s0
> ```

## 2. 仿真器安装

当前支持两种仿真方式，按实际场景选择：

| 后端 | 对应 conda env | 关键区别 |
| --- | --- | --- |
| **mujoco**  | `mujoco_env`（Python 3.11） | CPU 物理 + OSMesa 软渲染，**无独显也能跑**；物理稳定、依赖轻，作为通用验证默认 |
| **isaacgym**  | `gym_env`（Python 3.8） | GPU 物理 + 渲染，**必须 NVIDIA GPU**；动作执行更贴近训练时的物理分布 |

> **本次发布的功能均在 mujoco 后端上完成测试**，推荐优先使用 mujoco 

### 2.1 方式一：mujoco

**安装**

```bash
# 系统依赖：OSMesa 软件渲染 + GLX/GLFW（viewer 用）+ patchelf（修 ELF）
sudo apt-get update
sudo apt-get install -y libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf

# 创建并激活环境 (可选，但在服务器上强烈推荐)
conda create -n mujoco_env python=3.11
conda activate mujoco_env

# 安装官方 MuJoCo 包
pip install mujoco numpy

# 配置环境变量，使用CPU
echo 'export MUJOCO_GL=osmesa' >> ~/.bashrc
source ~/.bashrc

# bridge 运行依赖（依赖 §1.2 装的 cyclonedds c 库-已在1.2 完成前置官方资源部署）
pip install cyclonedds==0.10.5 pyyaml
```


**启动**

```bash
# 进入仿真代码根目录
cd simulation
# 配置环境变量
export PYTHONPATH=$(pwd)
# 启动mujoco仿真器
PYTHONUNBUFFERED=1 python sim2sim/robot2simulator/run_bridge.py --config sim2sim/configs/uniubi_robot2sim.yaml --print-ctrl --print-ctrl-hz 10 --viewer --backend mujoco
```

### 2.2 方式二：isaacgym

- 下载安装包：https://developer.nvidia.com/isaac-gym/download

**安装**

```bash
# 创建并激活环境 (可选，但在服务器上强烈推荐)
conda create -n gym_env python=3.8
conda activate gym_env

# 安装 isaacgym（先 cd 到下载解压后的 isaacgym 目录，例如 IsaacGym_Preview_4_Package/isaacgym）
cd python
pip install -e .

# bridge 运行依赖（依赖 §1.2 装的 cyclonedds c 库-已在1.2 完成前置官方资源部署）
pip install cyclonedds==0.10.5 pyyaml
```


**启动**

```bash
# 进入仿真代码根目录
cd simulation
# 配置环境变量
export PYTHONPATH=$(pwd)
# 启动isaacgym仿真器
PYTHONUNBUFFERED=1 python sim2sim/robot2simulator/run_bridge.py --config sim2sim/configs/uniubi_robot2sim.yaml --print-ctrl --print-ctrl-hz 10 --viewer --backend isaacgym
```

## 3. 注意事项

- 仿真代码中配置都是硬编码默认值。**调整任何一项都要同步改设备服务**，两边对齐后再发版，请谨慎修改。
