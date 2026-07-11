# Mock Service 开发指南

这个包是一个自包含的 x86_64 Linux VM mock 运行环境。所有运行时文件都应部署在：

```text
/uniubi_mock
```

请将该包部署到 `/uniubi_mock` 下，避免污染 VM 全局的 `/vendor`、`/etc`、`/product` 和 `/data` 目录。

## 仓库结构

```text
mockService/
└── uniubi_mock/                 # 部署到 VM /uniubi_mock 的内容
    ├── vendor/x86_64/usr/bin/   # x86_64 可执行文件
    ├── vendor/x86_64/usr/lib/   # x86_64 动态库
    ├── etc/uos/                 # 服务配置，已使用 /uniubi_mock 路径
    │   └── robot_simulate_proxy  # 仿真 host DDS reader/writer 配置
    ├── etc/dds/                 # CycloneDDS 配置
    ├── product/mock/            # mock 产品配置
    ├── product/model/motion/    # 运控加密模型
    └── data/                    # 运行时数据根目录
```

部署完成后，VM 上的目录结构应为：

```text
/uniubi_mock/
├── vendor/usr/bin/
├── vendor/usr/lib/
├── etc/uos/
├── etc/dds/
├── product/mock/
├── product/model/motion/
└── data/
    ├── cache/
    ├── config/
    └── logger/log/
```

## 部署到 VM

```bash
export SIM_ROOT=/path/to/mockService/uniubi_mock
export PLATFORM=x86_64
export MOCK_ROOT=/uniubi_mock

sudo mkdir -p "$MOCK_ROOT/vendor/usr" "$MOCK_ROOT/etc" "$MOCK_ROOT/product" "$MOCK_ROOT/data/config" "$MOCK_ROOT/data/cache" "$MOCK_ROOT/data/logger/log"
sudo cp -a "$SIM_ROOT/vendor/$PLATFORM/usr/bin" "$MOCK_ROOT/vendor/usr/"
sudo cp -a "$SIM_ROOT/vendor/$PLATFORM/usr/lib" "$MOCK_ROOT/vendor/usr/"
sudo cp -a "$SIM_ROOT/etc/uos" "$MOCK_ROOT/etc/"
sudo cp -a "$SIM_ROOT/etc/dds" "$MOCK_ROOT/etc/"
sudo cp -a "$SIM_ROOT/product/mock" "$MOCK_ROOT/product/"
sudo mkdir -p "$MOCK_ROOT/product/model"
sudo cp -a "$SIM_ROOT/product/model/motion" "$MOCK_ROOT/product/model/"
sudo mkdir -p "$MOCK_ROOT/data/config" "$MOCK_ROOT/data/cache" "$MOCK_ROOT/data/logger/log"
```

`vendor` 按平台区分。当前包内容适用于 `x86_64`：

```text
uniubi_mock/vendor/
└── x86_64/usr/
    ├── bin/
    └── lib/
```

部署时请使用同一个平台目录下的 `bin` 和 `lib`。

## 运行时路径

服务配置已经写成 `/uniubi_mock` 路径：

| 组件 | 运行时路径 |
| --- | --- |
| 动态库 | `/uniubi_mock/vendor/usr/lib` |
| 可执行文件 | `/uniubi_mock/vendor/usr/bin` |
| UOS 配置 | `/uniubi_mock/etc/uos` |
| DDS 配置 | `/uniubi_mock/etc/dds` |
| 产品配置 | `/uniubi_mock/product/mock` |
| 运控模型 | `/uniubi_mock/product/model/motion` |
| 缓存 | `/uniubi_mock/data/cache` |
| 日志 | `/uniubi_mock/data/logger/log` |

相关 RobotService 路径配置如下：

| 运行时行为 | 配置方式 |
| --- | --- |
| `motionServer` 配置文件 | 启动参数：`/uniubi_mock/etc/uos/motionServer` |
| `robotServer` 配置文件 | 启动参数：`/uniubi_mock/etc/uos/robotServer` |
| `robotMonitorServer` 配置文件 | 启动参数：`-C /uniubi_mock/etc/uos/robotMonitor` |
| 仿真 host DDS reader/writer | `robotServerCapacity.simulateProxy.ddsConfig` 指向 `/uniubi_mock/etc/uos/robot_simulate_proxy` |
| DDS XML 路径 | UOS 配置中的 `dds.domain[].url` |
| 产品配置路径 | UOS 配置中的 `config.defConfigPath` |
| 模型路径 | `motionServer` 配置中的 `motion.modelDir` |
| 缓存路径 | `robotServer` 配置中的 `fileCache.path` |
| 日志路径 | `robotMonitor` 配置中的 `log.logPath` |

`robotMonitorServer` 提供日志支持。先启动并保持一个 `robotMonitorServer` 进程，再启动 `motionServer` 和 `robotServer`。

## 启动服务

```bash
export MOCK_ROOT=/uniubi_mock
export LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH}

sudo pkill -x robotMonitorServer || true
sudo pkill -x robotServer || true
sudo pkill -x motionServer || true

sudo env LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH} \
  $MOCK_ROOT/vendor/usr/bin/robotMonitorServer -C $MOCK_ROOT/etc/uos/robotMonitor &

sudo env LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH} \
  $MOCK_ROOT/vendor/usr/bin/motionServer $MOCK_ROOT/etc/uos/motionServer true &

sudo env LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH} \
  $MOCK_ROOT/vendor/usr/bin/robotServer $MOCK_ROOT/etc/uos/robotServer true &
```

运行时日志由 monitor/log 配置写入 `/uniubi_mock/data/logger/log`。

## Host DDS 网卡配置

Host 侧发现和 RobotServer RPC 使用 CycloneDDS host domain，配置文件为：

```text
/uniubi_mock/etc/dds/host_config.xml
```

`robotServer` 的仿真 host proxy 配置在：

```text
/uniubi_mock/etc/uos/robot_simulate_proxy
```

`robotServerCapacity.simulateProxy.interface` 控制 host domain 延迟绑定的网卡候选列表；mock 包默认包含 `enp1s0`、`eth0`、`wlan0`，需要至少有一个网卡在 VM 中存在且已获取 IPv4 地址。`robot_simulate_proxy` 内部的 DDS XML 路径必须保持为 `/uniubi_mock/etc/dds/host_config.xml`。

这个文件通过 `NetworkInterface name="..."` 依赖 VM 实际网卡名。第一次在新 VM 上启动服务前，先检查网卡名：

```bash
ip -br addr
```

如果 VM 使用的网卡名没有出现在 `host_config.xml` 中，只修改或新增对应的 `<NetworkInterface name="...">` 条目即可。常见 VM 网卡名包括 `enp1s0`、`ens33` 和 `eth0`。

除了这个 host DDS 网卡名适配项以外，不建议随意修改包内 UOS、DDS 或 product 配置。其他配置值与服务 domain、运行时路径、RPC/event topic 和 mock 产品能力绑定。

## 关键配置检查

`etc/uos/robotServer` 只预初始化本地 motion domain。仿真 host domain、`robotServer` RPC server 和 host EventBus 由 `robotServerCapacity.simulateProxy.ddsConfig` 指向的 `/uniubi_mock/etc/uos/robot_simulate_proxy` 在网卡稳定后延迟初始化。

`etc/uos/robot_simulate_proxy` 必须保持 host EventBus 双向启用：

```json
{
  "server": "robotServer",
  "domain": "host",
  "withService": true,
  "withClient": true
}
```

`withService=true` 用于接收 `robotServer.discoverDevice.request`。  
`withClient=true` 用于发布 `robotServer.discoverDevice.response`。

DDS domain：

| 配置 | Domain | 用途 |
| --- | --- | --- |
| `etc/dds/host_config.xml` | `42` | host 侧发现和 RobotServer RPC |
| `etc/dds/motion_config.xml` | `1` | 本地 motion 服务通信 |

## 校验命令

部署完成后，在目标 VM 上执行：

```bash
export MOCK_ROOT=/uniubi_mock

jq . $MOCK_ROOT/etc/uos/motionServer >/dev/null
jq . $MOCK_ROOT/etc/uos/robotServer >/dev/null
jq . $MOCK_ROOT/etc/uos/robotMonitor >/dev/null
jq . $MOCK_ROOT/etc/uos/robot_simulate_proxy >/dev/null
jq . $MOCK_ROOT/product/mock/motionConfig >/dev/null
jq . $MOCK_ROOT/product/mock/motionCapacity >/dev/null
jq . $MOCK_ROOT/product/mock/robotAppConfig >/dev/null
jq . $MOCK_ROOT/product/mock/robotServerCapacity >/dev/null

LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib ldd $MOCK_ROOT/vendor/usr/bin/motionServer
LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib ldd $MOCK_ROOT/vendor/usr/bin/robotServer

grep -n 'Domain Id' $MOCK_ROOT/etc/dds/host_config.xml
grep -n 'Domain Id' $MOCK_ROOT/etc/dds/motion_config.xml
grep -n 'NetworkInterface' $MOCK_ROOT/etc/dds/host_config.xml
```

## 问题排查

如果发现流程没有返回设备：

- 确认 `robotServer` 使用 `$MOCK_ROOT/etc/uos/robotServer` 启动。
- 确认 `robot_simulate_proxy` EventBus 配置了 `withService=true` 和 `withClient=true`。
- 确认 host client 和 `robotServer` 都使用 host domain `42`。
- 确认 `host_config.xml` 包含 VM 用于 host 侧发现的网卡名。
- 确认 `robotServerCapacity.simulateProxy.ddsConfig` 指向 `/uniubi_mock/etc/uos/robot_simulate_proxy`，且 `simulateProxy.interface` 包含 VM 实际联调网卡。
- 确认 VM 网络允许 multicast。

如果 `robotServer` 无法访问 `motionServer`：

- 确认先启动了 `motionServer`。
- 确认 `motion_config.xml` 使用 domain `1`。
- 确认 `lo` 已启用：

```bash
ip addr show lo
sudo ip link set lo up
```

如果 `motionServer` 被杀掉后需要重启，重启服务前先清理运行时内存配置：

```bash
sudo rm -f /tmp/memoryConfig
```

## 清理

如需从 VM 移除 mock 环境：

```bash
sudo pkill -x robotServer || true
sudo pkill -x motionServer || true
sudo pkill -x robotMonitorServer || true
sudo rm -rf /uniubi_mock
```

清理命令只会删除 mock 运行时根目录 `/uniubi_mock`；VM 全局的 `/vendor`、`/etc`、`/product` 和 `/data` 目录不属于这个包。
