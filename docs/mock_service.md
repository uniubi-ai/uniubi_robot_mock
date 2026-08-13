# Mock Service Development Guide

**English** | [简体中文](mock_service_zh.md)

This package is a self-contained x86_64 Linux mock runtime. It can run directly
on a physical host or, when needed, in a virtual machine. Deploy all runtime
files under:

```text
/uniubi_mock
```

Do not copy the package into the host's global `/vendor`, `/etc`, `/product`, or
`/data` directories.

## Repository Layout

```text
mockService/
└── uniubi_mock/                  # Deploy this directory as /uniubi_mock
    ├── vendor/x86_64/usr/bin/    # x86_64 executables
    ├── vendor/x86_64/usr/lib/    # x86_64 shared libraries
    ├── etc/uos/                  # Service configuration using /uniubi_mock paths
    │   └── robot_simulate_proxy  # Simulation host DDS reader/writer configuration
    ├── etc/dds/                  # Cyclone DDS configuration
    ├── product/mock/             # Mock product configuration
    ├── product/model/motion/     # Encrypted motion models
    └── data/                     # Runtime data root
```

The deployed host layout is:

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

## Deploy to an x86_64 Linux Host

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

The `vendor` directory is platform-specific. The current package targets
`x86_64`:

```text
uniubi_mock/vendor/
└── x86_64/usr/
    ├── bin/
    └── lib/
```

Always use `bin` and `lib` from the same platform directory.

## Runtime Paths

The service configuration already uses `/uniubi_mock` paths:

| Component | Runtime path |
|---|---|
| Shared libraries | `/uniubi_mock/vendor/usr/lib` |
| Executables | `/uniubi_mock/vendor/usr/bin` |
| UOS configuration | `/uniubi_mock/etc/uos` |
| DDS configuration | `/uniubi_mock/etc/dds` |
| Product configuration | `/uniubi_mock/product/mock` |
| Motion models | `/uniubi_mock/product/model/motion` |
| Cache | `/uniubi_mock/data/cache` |
| Logs | `/uniubi_mock/data/logger/log` |

Relevant RobotService path settings are:

| Runtime behavior | Configuration |
|---|---|
| `motionServer` configuration | Startup argument: `/uniubi_mock/etc/uos/motionServer` |
| `robotServer` configuration | Startup argument: `/uniubi_mock/etc/uos/robotServer` |
| `robotMonitorServer` configuration | Startup argument: `-C /uniubi_mock/etc/uos/robotMonitor` |
| Simulation host DDS reader/writer | `robotServerCapacity.simulateProxy.ddsConfig` points to `/uniubi_mock/etc/uos/robot_simulate_proxy` |
| DDS XML paths | `dds.domain[].url` in the UOS configuration |
| Product configuration path | `config.defConfigPath` in the UOS configuration |
| Model path | `motion.modelDir` in the `motionServer` configuration |
| Cache path | `fileCache.path` in the `robotServer` configuration |
| Log path | `log.logPath` in the `robotMonitor` configuration |

`robotMonitorServer` provides logging and supervises managed services. Start
and keep one `robotMonitorServer` process running before starting
`motionServer` and `robotServer`.

## Start the Services

```bash
export MOCK_ROOT=/uniubi_mock
export LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH}

# The monitor starts managed services, so stop it first during a restart.
sudo pkill -TERM -f "^$MOCK_ROOT/vendor/usr/bin/robotMonitorServer( |$)" || true
sleep 2
sudo pkill -KILL -f "^$MOCK_ROOT/vendor/usr/bin/robotMonitorServer( |$)" || true
sudo pkill -TERM -f "^$MOCK_ROOT/vendor/usr/bin/robotServer( |$)" || true
sudo pkill -TERM -f "^$MOCK_ROOT/vendor/usr/bin/motionServer( |$)" || true
sleep 2
sudo pkill -KILL -f "^$MOCK_ROOT/vendor/usr/bin/robotServer( |$)" || true
sudo pkill -KILL -f "^$MOCK_ROOT/vendor/usr/bin/motionServer( |$)" || true
sudo rm -f /tmp/memoryConfig /tmp/robot_monitor /tmp/roudiMonitor /tmp/roudiMonitor.lock

sudo env LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH} \
  $MOCK_ROOT/vendor/usr/bin/robotMonitorServer -C $MOCK_ROOT/etc/uos/robotMonitor &

sudo env LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH} \
  $MOCK_ROOT/vendor/usr/bin/motionServer $MOCK_ROOT/etc/uos/motionServer true &

sudo env LD_LIBRARY_PATH=$MOCK_ROOT/vendor/usr/lib:${LD_LIBRARY_PATH} \
  $MOCK_ROOT/vendor/usr/bin/robotServer $MOCK_ROOT/etc/uos/robotServer true &
```

All three services must run with `sudo`. `motionServer` creates real-time
scheduling threads. An unprivileged process may still accept RPC connections
while its control thread is not running. Wait until all three services are
ready before starting the MuJoCo bridge and SDK client.

The monitor and log configuration writes runtime logs to
`/uniubi_mock/data/logger/log`.

## Host DDS Network Interface Configuration

Host-side discovery and RobotServer RPC use the Cyclone DDS host domain:

```text
/uniubi_mock/etc/dds/host_config.xml
```

The `robotServer` simulation host proxy is configured in:

```text
/uniubi_mock/etc/uos/robot_simulate_proxy
```

`robotServerCapacity.simulateProxy.interface` specifies candidate interfaces
for delayed host-domain initialization. The mock package includes `enp1s0`,
`eth0`, and `wlan0` by default. At least one listed interface must exist on the
host and have an IPv4 address. The DDS XML path inside `robot_simulate_proxy`
must remain `/uniubi_mock/etc/dds/host_config.xml`.

The `<NetworkInterface name="...">` entry in `host_config.xml` must match the
host. Before the first startup on a new host, inspect available interfaces:

```bash
ip -br addr
```

If the active interface is absent, change or add only the corresponding
`<NetworkInterface name="...">` entry. Common names include `enp1s0`, `ens33`,
`eth0`, and `wlan0`.

Apart from this host-interface adaptation, do not change the bundled UOS, DDS,
or product configuration without validating the complete contract. Other
values are coupled to service domains, runtime paths, RPC and event topics, and
mock product capabilities.

## Critical Configuration Checks

`etc/uos/robotServer` initializes only the local motion domain. After the
network interface becomes stable, the host domain, `robotServer` RPC server,
and host EventBus are initialized from
`/uniubi_mock/etc/uos/robot_simulate_proxy`, the file referenced by
`robotServerCapacity.simulateProxy.ddsConfig`.

The host EventBus must remain bidirectional:

```json
{
  "server": "robotServer",
  "domain": "host",
  "withService": true,
  "withClient": true
}
```

- `withService=true` receives `robotServer.discoverDevice.request`.
- `withClient=true` publishes `robotServer.discoverDevice.response`.

DDS domains:

| Configuration | Domain | Purpose |
|---|---:|---|
| `etc/dds/host_config.xml` | `42` | Host discovery and RobotServer RPC |
| `etc/dds/motion_config.xml` | `1` | Local motion-service communication |

## Validation Commands

Run these commands on the target host after deployment:

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

## Troubleshooting

If discovery does not return a device:

- Confirm that `robotServer` uses `$MOCK_ROOT/etc/uos/robotServer`.
- Confirm that the `robot_simulate_proxy` EventBus sets both
  `withService=true` and `withClient=true`.
- Confirm that the host client and `robotServer` both use host domain `42`.
- Confirm that `host_config.xml` contains the host interface used for discovery.
- Confirm that `robotServerCapacity.simulateProxy.ddsConfig` points to
  `/uniubi_mock/etc/uos/robot_simulate_proxy` and that
  `simulateProxy.interface` contains the active host interface.
- Confirm that the host network permits multicast traffic.

If `robotServer` cannot reach `motionServer`:

- Start `motionServer` first.
- Confirm that `motion_config.xml` uses domain `1`.
- Confirm that loopback is enabled:

```bash
ip addr show lo
sudo ip link set lo up
```

If `motionServer` was killed and must be restarted, clear the runtime memory
configuration first:

```bash
sudo rm -f /tmp/memoryConfig
```

## Remove the Mock Runtime

```bash
sudo pkill -TERM -f '^/uniubi_mock/vendor/usr/bin/robotMonitorServer( |$)' || true
sleep 2
sudo pkill -KILL -f '^/uniubi_mock/vendor/usr/bin/robotMonitorServer( |$)' || true
sudo pkill -TERM -f '^/uniubi_mock/vendor/usr/bin/robotServer( |$)' || true
sudo pkill -TERM -f '^/uniubi_mock/vendor/usr/bin/motionServer( |$)' || true
sleep 2
sudo pkill -KILL -f '^/uniubi_mock/vendor/usr/bin/robotServer( |$)' || true
sudo pkill -KILL -f '^/uniubi_mock/vendor/usr/bin/motionServer( |$)' || true
sudo rm -rf /uniubi_mock
```

The cleanup removes only `/uniubi_mock`. It does not remove or modify the
host's global `/vendor`, `/etc`, `/product`, or `/data` directories.
