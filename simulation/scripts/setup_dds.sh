#!/usr/bin/env bash
# 把 CYCLONEDDS_URI 绑定到指定网卡，仅对当前 shell 生效，不写 ~/.bashrc。
# 用法：
#   source simulation/scripts/setup_dds.sh <iface>
# 例：
#   source simulation/scripts/setup_dds.sh enp3s0
# 不传参数时打印可选网卡列表。

if ! (return 0 2>/dev/null); then
  echo "[setup_dds] 必须用 source 运行，否则环境变量不会留在当前 shell：" >&2
  echo "  source simulation/scripts/setup_dds.sh <iface>" >&2
  exit 1
fi

_setup_dds() {
  local iface="$1"
  if [ -z "$iface" ]; then
    echo "[setup_dds] 用法: source simulation/scripts/setup_dds.sh <iface>"
    echo "可选网卡（up、非 loopback）："
    if command -v ip >/dev/null 2>&1; then
      ip -br link show up | awk '$1!="lo" {print "  " $1}'
      echo "用 'ip -br addr' 查看每张卡的 IP，挑与真机同段的那张。"
    else
      echo "  （未找到 'ip' 命令，请用 ifconfig 等工具自行查询）"
    fi
    return 1
  fi
  export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${iface}\"/></Interfaces></General></Domain></CycloneDDS>"
  echo "[setup_dds] CYCLONEDDS_URI 已绑定网卡: ${iface}（仅当前 shell 有效）"
}

_setup_dds "$@"
_setup_dds_rc=$?
unset -f _setup_dds
return $_setup_dds_rc 2>/dev/null || true
