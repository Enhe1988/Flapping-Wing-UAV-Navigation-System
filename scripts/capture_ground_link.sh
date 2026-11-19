#!/usr/bin/env bash
# Read-only capture on the ground-link interface; run locally for sudo's prompt.
set -e
capture_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
capture_interface="${1:-any}"
capture_log="$capture_root/log/ground-link-capture.txt"
capture_filter='arp or udp'
if [[ $capture_interface == --other-ports ]]; then
  capture_interface=any
  capture_log="$capture_root/log/ground-other-ports-capture.txt"
  capture_filter='net 192.168.1.0/24 and (udp or tcp) and not port 9000'
fi
mkdir -p "$capture_root/log"
if [[ $capture_interface != any && ! -d "/sys/class/net/$capture_interface" ]]; then
  echo "网卡不存在：$capture_interface" >&2
  exit 1
fi
if [[ ! -x /usr/sbin/tcpdump || ! -x /usr/bin/sudo ]]; then
  echo '需要本机安装 tcpdump 和 sudo' >&2
  exit 1
fi
# Authenticate in the user's terminal, not in the capture log.
/usr/bin/sudo -v
echo "正在只读抓取 $capture_interface，过滤：$capture_filter；约 20 秒，不发送数据。"
set +e
/usr/bin/sudo -n /usr/bin/timeout --signal=INT 20 /usr/sbin/tcpdump \
  -ni "$capture_interface" -nn -s 256 -l "$capture_filter" > "$capture_log" 2>&1
capture_status=$?
set -e
cat "$capture_log"
if [[ $capture_status != 0 && $capture_status != 124 ]]; then
  echo "抓包失败，退出码：$capture_status" >&2
  exit "$capture_status"
fi
echo "抓包已保存：$capture_log"
