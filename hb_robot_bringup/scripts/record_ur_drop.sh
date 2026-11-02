#!/usr/bin/env bash
# Run on the Jetson. Records only; never sends robot commands.
set -eu
seconds="${1:-120}"
case "$seconds" in ''|*[!0-9]*) echo 'Duration must be seconds'; exit 2;; esac
command -v tcpdump >/dev/null || { echo 'Install tcpdump on the Jetson first: sudo apt-get install tcpdump'; exit 1; }
robot_ip=192.168.50.55
iface=enP8p1s0
ip link show "$iface" >/dev/null
out="$HOME/ur-drop-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$out"
sudo -v
{ date -Is; uname -a; uptime; ip route get "$robot_ip"; ip -s link show "$iface"; ss -tinp; } > "$out/before.txt" 2>&1
ps -eLo pid,tid,cls,rtprio,psr,pcpu,stat,wchan:28,comm > "$out/threads-before.txt"
timeout "$seconds" top -b -H -d 1 > "$out/top-threads.txt" 2>&1 &
monitor_pid=$!
finish() {
    trap - EXIT
    kill "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
    { date -Is; ip -s link show "$iface"; ss -tinp; } > "$out/after.txt" 2>&1
    python3 - "$out" <<'PY'
from pathlib import Path
import shutil,sys
root=Path.home()/'.ros/log'
target=Path(sys.argv[1])/'ros-logs'
target.mkdir(exist_ok=True)
for prefix in ('ur_ros2_control_node','ros2_control_node','move_group','cut_profile_server','inspect_profile_server'):
    paths=sorted(root.glob(prefix+'*.log'),key=lambda p:p.stat().st_mtime,reverse=True)
    if paths: shutil.copy2(paths[0],target/paths[0].name)
launches=sorted(root.glob('*/launch.log'),key=lambda p:p.stat().st_mtime,reverse=True)
for i,p in enumerate(launches[:3]): shutil.copy2(p,target/f'launch-{i}.log')
PY
    tar -czf "$out.tar.gz" -C "$(dirname "$out")" "$(basename "$out")"
    printf '\nCapture saved: %s.tar.gz\n' "$out"
}
trap finish EXIT
printf 'Recording %s seconds to %s\nNo motion is initiated by this script.\n' "$seconds" "$out"
# All TCP with the robot: reverse control, RTDE, script and dashboard connections.
# Snaplen 128 preserves packet timing/flags and a small payload prefix.
set +e
sudo timeout --signal=INT "$seconds" tcpdump -i "$iface" -nn -s 128 -B 8192 -U -w "$out/robot-tcp.pcap" "host $robot_ip and tcp" 2> "$out/tcpdump-status.txt"
status=$?
set -e
if [ "$status" -ne 0 ] && [ "$status" -ne 124 ] && [ "$status" -ne 130 ]; then
    cat "$out/tcpdump-status.txt"
    exit "$status"
fi
