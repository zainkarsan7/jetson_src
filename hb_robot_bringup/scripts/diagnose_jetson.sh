#!/usr/bin/env bash
# Read-only snapshot. Does not start/restart nodes or send robot commands.
set -u
section() { printf '\n--- %s ---\n' "$1"; }
section 'Host and load'
date -Is
hostname
uname -a
uptime
free -h
section 'Network routes and interface counters'
ip route
ip -s link
section 'Highest CPU consumers'
ps -eo pid,ppid,pcpu,pmem,comm --sort=-pcpu | head -25
section 'Robot control threads and actual executable'
for pid in $(pgrep -f '^/[^ ]*/(ur_ros2_control_node|ros2_control_node)( |$)' || true); do
    readlink -f "/proc/$pid/exe"
    ps -L -p "$pid" -o pid,tid,cls,rtprio,pri,psr,pcpu,comm
    cat "/proc/$pid/limits"
done
section 'Camera executable and command line'
for pid in $(pgrep -f '^/[^ ]*/azure_kinect_node(_codex)?( |$)' || true); do
    readlink -f "/proc/$pid/exe"
    tr '\0' ' ' < "/proc/$pid/cmdline"
    printf '\n'
    # Only ROS transport configuration, not the full process environment.
    tr '\0' '\n' < "/proc/$pid/environ" |
        grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION|CYCLONEDDS_URI|FASTRTPS_DEFAULT_PROFILES_FILE|FASTDDS_DEFAULT_PROFILES_FILE)=' || true
done
section 'Recent control and camera log findings'
python3 - <<'PY'
from pathlib import Path
import re
root = Path.home()/'.ros/log'
pattern = re.compile(r'reverse interface|deactivate|tolerance|FIFO|real.time|priority|deadline|overrun|missed|timeout|capture|recover|failed', re.I)
for prefix in ('ur_ros2_control_node', 'ros2_control_node', 'azure_kinect_node'):
    files = sorted(root.glob(prefix+'*.log'), key=lambda p:p.stat().st_mtime, reverse=True)
    for p in files[:1]:
        print('\nLOG:', p)
        with p.open(errors='replace') as f:
            lines=[line.rstrip() for line in f if pattern.search(line)]
        print('\n'.join(lines[-60:]))
PY
section 'Local IMU probe (adds one subscriber for 10 seconds)'
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    printf 'Probe ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s\n' "${ROS_DOMAIN_ID:-0}" "${RMW_IMPLEMENTATION:-default}"
    echo 'Compare these with the camera process environment above before interpreting a missing stream.'
    timeout 10s stdbuf -oL ros2 topic hz /k4a/imu || true
else
    echo 'Humble setup not found; run ros2 topic hz /k4a/imu in the camera launch environment.'
fi
section 'Done'
