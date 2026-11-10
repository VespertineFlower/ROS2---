#!/usr/bin/env bash
# Stop every process/window started by this project (Gazebo, RViz, nodes).
# Usage:  bash tools/stop_all.sh
set -u

for name in gzserver gzclient rviz2; do
    pkill -9 -x "$name" 2>/dev/null && echo "stopped $name"
done

# ros2 launch does not always take the node processes down with it, so scan
# /proc for the python executables installed by these packages.
killed=0
for pid in /proc/[0-9]*; do
    pid=${pid#/proc/}
    cmd=$( (tr '\0' ' ' < "/proc/$pid/cmdline") 2>/dev/null ) || continue
    case "$cmd" in
        /usr/bin/python3*lib/maze_control/*|/usr/bin/python3*lib/maze_bringup/*)
            kill -9 "$pid" 2>/dev/null && killed=$((killed + 1))
            ;;
    esac
done
echo "stopped $killed robot node(s)"

sleep 1
left="$(pgrep -x gzserver; pgrep -x rviz2)"
if [ -n "${left:-}" ]; then
    echo "still running: $left"
else
    echo "all clean"
fi
