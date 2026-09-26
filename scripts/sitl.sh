#!/usr/bin/env bash
# Start/stop the PX4 SITL + Gazebo (headless) for the ijc_control experiments.
#   sitl.sh start [world] [autostart]   (defaults: ijc_calm, 4050 = ijc_control;
#                                        4051 = standard PX4 controllers, same vehicle)
#   sitl.sh stop
set -u
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
LOG=$ROOT/logs/sitl_console.log

case "${1:-}" in
start)
	mkdir -p "$(dirname "$LOG")"
	cd "$PX4" || exit 1
	export HEADLESS=1
	export PX4_GZ_WORLD="${2:-ijc_calm}"
	export PX4_SYS_AUTOSTART="${3:-4050}"
	# faster than real time (gz real_time_factor); PX4 follows the simulation clock
	export PX4_SIM_SPEED_FACTOR="${SIM_SPEED:-1}"
	# start from the airframe defaults: drop parameters saved by previous runs
	rm -f "$PX4"/build/px4_sitl_default/rootfs/parameters*.bson
	# 'sleep infinity' keeps the pxh shell's stdin open
	nohup bash -c "sleep infinity | make px4_sitl gz_quadtailsitter_ijc" > "$LOG" 2>&1 &
	for _ in $(seq 1 60); do
		if grep -q "Ready for takeoff" "$LOG" 2>/dev/null; then
			echo "SITL ready (world ${PX4_GZ_WORLD}, airframe ${PX4_SYS_AUTOSTART})"
			exit 0
		fi
		if grep -q -E "ERROR \[init\]|gz_bridge failed" "$LOG" 2>/dev/null; then
			echo "SITL failed to start, see $LOG"
			exit 1
		fi
		sleep 2
	done
	echo "timeout waiting for SITL, see $LOG"
	exit 1
	;;
stop)
	pkill -INT -f "px4_sitl_default/bin/px4" 2>/dev/null
	sleep 2
	pkill -f "make px4_sitl gz_quadtailsitter_ijc" 2>/dev/null
	pkill -f "sleep infinity" 2>/dev/null
	pkill -f '^(ruby [^ ]*)?gz sim' 2>/dev/null
	sleep 1
	pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
	pkill -9 -f '^(ruby [^ ]*)?gz sim' 2>/dev/null
	echo "SITL stopped"
	;;
*)
	echo "usage: $0 start [world] | stop"
	exit 1
	;;
esac
