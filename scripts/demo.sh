#!/usr/bin/env bash
# Interactive demo with the Gazebo GUI: takeoff, scenario, landing; the window stays open until you close it.
#
#   demo.sh <hover|circle> <comp|nocomp|px4> <calm|paper|gusty> [duration_s]
#
#   scenario    hover: hold the takeoff point (2.5 m); circle: R = 2 m, period 20 s, 2.5 m (S3 reference)
#   controller  comp: ijc_control with disturbance compensation; nocomp: ijc_control without it;
#               px4: standard PX4 controllers with the tuned gains (same vehicle, same reference)
#   wind        calm: none; paper: wind of S2/S3 (mean 4 m/s, gusts 3-5 m/s, turbulence);
#               gusty: gust fronts of the videos (fronts_sharp x 0.85)
#   duration_s  scenario duration (default 40 s = two laps of the circle)
#
# Example: ./scripts/demo.sh circle comp gusty
# Prints the position error live (ground truth and EKF) and its RMS at the end of the scenario.
# Close the Gazebo window (or press Ctrl+C) to stop everything. Logs: logs/demo/ (ULog, consoles).
set -u
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
LOG=$ROOT/logs/demo
WORLD=ijc_demo

usage() { sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
[ $# -ge 3 ] || usage
SCEN=$1; CTRL=$2; WIND=$3; DUR=${4:-40}
case $SCEN in
hover)  SCEN_P="--param IJC_DIST=0 --param IJC_TRAJ=0" ;;
circle) SCEN_P="--param IJC_DIST=0 --param IJC_TRAJ=1 --param IJC_TRJ_R=2.0 --param IJC_TRJ_T=20.0 --param IJC_TRJ_TR=5.0" ;;
*) usage ;;
esac
case $CTRL in
comp)   AF=4050; CTRL_P="--param IJC_COMP=1" ;;
nocomp) AF=4050; CTRL_P="--param IJC_COMP=0" ;;
px4)    AF=4051; CTRL_P="--param IJC_COMP=1 $(cd $S && python3 -c "import tune; print(' '.join(f'--param {k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")" ;;
*) usage ;;
esac
case $WIND in calm|paper|gusty) ;; *) usage ;; esac
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
	echo "No graphical display (DISPLAY/WAYLAND_DISPLAY): run it from a terminal of the desktop session."
	exit 1
fi
mkdir -p $LOG

cleanup() {
	pkill -f "ijc_flight.py --name dem[o]" 2>/dev/null
	$S/sitl.sh stop > /dev/null 2>&1   # PX4 and every 'gz sim' (server and GUI)
}

echo "== cleaning previous simulations"
cleanup
trap 'echo; echo "== stopping"; cleanup; exit 130' INT TERM

echo "== world: $SCEN, wind $WIND"
python3 $S/make_demo_world.py $SCEN $WIND || exit 1

# shellcheck disable=SC1091
source $S/gpu_env.sh   # GUI on the NVIDIA GPU (PRIME offload) when its driver is loaded
echo "== starting Gazebo (server + GUI; GPU: $GPU_NAME)"
# shellcheck disable=SC1091
set +u; source $PX4/build/px4_sitl_default/rootfs/gz_env.sh; set -u
gz sim -s -r $PX4/Tools/simulation/gz/worlds/$WORLD.sdf > $LOG/gz_server.log 2>&1 &
for _ in $(seq 1 60); do
	gz service -i --service /world/$WORLD/scene/info 2>/dev/null | grep -q "Service providers" && break
	sleep 1
done
gz sim -g --gui-config $LOG/gui.config > $LOG/gz_gui.log 2>&1 &
GUI=$!

echo "== starting PX4 (airframe $AF, controller $CTRL)"
if ! $S/sitl.sh start $WORLD $AF; then
	cleanup; exit 1
fi
cp $ROOT/logs/sitl_console.log $LOG/px4_console.log 2>/dev/null
# OpenGL renderer actually used by the GUI (Ogre log)
echo "== GUI renderer: $(grep -h 'GL_RENDERER' ~/.gz/rendering/ogre2.log 2>/dev/null | tail -1 | sed 's/.*GL_RENDERER = //')"

echo "== flight: takeoff (5 s) + settle (10 s), scenario ($DUR s), landing"
# the wind trigger also times the desired-position sphere of the circle
GUST=""; { [ $WIND = gusty ] || [ $SCEN = circle ]; } && GUST="--gust-topic /ijc_gust_start"
python3 -u $S/ijc_flight.py --name demo_${SCEN}_${CTRL}_${WIND} --outdir $LOG --simtime --live --world $WORLD \
	$GUST --settle 10 --duration $DUR $SCEN_P $CTRL_P \
	| grep --line-buffered -v -E "^  [A-Z_0-9]+ = |pre-arm|offboard mode|^arm:|^connected" &
FLIGHT=$!
while kill -0 $FLIGHT 2>/dev/null; do
	if ! kill -0 $GUI 2>/dev/null; then   # window closed during the flight
		echo; echo "== Gazebo window closed: stopping"
		cleanup; exit 0
	fi
	sleep 1
done

echo
echo "== done. The simulation keeps running: close the Gazebo window (or Ctrl+C) to stop everything."
while kill -0 $GUI 2>/dev/null; do sleep 1; done
cleanup
echo "== stopped"
