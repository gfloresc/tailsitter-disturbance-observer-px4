#!/usr/bin/env bash
# Record the camera frames of the video runs (worlds ijc_*_video, headless rendering, real time).
#   record_videos.sh <S1|S2|hover|circle> [cases...]     cases: comp nocomp px4 (default: all three)
#   S1, S2: first version (follow camera); hover, circle: aggressive gusts (worlds from make_video_worlds.py)
# Output: logs/video/<S>_<case>.ulg, logs/video/frames_<S>_<case>/ (JPEG + frames.csv, simulation time)
# and, for hover/circle, logs/video/wind_<S>_<case>.csv (wind applied by IjcGustSystem)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
OUT=$ROOT/logs/video
NAME=${1:?S1 or S2}; shift
CASES=${*:-comp nocomp px4}
mkdir -p $OUT
# shellcheck disable=SC1091
source $S/gpu_env.sh   # offscreen camera rendering on the NVIDIA GPU
case $NAME in
S1) GUST=""; WORLD=ijc_calm_video; SCEN="--param IJC_DIST=1 --param IJC_AF=2.942 --param IJC_ATX=9.106 --param IJC_ATY=20.272 --param IJC_ATZ=7.003 --param IJC_TRAJ=0"; DUR=22 ;;
S2) GUST=""; WORLD=ijc_wind_video; SCEN="--param IJC_DIST=0 --param IJC_TRAJ=0"; DUR=62 ;;
hover) GUST="--gust-topic /ijc_gust_start"; WORLD=ijc_gust_hover_video; SCEN="--param IJC_DIST=0 --param IJC_TRAJ=0"; DUR=42 ;;
circle) GUST="--gust-topic /ijc_gust_start"; WORLD=ijc_gust_circle_video; SCEN="--param IJC_DIST=0 --param IJC_TRAJ=1 --param IJC_TRJ_R=2.0 --param IJC_TRJ_T=20.0 --param IJC_TRJ_TR=5.0"; DUR=42 ;;
*) echo "unknown scenario"; exit 1 ;;
esac
PX4T=$(cd $S && python3 -c "import tune; print(' '.join(f'--param {k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")

for c in $CASES; do
	case $c in
	comp)   AF=4050; P="--param IJC_COMP=1" ;;
	nocomp) AF=4050; P="--param IJC_COMP=0" ;;
	px4)    AF=4051; P="--param IJC_COMP=1 $PX4T" ;;
	esac
	$S/sitl.sh stop > /dev/null
	# Gazebo server with offscreen (EGL) rendering for the camera sensor; PX4 attaches to it
	( source $PX4/build/px4_sitl_default/rootfs/gz_env.sh && env -u DISPLAY gz sim -s -r --headless-rendering \
		$PX4/Tools/simulation/gz/worlds/$WORLD.sdf > $OUT/gz_${NAME}_$c.log 2>&1 & )
	sleep 5
	$S/sitl.sh start $WORLD $AF
	rm -rf $OUT/frames_${NAME}_$c
	python3 $S/record_camera.py $OUT/frames_${NAME}_$c &
	REC=$!
	python3 $S/ijc_flight.py --name ${NAME}_$c --outdir $OUT --simtime $GUST --settle 10 --duration $DUR $SCEN $P | grep -E "hover:|scenario done|disarmed|ULog"
	kill -TERM $REC; wait $REC || true
	[ -f $OUT/wind_$WORLD.csv ] && mv $OUT/wind_$WORLD.csv $OUT/wind_${NAME}_$c.csv
	$S/sitl.sh stop > /dev/null
	echo "$NAME $c: $(($(wc -l < $OUT/frames_${NAME}_$c/frames.csv) - 1)) frames"
done
