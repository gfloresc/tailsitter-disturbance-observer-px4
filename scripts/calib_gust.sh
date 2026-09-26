#!/usr/bin/env bash
# Calibrate the aggressive gust profile of the demonstration videos (world ijc_gust, no camera, x4).
#   calib_gust.sh <profile> <scale> [hover|circle ...]
# Regenerates the worlds with the given disturbance profile and scale (make_video_worlds.py), flies the
# three video cases and prints the fraction of time with unallocated (saturated) thrust/torque and the
# ground-truth RMS position error.
set -e
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
OUT=$ROOT/logs/video/calib
PROF=${1:?profile}; SCALE=${2:?scale}; shift 2
KINDS=${*:-hover circle}
mkdir -p $OUT
python3 $S/make_video_worlds.py $PROF $SCALE > /dev/null
PX4T=$(cd $S && python3 -c "import tune; print(' '.join(f'--param {k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")
for k in $KINDS; do
	case $k in
	hover)  SCEN="--param IJC_DIST=0 --param IJC_TRAJ=0" ;;
	circle) SCEN="--param IJC_DIST=0 --param IJC_TRAJ=1 --param IJC_TRJ_R=2.0 --param IJC_TRJ_T=20.0 --param IJC_TRJ_TR=5.0" ;;
	esac
	for c in comp nocomp px4; do
		case $c in
		comp)   AF=4050; P="--param IJC_COMP=1" ;;
		nocomp) AF=4050; P="--param IJC_COMP=0" ;;
		px4)    AF=4051; P="--param IJC_COMP=1 $PX4T" ;;
		esac
		N=${k}_${c}_${PROF}_s${SCALE}
		$S/sitl.sh stop > /dev/null
		SIM_SPEED=4 $S/sitl.sh start ijc_gust $AF > /dev/null
		python3 $S/ijc_flight.py --name $N --outdir $OUT --speed 4 --simtime --gust-topic /ijc_gust_start --settle 10 --duration 42 $SCEN $P > $OUT/$N.txt 2>&1 || true
	done
done
$S/sitl.sh stop > /dev/null
cd $S && python3 calib_summary.py $OUT $PROF $SCALE $KINDS
