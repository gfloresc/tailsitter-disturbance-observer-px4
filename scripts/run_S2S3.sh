#!/usr/bin/env bash
# Scenarios with Gazebo wind (world ijc_wind: mean 4 m/s, gusts 3-5 m/s, turbulence), four cases:
#   <S>_comp, <S>_nocomp (ijc_control, final gains, airframe 4050), <S>_px4 (standard PX4 controllers
#   tuned with scripts/tune.py), <S>_px4ship (shipped quadtailsitter tuning, airframe 4051).
#   run_S2S3.sh S2   hover under wind, 60 s
#   run_S2S3.sh S3   circle (R = 2 m, T = 20 s, constant altitude, psi_d = 0) under wind, 60 s (3 laps)
# Each run starts from a fresh SITL, real time.
set -e
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
NAME=${1:?S2 or S3}
case $NAME in
S2) SCEN="--param IJC_DIST=0 --param IJC_TRAJ=0" ;;
S3) SCEN="--param IJC_DIST=0 --param IJC_TRAJ=1 --param IJC_TRJ_R=2.0 --param IJC_TRJ_T=20.0 --param IJC_TRJ_TR=5.0" ;;
*) echo "unknown scenario $NAME"; exit 1 ;;
esac
COMMON="--settle 10 --duration 62"
ONLY="${2:-comp nocomp px4 px4ship}"
PX4T=$(cd $S && python3 -c "import tune; print(' '.join(f'--param {k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")

run() {  # name airframe extra-params
	$S/sitl.sh stop > /dev/null
	$S/sitl.sh start ijc_wind "$2"
	python3 $S/ijc_flight.py --name "$1" $SCEN $COMMON $3 | grep -E "hover:|scenario done|disarmed|ULog"
}

for c in $ONLY; do
	case $c in
	comp)    run ${NAME}_comp    4050 "--param IJC_COMP=1" ;;
	nocomp)  run ${NAME}_nocomp  4050 "--param IJC_COMP=0" ;;
	px4)     run ${NAME}_px4     4051 "--param IJC_COMP=1 $PX4T" ;;
	px4ship) run ${NAME}_px4ship 4051 "--param IJC_COMP=1" ;;
	esac
done
$S/sitl.sh stop > /dev/null

cd $S && python3 postprocess.py $NAME ../logs/${NAME}_comp.ulg ../logs/${NAME}_nocomp.ulg --px4 ../logs/${NAME}_px4.ulg \
	--px4-shipped ../logs/${NAME}_px4ship.ulg --tend 60
