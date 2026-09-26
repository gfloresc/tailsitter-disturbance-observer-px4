#!/usr/bin/env bash
# Scenario S1 (synthetic force/torque disturbances in hover), three cases:
#   <S>_comp    ijc_control, compensation on   (airframe 4050, IJC_COMP=1)
#   <S>_nocomp  ijc_control, compensation off  (airframe 4050, IJC_COMP=0)
#   <S>_px4     standard PX4 controllers, tuned with the same criterion as ijc_control (scripts/tune.py)
#   <S>_px4ship standard PX4 controllers with the tuning PX4 ships for the quadtailsitter (airframe 4051)
# Each run starts from a fresh SITL (identical initial conditions).
# A_F = 0.3 g. A_T = fraction of the per-axis angular-acceleration authority at hover
# (scripts/normalization.py: [30.354, 67.572, 23.342] rad/s^2):
#   run_S1.sh          -> S1,  A_T = 30 %
#   run_S1.sh high     -> S1h, A_T = 50 %
set -e
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
if [ "${1:-}" = "high" ]; then
	NAME=S1h; AT="--param IJC_ATX=15.177 --param IJC_ATY=33.786 --param IJC_ATZ=11.671"
else
	NAME=S1;  AT="--param IJC_ATX=9.106 --param IJC_ATY=20.272 --param IJC_ATZ=7.003"
fi
DIST="--param IJC_DIST=1 --param IJC_AF=2.942 $AT"
COMMON="--settle 10 --duration 25"
ONLY="${2:-comp nocomp px4 px4ship}"
PX4T=$(cd $S && python3 -c "import tune; print(' '.join(f'--param {k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")

run() {  # name airframe extra-params
	$S/sitl.sh stop > /dev/null
	$S/sitl.sh start ijc_calm "$2"
	python3 $S/ijc_flight.py --name "$1" $DIST $COMMON $3 | grep -E "hover:|scenario done|disarmed|ULog"
}

for c in $ONLY; do
	case $c in
	comp)   run ${NAME}_comp   4050 "--param IJC_COMP=1" ;;
	nocomp) run ${NAME}_nocomp 4050 "--param IJC_COMP=0" ;;
	px4)    run ${NAME}_px4    4051 "--param IJC_COMP=1 $PX4T" ;;
	px4ship) run ${NAME}_px4ship 4051 "--param IJC_COMP=1" ;;
	esac
done
$S/sitl.sh stop > /dev/null

cd $S && python3 postprocess.py $NAME ../logs/${NAME}_comp.ulg ../logs/${NAME}_nocomp.ulg --px4 ../logs/${NAME}_px4.ulg \
	--px4-shipped ../logs/${NAME}_px4ship.ulg
