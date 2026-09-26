#!/usr/bin/env bash
# S2/S3 with repetitions: run_S2S3_reps.sh <S2|S3> [reps=5] [speed=4]
# Logs: logs/<S>/<S>_<case>_r<i>.ulg for the four cases (see run_S2S3.sh). Each flight starts from
# a fresh SITL; repetitions differ in the sensor-noise and wind-noise realizations.
set -e
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
NAME=${1:?S2 or S3}; REPS=${2:-5}; SPEED=${3:-4}
case $NAME in
S2) SCEN="--param IJC_DIST=0 --param IJC_TRAJ=0" ;;
S3) SCEN="--param IJC_DIST=0 --param IJC_TRAJ=1 --param IJC_TRJ_R=2.0 --param IJC_TRJ_T=20.0 --param IJC_TRJ_TR=5.0" ;;
*) echo "unknown scenario $NAME"; exit 1 ;;
esac
OUT=$ROOT/logs/$NAME
COMMON="--settle 10 --duration 62 --speed $SPEED --outdir $OUT"
PX4T=$(cd $S && python3 -c "import tune; print(' '.join(f'--param {k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")

run() {  # name airframe extra-params
	[ -f "$OUT/$1.ulg" ] && return 0     # resumable
	$S/sitl.sh stop > /dev/null
	SIM_SPEED=$SPEED $S/sitl.sh start ijc_wind "$2" > /dev/null
	python3 $S/ijc_flight.py --name "$1" $SCEN $COMMON $3 | grep -E "ULog" || echo "FAILED $1"
}

for i in $(seq 1 $REPS); do
	run ${NAME}_comp_r$i    4050 "--param IJC_COMP=1"
	run ${NAME}_nocomp_r$i  4050 "--param IJC_COMP=0"
	run ${NAME}_px4_r$i     4051 "--param IJC_COMP=1 $PX4T"
	run ${NAME}_px4ship_r$i 4051 "--param IJC_COMP=1"
done
$S/sitl.sh stop > /dev/null
