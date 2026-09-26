#!/usr/bin/env bash
# Final runs after the magnetometer fix (sensor_mag_sim): S2/S3 x5 at 4x, 1x equivalence runs,
# S1/S1h (4 cases, real time), S1r validation (3 seeds, 4x).
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
cd $S
./run_S2S3_reps.sh S2 5 4
./run_S2S3_reps.sh S3 5 4
# real-time equivalence check of the 4x runs (compensated and PX4 tuned)
PX4T=$(python3 -c "import tune; print(' '.join(f'--param {k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")
for SC in S2 S3; do
	[ $SC = S2 ] && P="--param IJC_TRAJ=0" || P="--param IJC_TRAJ=1 --param IJC_TRJ_R=2.0 --param IJC_TRJ_T=20.0 --param IJC_TRJ_TR=5.0"
	./sitl.sh stop >/dev/null; ./sitl.sh start ijc_wind 4050 >/dev/null
	python3 ijc_flight.py --name ${SC}_comp_rt --outdir ../logs/rt --settle 10 --duration 62 --param IJC_DIST=0 $P --param IJC_COMP=1 | grep ULog
	./sitl.sh stop >/dev/null; ./sitl.sh start ijc_wind 4051 >/dev/null
	python3 ijc_flight.py --name ${SC}_px4_rt --outdir ../logs/rt --settle 10 --duration 62 --param IJC_DIST=0 $P --param IJC_COMP=1 $PX4T | grep ULog
done
./sitl.sh stop >/dev/null
./run_S1.sh
./run_S1.sh high
./run_validation.sh
echo ALL_DONE
