#!/usr/bin/env bash
# Validation of the selected gains on S1r (randomized S1-like disturbances, IJC_DIST = 2),
# seeds not used during the search; no retuning. Results: results/tuning/validation.csv
set -e
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
cd "$S"
IJC="IJC_KQ=400.0 IJC_KOM=30.0 IJC_K1=100.0 IJC_L1=2500.0 IJC_K2=20.0 IJC_L2=100.0 IJC_KP=4.0 IJC_KV=4.0"
PX4T=$(python3 -c "import tune; print(' '.join(f'{k}={v}' for k,v in tune.px4_params(9.0,2.0,2.5,3.0,3.0,4.0,1.0).items()))")
for seed in 11 23 37; do
	python3 tune.py validate ijc comp       $seed $IJC IJC_COMP=1
	python3 tune.py validate ijc nocomp     $seed $IJC IJC_COMP=0
	python3 tune.py validate px4 px4tuned   $seed $PX4T
	python3 tune.py validate px4 px4shipped $seed
done
