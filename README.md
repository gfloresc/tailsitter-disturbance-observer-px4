# Observer-based disturbance-compensated control of a tail-sitter UAV in PX4 SITL

PX4 + Gazebo software-in-the-loop implementation of the controller of the paper *"Observer-Based
Disturbance-Compensated Position and Attitude Control of Tail-Sitter UAVs"* (International Journal of
Control, 2026): translational and rotational disturbance observers with compensation in the position
and attitude laws, compared with the same controller without compensation and with the standard PX4
controllers.

**Video:** https://youtu.be/IsXVBqzxwI0

## Requirements

Ubuntu 22.04 (x86_64), Python 3.10, about 5 GB of free disk. Gazebo Harmonic and the PX4 toolchain are
installed in step 2 below.

## Installation

```bash
git clone https://github.com/gfloresc/tailsitter-disturbance-observer-px4.git
cd tailsitter-disturbance-observer-px4
./setup.sh --no-build                       # PX4 v1.17.0 + the changes of this repository
bash PX4-Autopilot/Tools/setup/ubuntu.sh    # PX4 toolchain and Gazebo (once; then log out and in)
pip3 install --user -r requirements.txt
make -C PX4-Autopilot px4_sitl_default
```

## Run

Interactive flight with the Gazebo GUI:

```bash
./scripts/demo.sh <hover|circle> <comp|nocomp|px4> <calm|paper|gusty>
./scripts/demo.sh circle comp gusty
```

`comp`: with disturbance compensation, `nocomp`: without it, `px4`: standard PX4 controllers (tuned).
The terminal prints the position error and its RMS at the end.

## Reproduce the results of the paper

```bash
cd scripts
./run_S1.sh                 # S1: hover with force/torque disturbances
./run_S2S3_reps.sh S2 5 4   # S2: hover under wind (5 repetitions, 4x real time)
./run_S2S3_reps.sh S3 5 4   # S3: circle under wind
python3 postprocess.py S2 --reps ../logs/S2 --tend 60
python3 postprocess.py S3 --reps ../logs/S3 --tend 60
```

Figures and metrics are written to `results/`. The flight logs of the paper are in the
[Release v1.0](https://github.com/gfloresc/tailsitter-disturbance-observer-px4/releases/tag/v1.0)
(extract the `ulogs_*.tar.zst` files at the repository root to regenerate the figures without flying).
Technical details: `docs/NOTES.md`.

## License

BSD 3-Clause (`LICENSE`). The patches in `px4/patches/` modify PX4 files, which remain under the PX4
license.

## Citation

If you use this code, please cite:

```bibtex
@article{flores2026tailsitter,
  author  = {Flores, Alejandro and Flores, Gerardo},
  title   = {Observer-Based Disturbance-Compensated Position and Attitude Control of Tail-Sitter {UAVs}},
  journal = {International Journal of Control},
  year    = {2026}
}
```
