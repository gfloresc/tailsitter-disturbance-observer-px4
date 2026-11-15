# Observer-based disturbance-compensated control of a tail-sitter UAV in PX4 SITL

PX4 + Gazebo software-in-the-loop implementation of the control scheme of the paper
*"Observer-Based Disturbance-Compensated Position and Attitude Control of Tail-Sitter UAVs"*
(A. Flores and G. Flores, International Journal of Control, 2026; the manuscript is not included).
Only the paper's control laws are in the loop: translational and rotational disturbance observers
without leakage, the position law with the force
estimate `F_hat`, the desired attitude `Q_d` with constant yaw, and the attitude law with the torque
estimate `T_hat`. PX4 provides the infrastructure (uORB, EKF2, control allocator, logger, Gazebo
bridge); no standard PX4 control law runs when the module `ijc_control` flies. The standard PX4
controllers, with their shipped and with a tuned set of gains, are included as comparison cases.

**Video:** https://youtu.be/IsXVBqzxwI0 (hover and circular tracking under gusty wind: with
disturbance compensation, without compensation and PX4 tuned).

The repository contains only our code: the PX4 changes are stored as patches against the official
PX4 **v1.17.0** plus our new files, and `setup.sh` rebuilds the exact PX4 tree used for the results.

## Contents

| Path | Content |
|---|---|
| `setup.sh` | clones PX4 v1.17.0, applies the patches, copies our files and builds |
| `px4/VERSION` | exact base: PX4 `v1.17.0` = `d6f12ad1c4f7...`, Gazebo models submodule `b6127f4ec20d...` |
| `px4/patches/` | changes to existing PX4 files (11 files) and to the Gazebo models submodule (1 file) |
| `px4/overlay/` | new files, with their paths in the PX4 tree: module `src/modules/ijc_control/`, uORB messages `msg/Ijc*.msg`, airframes 4050 (ijc_control) and 4051 (standard PX4 controllers), Gazebo plugins `gz_plugins/ijc_{wrench,gust,video}/`, model `quadtailsitter_ijc`, worlds `ijc_*.sdf` |
| `px4/export_px4_changes.sh` | regenerates `patches/` and `overlay/` from a modified PX4 tree (maintainers) |
| `scripts/` | SITL control, automated flights, experiments S1-S3, gain search, figures, metrics, videos, interactive demo |
| `spec/` | implementation brief (`SPEC_px4_ijc.md`) and the reference Python simulation of the paper (`sim_python/`) |
| `reference/` | figures of the reference Python simulation |
| `results/` | figures, metrics (`metrics.csv`, `metrics_runs.csv`, LaTeX table), parameters used, gain-search results |
| `docs/NOTES.md` | technical notes: implementation, normalization, gain selection, methodology, detailed results, deviations from the spec |

Large data (ULogs, time series) are distributed as Release assets (see [Data](#data-release-v10)).

## Requirements

- Ubuntu 22.04 (x86_64), about 5 GB of free disk for PX4 and its build (plus the toolchain installed by PX4's setup script).
- Gazebo Harmonic (installed by PX4's setup script), Python 3.10.
- A GPU is optional. On hybrid-graphics laptops (Intel + NVIDIA, PRIME on-demand) the scripts render
  Gazebo on the NVIDIA GPU when its driver is loaded (`scripts/gpu_env.sh`).

## Installation

```bash
git clone <this repository> tailsitter-disturbance-observer-px4
cd tailsitter-disturbance-observer-px4

# 1. PX4 v1.17.0 + our changes (no build yet)
./setup.sh --no-build

# 2. System dependencies, once (sudo): PX4 toolchain and Gazebo Harmonic
bash PX4-Autopilot/Tools/setup/ubuntu.sh
#    then log out and in again (dialout group)

# 3. Python packages
pip3 install --user -r requirements.txt

# 4. Build
make -C PX4-Autopilot px4_sitl_default
```

If the dependencies are already installed, `./setup.sh` does steps 1 and 4 at once.

`setup.sh` clones the official PX4 repository at tag `v1.17.0` with its submodules, checks the exact
commits of PX4 and of the Gazebo models submodule, applies `px4/patches/`, copies `px4/overlay/`,
generates the ground texture of the demo/video worlds and builds `px4_sitl_default`. The changes stay
uncommitted in the PX4 working tree (`git -C PX4-Autopilot status` lists them). PX4 is expected in
`./PX4-Autopilot`; to use another location set `PX4_DIR` for `setup.sh` and for the scripts.

## Quick start: interactive demo with the Gazebo GUI

```bash
./scripts/demo.sh <hover|circle> <comp|nocomp|px4> <calm|paper|gusty> [duration_s]
./scripts/demo.sh circle comp gusty
```

| Argument | Values |
|---|---|
| scenario | `hover`: hold the takeoff point at 2.5 m; `circle`: R = 2 m, period 20 s, 2.5 m (reference of S3) |
| controller | `comp`: ijc_control with disturbance compensation (airframe 4050, `IJC_COMP=1`); `nocomp`: without it (`IJC_COMP=0`); `px4`: standard PX4 controllers with the tuned gains (airframe 4051) |
| wind | `calm`: none; `paper`: wind of S2/S3 (mean 4 m/s, gusts 3-5 m/s, turbulence); `gusty`: gust fronts of the videos |
| duration_s | scenario duration, default 40 s |

The script stops previous simulations, generates the world `ijc_demo`, starts the Gazebo server and
GUI, starts PX4 with the airframe and parameters of the chosen controller, and flies automatically:
takeoff (5 s), hover (10 s), scenario, landing. The terminal prints the position error twice per second
(against Gazebo's ground truth and as seen by the controller through the EKF) and its RMS at the end.
Close the Gazebo window, or press Ctrl+C, to stop everything. Logs go to `logs/demo/`.

In the window: green sphere = hover reference (hover) or instantaneous desired position (circle);
yellow ring = desired circle at the flight altitude; orange dots = last 5 s (hover) or last lap
(circle); blue arrow = wind. Camera: left drag pans, scroll or right drag zooms, middle drag (or
Shift + left drag) orbits; right-click on the drone -> *Follow* / *Free Look* / *Move To*. Do not
use *Reset* during a flight (it restarts the simulation clock and the flight script stops).

## Reproducing the experiments

All experiment scripts are in `scripts/` and run headless. Each flight starts from a fresh SITL with
the airframe default parameters; logs go to `logs/`.

| Scenario | Description |
|---|---|
| S1 | hover with synthetic force/torque disturbances (A_T = 30 % of the authority); S1h: 50 % |
| S1r | S1-like randomized disturbances (3 seeds), validation of the selected gains |
| S2 | hover under Gazebo wind (mean 4 m/s, gusts, turbulence), 60 s |
| S3 | circle (R = 2 m, T = 20 s) under the same wind, 60 s |

Four cases in every scenario: compensated and non-compensated `ijc_control` (airframe 4050), standard
PX4 controllers with the tuned gains and with the gains shipped by PX4 (airframe 4051).

```bash
cd scripts
./run_S1.sh                 # S1: 4 flights in real time + figures + metrics
./run_S1.sh high            # S1h
./run_validation.sh         # S1r validation (4x real time)
./run_S2S3_reps.sh S2 5 4   # S2: 4 cases x 5 repetitions at 4x real time
./run_S2S3_reps.sh S3 5 4   # S3
python3 postprocess.py S2 --reps ../logs/S2 --tend 60   # mean +- std over the repetitions, figures of repetition 1
python3 postprocess.py S3 --reps ../logs/S3 --tend 60   # (+ top view)
./run_all_final.sh          # all of the above, plus the real-time equivalence runs of the 4x runs
```

The simulation runs faster than real time (4x) where stated; the equivalence with real-time runs was
verified (docs/NOTES.md). Optional gain search (about 1.5 h at 4x): `python3 tune.py search ijc` and
`python3 tune.py search px4`. Single flights: `./sitl.sh start [world] [4050|4051]`, then
`python3 ijc_flight.py --name <run> --param NAME=VALUE ... --settle 10 --duration 25`, and
`./sitl.sh stop`.

### Figures, metrics and tables

`run_S1.sh` and `postprocess.py` write, in the style of the paper's plotting script
(`spec/sim_python/plots.py`):

- `results/figures/sitl_<S>_{pos,att,inputs,inputs_cmp}.{pdf,png}` (S1, S1h, S2, S3) and `sitl_S3_xy.pdf`,
- `results/metrics.csv`, `results/metrics_runs.csv` (per repetition) and `results/metrics_table.tex`
  (ground-truth position error as the main metric, attitude errors, estimation errors against the
  applied or the measured aerodynamic wrench, saturation),
- `results/params.txt` (`python3 write_params.py <ulogs>`: parameters actually used),
- `results/plotdata/` (`python3 export_plotdata.py`: time series of the figures as CSV at 1 kHz).

With the Release ULogs (below) the figures and tables can be regenerated without flying.

### Reference Python simulation

```bash
cd spec/sim_python && python3 simnb.py && python3 plots.py   # needs numba; figures as in reference/
```

## Videos

```bash
./scripts/make_videos.sh hover circle
```

It generates the worlds (`make_video_worlds.py`, disturbance profile `fronts_sharp` x 0.85), records
the three cases of each scenario with an offscreen camera in Gazebo (real time), and composes each
video with ffmpeg: title screen, three synchronized columns and a live plot of the position error
(`compose_aggressive.py`). Output: `results/videos/{hover,circle}_aggressive_comparison.mp4`. The
choice of the disturbance (search at 4x with a saturation limit of 5 % for every case,
`calib_gust.sh`) and all the numbers are documented in `results/videos/README.md`. The narrated
video of these two scenarios is on YouTube: https://youtu.be/IsXVBqzxwI0.

## Data (Release v1.0)

The simulation logs and the full-rate time series are not stored in the repository; they are the
assets of the GitHub Release
[v1.0](https://github.com/gfloresc/tailsitter-disturbance-observer-px4/releases/tag/v1.0).

| File | Size | Content |
|---|---|---|
| `results_data.zip` | 58 MB | `results/` as in the repository (figures, `metrics.csv`, `metrics_runs.csv`, `metrics_table.tex`, `params.txt`, gain-search CSVs) plus the 1 kHz time series of every figure, `results/plotdata/<S>_<case>.csv` (S1, S2, S3 x 4 cases; columns in `results/plotdata/README.md`) |
| `plotdata_100Hz.zip` | 7.5 MB | the same time series subsampled to 100 Hz (all columns), with `metrics.csv`, `metrics_runs.csv` and `params.txt`: the light version for plotting |
| `ulogs_S1.tar.zst` | 109 MB | PX4 ULogs of S1 and S1h, 4 cases each (`logs/S1_*.ulg`, `logs/S1h_*.ulg`) with their run metadata (`.json`: parameters, durations) |
| `ulogs_S2.tar.zst` | 587 MB | ULogs of S2, 4 cases x 5 repetitions (`logs/S2/S2_<case>_r<i>.ulg` + `.json`) |
| `ulogs_S3.tar.zst` | 618 MB | ULogs of S3, 4 cases x 5 repetitions (`logs/S3/S3_<case>_r<i>.ulg` + `.json`) |
| `SHA256SUMS` | | SHA-256 checksums of the five files above |
| `RELEASE_NOTES.md` | | summary of the assets |

Cases: `comp` (ijc_control with compensation), `nocomp` (without), `px4` (standard PX4 controllers,
tuned gains), `px4ship` (standard PX4 controllers, gains shipped by PX4).

### Download

From the repository root, into `release/` (ignored by git):

```bash
gh release download v1.0 -R gfloresc/tailsitter-disturbance-observer-px4 -D release
# or only some files:
gh release download v1.0 -R gfloresc/tailsitter-disturbance-observer-px4 -D release -p 'ulogs_S1.tar.zst' -p SHA256SUMS
```

The files can also be downloaded from the Release page in the browser.

### Verify the checksums

```bash
cd release && sha256sum -c --ignore-missing SHA256SUMS && cd ..
```

Every downloaded file must report `OK` (`--ignore-missing` skips the files you did not download).

### Where to put them

Run the commands from the repository root:

```bash
tar --zstd -xf release/ulogs_S1.tar.zst   # -> logs/S1_*.ulg, logs/S1h_*.ulg
tar --zstd -xf release/ulogs_S2.tar.zst   # -> logs/S2/
tar --zstd -xf release/ulogs_S3.tar.zst   # -> logs/S3/
unzip -o release/results_data.zip         # -> results/ (adds results/plotdata/*.csv; the other files are those of the repository)
unzip release/plotdata_100Hz.zip -d release/   # -> release/plotdata_100Hz/
```

(`tar --zstd` needs the `zstd` package: `sudo apt install zstd`.) `logs/` and the extracted data are
ignored by git.

### Regenerate figures, metrics and tables from the ULogs (no simulation needed)

```bash
cd scripts
python3 postprocess.py S1  ../logs/S1_comp.ulg  ../logs/S1_nocomp.ulg  --px4 ../logs/S1_px4.ulg  --px4-shipped ../logs/S1_px4ship.ulg
python3 postprocess.py S1h ../logs/S1h_comp.ulg ../logs/S1h_nocomp.ulg --px4 ../logs/S1h_px4.ulg --px4-shipped ../logs/S1h_px4ship.ulg
python3 postprocess.py S2 --reps ../logs/S2 --tend 60   # mean +- std over the 5 repetitions, figures of repetition 1
python3 postprocess.py S3 --reps ../logs/S3 --tend 60
python3 write_params.py ../logs/S1_{comp,nocomp,px4,px4ship}.ulg ../logs/S1h_{comp,nocomp,px4,px4ship}.ulg \
    ../logs/S2/S2_{comp,nocomp,px4,px4ship}_r1.ulg ../logs/S3/S3_{comp,nocomp,px4,px4ship}_r1.ulg
python3 export_plotdata.py                              # results/plotdata/*.csv (1 kHz)
```

They rewrite `results/figures/`, `results/metrics.csv`, `results/metrics_runs.csv`,
`results/metrics_table.tex`, `results/params.txt` and `results/plotdata/` from the ULogs. Checked on a
fresh clone with the v1.0 assets: `metrics.csv`, `metrics_runs.csv`, `metrics_table.tex`, the PNG
figures and the 1 kHz time series are reproduced exactly; the PDF figures differ only in their
creation date, and `params.txt` only in the line with the version of the local PX4 tree (`n/a`
without `PX4-Autopilot`).

## Maintaining the PX4 changes

Work in `PX4-Autopilot` (and its submodule `Tools/simulation/gz`) as usual; new files must be tracked
by git (`git add`). Then `px4/export_px4_changes.sh` regenerates `px4/patches/` and `px4/overlay/` by
comparing the working tree with the base commits of `px4/VERSION`.

## License

BSD 3-Clause (`LICENSE`), the same license as PX4. The patches in `px4/patches/` modify PX4 files,
which remain under the PX4 license.
