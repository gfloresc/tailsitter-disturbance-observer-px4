# Comparison videos

The official final versions of the two gust-front videos, `hover_final.mp4` and `circle_final.mp4`
(with the corrected title screen), and the generated versions described below are distributed in the
Release asset `videos.zip`.

## Gust-front videos (current)

`hover_aggressive_comparison.mp4` (hover) and `circle_aggressive_comparison.mp4` (circle of radius
2 m, period 20 s, altitude 2.5 m, psi_d = 0, 5 s ramp of the angular rate as in S3), both under the
same disturbance. Three synchronized columns: *With disturbance compensation* (`ijc_control`,
IJC_COMP=1), *Without compensation* (IJC_COMP=0), *PX4 (tuned)* (standard PX4 controllers with the tuned
gains), all with the final gains. MP4, H.264 (CRF 18), 1920x1080, 30 fps, 43 s: 3 s title screen (its own
clip) + 40 s of scenario. Layout: three 640x780 camera columns and a full-width 1920x300 live plot of
the position error ||e_p|| (Gazebo ground truth; compensated blue solid, non compensated red dashed,
PX4 black dash-dot) revealed up to a cursor, with the scenario time. Review frames (t = 1 s and 20 s):
`*_aggressive_start.png`, `*_aggressive_middle.png`. Demonstration runs only: the results of S1-S3 do
not change.

### Disturbance: selection

Goal: the disturbance that best shows the advantage of the observer over PX4 (tuned) and over the non
compensated case, i.e. the largest RMS ||e_p||(PX4 tuned) - RMS ||e_p||(compensated), with no case
saturating the rotors more than 5 % of the time (fraction of `control_allocator_status` samples with
the thrust or torque setpoint not achieved). Search at x4 without camera (`scripts/calib_gust.sh
<profile> <scale> hover|circle`, summary in `logs/video/calib/summary.csv`), 40 s window, same wind
timing as the videos. Profiles (`PROFILES` in `scripts/make_video_worlds.py`; wind time 8-48 s is the
video window):

- `sines`: previous version, mean 3.5 m/s + slow/fast speed sines + +-50 deg direction sines + turbulence.
- `fronts`: mean 3.0 m/s + mild sines/turbulence + step-like gust fronts every 6.5 s (+2.5 m/s, 0.3 s
  rise, held 3 s, 1.2 s fall) and, 3 s after each, a sudden direction change of alternating sign (+-40 deg,
  0.4 s, held 2.5 s).
- `fronts_strong`: `fronts` with +3.5 m/s and +-55 deg.
- `dirsteps`: mean 3.5 m/s, only sudden +-60 deg direction steps every 5 s (0.3 s, held 3.5 s).
- `fronts_pulses`: `fronts` + external pulses: 5 N horizontal force (0.6 s, random direction) and
  0.4/0.28 N m roll/pitch torque (0.4 s), six of each.
- `sines_pulses`: mild sines + stronger pulses (8 N, 0.6 N m).
- `fronts_fpulses`: `fronts` + force-only pulses (4 N, 0.5 s).
- `fronts_sharp`: mean 2.8 m/s + mild sines + sharper, longer fronts (+2.8 m/s, 0.2 s rise, held 4 s,
  1 s fall) and +-45 deg direction changes (0.3 s, held 2.5 s).

| profile x scale | scenario | RMS comp [m] | RMS no comp [m] | RMS PX4 [m] | PX4 - comp [m] | worst saturation |
|---|---|---|---|---|---|---|
| sines x1 | hover | 0.210 | 0.621 | 0.352 | +0.142 | 0.5 % |
| fronts x1 | hover | 0.211 | 0.894 | 0.392 | +0.181 | **5.4 %** (comp) |
| fronts_strong x1 | hover | 0.926 | 0.911 | 0.621 | -0.305 | **28 %** (comp) |
| dirsteps x1 | hover | 0.192 | 0.394 | 0.220 | +0.027 | 0 % |
| fronts_pulses x1 | hover | 0.355 | 0.851 | 0.517 | +0.162 | **28 %** (comp) |
| sines_pulses x1 | hover | 0.230 | 0.517 | 0.299 | +0.069 | **9.8 %** (comp) |
| fronts x0.9 | hover | 0.177 | 0.694 | 0.342 | +0.165 | 1.3 % |
| fronts x0.9 | circle | 0.280 | 0.717 | 0.380 | +0.100 | 0.6 % |
| fronts_fpulses x0.9 | hover | 0.297 | 0.713 | 0.363 | +0.066 | **19 %** (comp) |
| **fronts_sharp x0.85 (chosen)** | hover | 0.174 | 0.726 | 0.345 | **+0.171** | 2.9 % |
| **fronts_sharp x0.85 (chosen)** | circle | 0.239 | 0.741 | 0.411 | **+0.172** | 2.9 % |
| fronts_sharp x0.9 | hover | 0.243 | 0.754 | 0.390 | +0.147 | **5.4 %** (comp) |

Findings: step-like gust fronts are what separates the observer from PX4 (the translational observer has a
0.1 s time constant, while the integral action of the PX4 velocity loop, MPC_XY_VEL_P_ACC =
MPC_XY_VEL_I_ACC = 3, has an integral time of about 1 s), sudden direction changes
alone barely do (+0.03 m), and external pulses do not help: even 4 N force pulses make the compensated
case saturate (19 %), because it reacts to the pulse with a fast attitude change. The compensated case is
always the first to saturate; beyond the limit its advantage collapses (fronts_strong). The chosen
profile gives the largest advantage within the limit in both scenarios; one step more (x0.9) violates it.

### Disturbance: chosen profile (`fronts_sharp` x 0.85)

| component | value |
|---|---|
| base wind | 2.4 m/s towards 70 deg ENU (NNE) |
| gust fronts | +2.4 m/s every 6.5 s (wind time 10, 16.5, ... 42.5 s): 0.2 s rise, held 4 s, 1 s fall |
| direction changes | 3 s after each front, +-45 deg (alternating sign): 0.3 s, held 2.5 s, 0.8 s back |
| slow / fast gusts | 0.68 m/s @ 6.5 s, 0.26 m/s @ 5.1 s / 0.34 m/s @ 2.3 s, 0.21 m/s @ 1.3 s |
| direction sines | 12 deg @ 9 s, 6 deg @ 2.7 s, + Gauss-Markov noise 3 deg |
| turbulence | Gauss-Markov, sigma 0.26 m/s horizontal / 0.10 m/s vertical, tau 0.3 s |
| onset | 5 s ramp, started 8 s (simulated) before the scenario start |

Measured in the 40 s window: speed 4.05 +- 1.06 m/s (min 1.4, max 6.6 m/s; the base wind is 2.4 m/s,
the fronts raise the average), direction 23-122 deg ENU (5-95 %). The wind is started by
`ijc_flight.py --gust-topic` a fixed simulated time before the scenario start, so all runs see the same
wind relative to their scenario (spread 11-13 ms in these runs); the vehicle takes off in calm air.

### Results of the video runs (40 s, ground truth, real time)

| video | case | RMS ||e_p|| [m] | max ||e_p|| [m] | saturated time |
|---|---|---|---|---|
| hover | with compensation | 0.207 | 0.46 | 2.74 % |
| hover | without compensation | 0.680 | 1.13 | 0 % |
| hover | PX4 (tuned) | 0.402 | 0.77 | 0 % |
| circle | with compensation | 0.213 | 0.45 | 1.68 % |
| circle | without compensation | 0.783 | 1.30 | 0 % |
| circle | PX4 (tuned) | 0.395 | 0.95 | 0 % |

### Scene

Worlds `ijc_gust_hover_video` / `ijc_gust_circle_video` (generated by `scripts/make_video_worlds.py`
from `ijc_calm`: same physics, 1 kHz): sky, shadows (sun almost overhead, darker ambient), textured
ground (model `ijc_ground`: grass-like noise, 1 m grid, 5 m major lines, 3x3 tiles = 96 m), no other
objects. Cameras fixed in the world (640x780, 30 Hz, rendered offscreen by the Gazebo server), markers
moved by `IjcVideoSystem` (`gz_plugins/ijc_video/`, `follow=false`):

- Hover: camera 2.1 m from the reference (azimuth -50 deg from east, 12 deg above), aimed at the
  green semi-transparent sphere at the hover reference; the drone is large in the frame and every run
  stays inside it (checked by projecting the ground-truth trajectories). Orange trail of the last 5 s.
  Wind arrow low, behind the reference (bottom of the image, seen from above so its direction reads
  well).
- Circle: near-overhead camera (75 deg elevation, from the south-east, 7.4 m from the circle centre at
  the flight altitude, HFOV 48 deg). The desired trajectory is a thin semi-transparent yellow ring at the
  flight altitude (placed relative to the takeoff point, like the hover reference); the circle on the
  ground was removed (even very faint it read as a second ring). A small green sphere moves along the
  ring at the instantaneous desired position: `IjcVideoSystem` evaluates ijc_control's circle reference
  (radius, period, 5 s ramp) with the scenario start given by the wind trigger + 8 s. Orange trail of
  one lap (20 s, 200 samples). Wind arrow at the centre of the ring.
- Wind arrow: blue 3D arrow of the instantaneous horizontal wind (0.17 m per m/s in hover, 0.23 in the
  circle, centred on a fixed point, updated at 30 Hz).

**How they are made** (`scripts/make_videos.sh hover circle`): `make_video_worlds.py fronts_sharp 0.85`
-> `record_videos.sh hover|circle` (Gazebo server with `--headless-rendering`, PX4 attached, real time,
frames with their simulation time, wind log per run) -> `compose_aggressive.py hover|circle`
(synchronization at the scenario start from the ULogs; live plot rendered with matplotlib frame by frame
and piped to ffmpeg; `hstack` + `vstack`, ASS labels, title clip, `concat`).

## Previous aggressive version (multi-frequency sines)

Profile `sines` x 1 (mean 3.5 m/s, slow 5-6.5 s and fast 1.3-2.3 s gusts up to 6.9 m/s, +-50 deg direction
sines, turbulence 0.4 m/s), fixed oblique cameras, circle drawn on the ground. The requested 6-8 m/s
mean wind was not flyable by any controller (at 7 m/s mean all three lose control); with that profile,
the largest scale within the saturation limit was 0.5 (0.55: compensated 8.4 %). Video runs: hover
0.234 / 0.544 / 0.360 m, circle 0.239 / 0.602 / 0.376 m RMS (compensated / non compensated / PX4
tuned), compensated saturation 0.5-0.6 %.

## First version

`S1_comparison.mp4` (synthetic force/torque disturbances in hover, 20 s) and `S2_comparison.mp4`
(hover under wind, 60 s): side by side, *With disturbance compensation* (`ijc_control`, IJC_COMP=1),
*Without compensation* (IJC_COMP=0) and *PX4 (tuned)* (standard PX4 controllers with the tuned gains),
all with the final gains. MP4, H.264, 1920x1080 (three 640x1080 columns), 30 fps; duration = scenario.

**How they were made** (`scripts/make_videos.sh`):

1. Recording (`scripts/record_videos.sh`): dedicated worlds `ijc_calm_video` / `ijc_wind_video` (same
   physics, wind and disturbance injection as `ijc_calm` / `ijc_wind`) with a virtual camera sensor
   (640x1080, 30 Hz, horizontal FOV 43 deg) rendered offscreen by the Gazebo server
   (`gz sim -s --headless-rendering`; no screen involved). The Gazebo system `IjcVideoSystem`
   (`PX4-Autopilot/src/modules/simulation/gz_plugins/ijc_video/`) keeps the camera at a fixed offset
   from the vehicle in the world frame (2.4 m east, 1.0 m south, 1.3 m up, looking at it; it does not
   rotate with the vehicle, so the tilt is visible), places a semi-transparent green sphere at the
   position reference (hover point) and, in S2, a blue arrow along the mean wind direction next to the
   vehicle; a 1 m grid on the ground gives the motion reference. Each flight uses the same automated
   procedure as the experiments (`scripts/ijc_flight.py`, waiting in simulation time); the frames are
   recorded with their simulation time (`scripts/record_camera.py`).
2. Editing (`scripts/compose_video.py`, ffmpeg 7.0.2 static build from the `imageio-ffmpeg` pip
   package): PX4 runs on the simulation clock, so the scenario start of each run is read from its ULog
   and every output frame (scenario time t = k/30 s) takes the camera frame with the nearest simulation
   time -> the three columns are synchronized at the scenario start. Columns joined with `hstack`;
   overlays with ASS subtitles (libass): column labels, scenario time, current position error |e_p| of
   each case (from Gazebo's ground truth, as the main metric of the paper tables) and the title during
   the first 3 s (overlaid, so that the duration equals the scenario). Encoding: libx264, CRF 18,
   yuv420p.

The flights of the videos are separate runs from the ones of the metrics (same configuration); the
ULogs are in `logs/video/`. Rendering is software (no GPU driver available), which does not affect the
simulation: PX4 follows the simulation clock.
