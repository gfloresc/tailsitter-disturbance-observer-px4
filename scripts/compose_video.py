#!/usr/bin/env python3
"""Side-by-side comparison video of one scenario (ffmpeg: concat + hstack + ASS overlays + H.264).

usage: compose_video.py <S1|S2>

Inputs (from record_videos.sh): logs/video/<S>_<case>.ulg and logs/video/frames_<S>_<case>/ for the
cases comp, nocomp, px4. Output: results/videos/<S>_comparison.mp4 (1920x1080, 30 fps, H.264), whose
duration equals the scenario (S1: 20 s, S2: 60 s).

Synchronization: camera frames carry the simulation time; PX4 runs on the simulation clock, so the
scenario start of each run is read from its ULog (first ijc_debug sample with t_scen >= 0). For each
output frame k (scenario time t = k / 30 s) the camera frame with the nearest simulation time is used.
Overlays: title (first 3 s), column labels, scenario time and the current position error |e_p| of
each case, from Gazebo's ground truth.
"""
import csv
import os
import subprocess
import sys

import numpy as np
import imageio_ffmpeg

import postprocess as pp
from ulog_tools import load

from paths import ROOT  # noqa: E402  (repository root)
LOGS = f'{ROOT}/logs/video'
OUT = f'{ROOT}/results/videos'
FPS = 30
COL_W, H = 640, 1080
CASES = [('comp', 'With disturbance compensation'), ('nocomp', 'Without compensation'), ('px4', 'PX4 (tuned)')]
SCEN = {
    'S1': dict(T=20.0, title='S1 - Synthetic force/torque disturbances in hover',
               sub='Quad tail-sitter, PX4 SITL + Gazebo. Disturbances start at t = 5 s.',
               legend='Green sphere: position reference'),
    'S2': dict(T=60.0, title='S2 - Hover under wind',
               sub='Quad tail-sitter, PX4 SITL + Gazebo. Mean wind 4 m/s, gusts 3-5 m/s, turbulence.',
               legend='Green sphere: position reference. Blue arrow: mean wind direction'),
}
FONTS = '/usr/share/fonts/truetype/dejavu'


def ass_time(t):
    t = max(t, 0.0)
    h = int(t // 3600)
    m = int(t % 3600 // 60)
    s = t % 60
    return f'{h}:{m:02d}:{s:05.2f}'


def scenario_start(ulog):
    d = load(ulog, ['ijc_debug'])['ijc_debug']
    i = np.argmax(d['t_scen'] >= 0)
    return d['timestamp_sample'][i] * 1e-6 - d['t_scen'][i]      # simulation time [s] of t_scen = 0


def main():
    S = sys.argv[1]
    cfg = SCEN[S]
    T = cfg['T']
    n_frames = int(round(T * FPS))
    os.makedirs(OUT, exist_ok=True)
    work = f'{LOGS}/compose_{S}'
    os.makedirs(work, exist_ok=True)

    inputs, errors = [], []
    for case, _ in CASES:
        ulog = f'{LOGS}/{S}_{case}.ulg'
        t0 = scenario_start(ulog)
        # camera frames -> nearest frame for each output time
        with open(f'{LOGS}/frames_{S}_{case}/frames.csv') as f:
            rows = list(csv.DictReader(f))
        files = [r['file'] for r in rows]
        ts = np.array([float(r['sim_time_s']) for r in rows])
        grid = t0 + np.arange(n_frames) / FPS
        idx = np.clip(np.searchsorted(ts, grid), 1, len(ts) - 1)
        idx = np.where(np.abs(ts[idx - 1] - grid) <= np.abs(ts[idx] - grid), idx - 1, idx)
        gap = np.abs(ts[idx] - grid).max()
        if gap > 1.5 / FPS:
            print(f'warning: {S} {case}: nearest camera frame up to {gap * 1e3:.0f} ms away')
        lst = f'{work}/{case}.txt'
        with open(lst, 'w') as f:
            for k in idx:
                f.write(f"file '{LOGS}/frames_{S}_{case}/{files[k]}'\nduration {1.0 / FPS:.6f}\n")
            f.write(f"file '{LOGS}/frames_{S}_{case}/{files[idx[-1]]}'\n")
        inputs.append(lst)
        # current position error (ground truth), sampled at the output frame times
        R = pp.load_run(ulog, T + 0.5)
        err = np.linalg.norm(R['e_p_gt'], axis=1)
        errors.append(np.interp(np.arange(n_frames) / FPS, R['t'], err))
        print(f'{S} {case}: scenario start at sim time {t0:.3f} s, {len(rows)} camera frames, '
              f'max |e_p| = {errors[-1].max():.2f} m')

    # ---- ASS overlays
    W = COL_W * len(CASES)
    ass = [
        '[Script Info]', 'ScriptType: v4.00+', f'PlayResX: {W}', f'PlayResY: {H}', 'WrapStyle: 2', '',
        '[V4+ Styles]',
        'Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, '
        'Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding',
        'Style: Label,DejaVu Sans,34,&H00FFFFFF,&H00FFFFFF,&H00000000,&H90000000,1,0,0,0,100,100,0,0,3,10,0,8,10,10,24,1',
        'Style: Err,DejaVu Sans Mono,34,&H00FFFFFF,&H00FFFFFF,&H00000000,&H90000000,1,0,0,0,100,100,0,0,3,10,0,2,10,10,90,1',
        'Style: Time,DejaVu Sans Mono,36,&H00FFFFFF,&H00FFFFFF,&H00000000,&H90000000,1,0,0,0,100,100,0,0,3,10,0,2,10,10,24,1',
        'Style: Title,DejaVu Sans,58,&H00FFFFFF,&H00FFFFFF,&H00000000,&HB0000000,1,0,0,0,100,100,0,0,3,24,0,5,60,60,0,1',
        'Style: Sub,DejaVu Sans,30,&H00FFFFFF,&H00FFFFFF,&H00000000,&HB0000000,0,0,0,0,100,100,0,0,3,14,0,5,60,60,0,1',
        '', '[Events]', 'Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text']
    end = ass_time(T)
    for j, (_, label) in enumerate(CASES):
        x = int(COL_W * (j + 0.5))
        ass.append(f'Dialogue: 1,0:00:00.00,{end},Label,,0,0,0,,{{\\pos({x},24)\\an8}}{label}')
    for k in range(n_frames):
        a, b = ass_time(k / FPS), ass_time((k + 1) / FPS)
        for j, e in enumerate(errors):
            x = int(COL_W * (j + 0.5))
            ass.append(f'Dialogue: 1,{a},{b},Err,,0,0,0,,{{\\pos({x},{H - 90})\\an2}}|e_p| = {e[k]:.2f} m')
        if k % 3 == 0:   # scenario time, 10 Hz
            ass.append(f'Dialogue: 1,{a},{ass_time((k + 3) / FPS)},Time,,0,0,0,,{{\\pos({W // 2},{H - 24})\\an2}}t = {k / FPS:4.1f} s')
    ass.append(f'Dialogue: 2,0:00:00.00,0:00:03.00,Title,,0,0,0,,{{\\pos({W // 2},{H // 2 - 50})\\an5\\fad(0,400)}}{cfg["title"]}')
    ass.append(f'Dialogue: 2,0:00:00.00,0:00:03.00,Sub,,0,0,0,,{{\\pos({W // 2},{H // 2 + 40})\\an5\\fad(0,400)}}{cfg["sub"]}\\N{cfg["legend"]}')
    ass_file = f'{work}/overlay.ass'
    with open(ass_file, 'w') as f:
        f.write('\n'.join(ass) + '\n')

    # ---- ffmpeg: concat (synchronized frames) -> hstack -> ASS -> H.264
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    cmd = [ff, '-y', '-hide_banner', '-loglevel', 'error']
    for lst in inputs:
        cmd += ['-f', 'concat', '-safe', '0', '-i', lst]
    chains = ''.join(f'[{j}:v]fps={FPS},scale={COL_W}:{H},setsar=1[c{j}];' for j in range(len(inputs)))
    stack = ''.join(f'[c{j}]' for j in range(len(inputs))) + f'hstack=inputs={len(inputs)}[s];'
    fc = chains + stack + f"[s]ass='{ass_file}':fontsdir='{FONTS}'[v]"
    out = f'{OUT}/{S}_comparison.mp4'
    cmd += ['-filter_complex', fc, '-map', '[v]', '-r', str(FPS), '-frames:v', str(n_frames),
            '-c:v', 'libx264', '-preset', 'slow', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]
    subprocess.run(cmd, check=True)
    print('written', out)


if __name__ == '__main__':
    main()
