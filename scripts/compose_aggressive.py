#!/usr/bin/env python3
"""Comparison video under the aggressive gusts (fixed camera, three synchronized columns + live error plot).

usage: compose_aggressive.py <hover|circle>

Inputs (from record_videos.sh hover|circle): logs/video/<k>_<case>.ulg, logs/video/frames_<k>_<case>/
and logs/video/wind_<k>_<case>.csv for the cases comp, nocomp, px4.
Output: results/videos/<k>_aggressive_comparison.mp4 (1920x1080, 30 fps, H.264):
  3 s title screen, then the 40 s scenario: three 640x780 camera columns (no scaling) and, below, a
  1920x300 live plot of the position error ||e_p|| (Gazebo ground truth) with a cursor and the time.
Synchronization as in compose_video.py: camera frames carry the simulation time, the scenario start
of each run is read from its ULog; the wind of each run is started by ijc_flight.py a fixed simulated
time before its scenario start (the offsets are printed).
"""
import csv
import os
import subprocess
import sys

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np  # noqa: E402
import imageio_ffmpeg  # noqa: E402

import postprocess as pp  # noqa: E402
from compose_video import ass_time, scenario_start  # noqa: E402

from paths import ROOT  # noqa: E402  (repository root)
LOGS = f'{ROOT}/logs/video'
OUT = f'{ROOT}/results/videos'
FPS = 30
T = 40.0
COL_W, COL_H = 640, 780
W, H = 1920, 1080
PLOT_H = H - COL_H
TITLE_S = 3.0
FONTS = '/usr/share/fonts/truetype/dejavu'
CASES = [('comp', 'With disturbance compensation', dict(color='#1f4fd1', ls='-', lw=2.2)),
         ('nocomp', 'Without compensation', dict(color='#d62020', ls=(0, (5, 3)), lw=2.2)),
         ('px4', 'PX4 (tuned)', dict(color='k', ls=(0, (7, 3, 1.5, 3)), lw=2.0))]
WIND = ('Wind gust fronts: steps of +2.4 m/s (0.2 s rise, held 4 s) every 6.5 s, sudden ±45° direction changes,\\N'
        'slow and fast gusts, turbulence  ·  mean {mean:.1f} m/s, peak {max:.1f} m/s')
KIND = {
    'hover': dict(title='Hover under aggressive wind gusts',
                  legend='Green sphere: position reference   Orange: last 5 s of the trajectory   '
                         'Blue arrow: instantaneous wind'),
    'circle': dict(title='Circular trajectory under aggressive wind gusts',
                   sub='Radius 2 m, period 20 s, altitude 2.5 m',
                   legend='Yellow ring: desired circle at the flight altitude   Green sphere: desired position   '
                          'Orange: last 20 s (one lap)   Blue arrow: instantaneous wind'),
}


def load_case(k, case, n_frames):
    ulog = f'{LOGS}/{k}_{case}.ulg'
    t0 = scenario_start(ulog)
    with open(f'{LOGS}/frames_{k}_{case}/frames.csv') as f:
        rows = list(csv.DictReader(f))
    files = [r['file'] for r in rows]
    ts = np.array([float(r['sim_time_s']) for r in rows])
    grid = t0 + np.arange(n_frames) / FPS
    idx = np.clip(np.searchsorted(ts, grid), 1, len(ts) - 1)
    idx = np.where(np.abs(ts[idx - 1] - grid) <= np.abs(ts[idx] - grid), idx - 1, idx)
    gap = np.abs(ts[idx] - grid).max()
    if gap > 1.5 / FPS:
        print(f'warning: {k} {case}: nearest camera frame up to {gap * 1e3:.0f} ms away')
    R = pp.load_run(ulog, T + 0.5)
    e = np.linalg.norm(R['e_p_gt'], axis=1)
    m = pp.metrics('S2', 'C', pp.load_run(ulog, T), False)
    with open(f'{LOGS}/wind_{k}_{case}.csv') as f:   # sim t, wind t, e, n, u (last line may be cut when gz stops)
        w = np.array([[float(x) for x in ln.split(',')] for ln in f.readlines()[1:] if ln.count(',') == 4 and ln.endswith('\n')])
    wind_t0 = np.interp(t0, w[:, 0], w[:, 1])
    sel = (w[:, 0] >= t0) & (w[:, 0] <= t0 + T)
    return dict(t0=t0, frames=[f'{LOGS}/frames_{k}_{case}/{files[i]}' for i in idx], n_cam=len(rows),
                t=R['t'], e=e, rms=float(np.sqrt(np.mean(e[R['t'] <= T] ** 2))), emax=float(e[R['t'] <= T].max()),
                unalloc=m['unalloc_pct'], wind_t0=wind_t0,
                wind_t=w[sel, 0] - t0, wind_speed=np.hypot(w[sel, 2], w[sel, 3]),
                wind_dir=np.degrees(np.arctan2(w[sel, 3], w[sel, 2])))


def plot_frames(D, n_frames):
    """Live error plot, one 1920x300 RGB frame per output frame (generator)."""
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 16, 'xtick.labelsize': 15, 'ytick.labelsize': 15,
                         'axes.labelsize': 17, 'mathtext.fontset': 'dejavusans'})
    fig = plt.figure(figsize=(W / 100, PLOT_H / 100), dpi=100, facecolor='white')
    canvas = FigureCanvasAgg(fig)   # postprocess selects the pdf backend
    ax = fig.add_axes([0.045, 0.2, 0.6, 0.74])
    ymax = 1.08 * max(d['e'].max() for d in D.values())
    ax.set_xlim(0, T)
    ax.set_ylim(0, ymax)
    ax.set_xlabel('t (s)', labelpad=1)
    ax.set_ylabel(r'$\|e_p\|$ (m)')
    ax.grid(True, color='0.85', lw=0.8)
    lines, dots = {}, {}
    for case, label, sty in CASES:
        d = D[case]
        lines[case], = ax.plot([], [], label=f'{label}  (RMS {d["rms"]:.2f} m)', **sty)
        dots[case], = ax.plot([], [], 'o', color=sty['color'], ms=6)
    cursor = ax.axvline(0, color='0.35', lw=1.2)
    ax.legend(loc='upper left', bbox_to_anchor=(1.012, 1.04), frameon=False, fontsize=15.5, handlelength=3.2)
    ttxt = fig.text(0.665, 0.1, '', fontsize=30, family='DejaVu Sans Mono', weight='bold')
    fig.text(0.665, 0.4, 'Position error (Gazebo ground truth)', fontsize=15, color='0.3')
    canvas.draw()
    for k in range(n_frames):
        tk = k / FPS
        for case, _, _ in CASES:
            d = D[case]
            n = np.searchsorted(d['t'], tk, side='right')
            lines[case].set_data(d['t'][:n], d['e'][:n])
            dots[case].set_data([tk], [np.interp(tk, d['t'], d['e'])])
        cursor.set_xdata([tk, tk])
        ttxt.set_text(f't = {tk:5.2f} s')
        canvas.draw()
        yield np.asarray(canvas.buffer_rgba())[:, :, :3].tobytes()
    plt.close(fig)


def write_ass(path, events, w, h):
    head = [
        '[Script Info]', 'ScriptType: v4.00+', f'PlayResX: {w}', f'PlayResY: {h}', 'WrapStyle: 2', '',
        '[V4+ Styles]',
        'Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, '
        'Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding',
        'Style: Label,DejaVu Sans,30,&H00FFFFFF,&H00FFFFFF,&H00000000,&H80000000,1,0,0,0,100,100,0,0,3,9,0,8,10,10,20,1',
        'Style: Title,DejaVu Sans,62,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,1,0,0,0,100,100,0,0,1,0,0,5,60,60,0,1',
        'Style: Sub,DejaVu Sans,36,&H00E0E0E0,&H00FFFFFF,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,0,0,5,60,60,0,1',
        'Style: Small,DejaVu Sans,27,&H00B8B8B8,&H00FFFFFF,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,0,0,5,60,60,0,1',
        '', '[Events]', 'Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text']
    with open(path, 'w') as f:
        f.write('\n'.join(head + events) + '\n')


def main():
    k = sys.argv[1]
    cfg = KIND[k]
    n_frames = int(round(T * FPS))
    work = f'{LOGS}/compose_{k}'
    os.makedirs(work, exist_ok=True)
    os.makedirs(OUT, exist_ok=True)

    D = {case: load_case(k, case, n_frames) for case, _, _ in CASES}
    for case, _, _ in CASES:
        d = D[case]
        print(f'{k} {case}: scenario start at sim time {d["t0"]:.3f} s, wind time there {d["wind_t0"]:.3f} s, '
              f'{d["n_cam"]} camera frames, RMS |e_p| {d["rms"]:.3f} m, max {d["emax"]:.3f} m, '
              f'unallocated {d["unalloc"]:.2f} %')
    wt = np.array([D[c]['wind_t0'] for c, _, _ in CASES])
    print(f'wind synchronization: spread of the wind time at the scenario start {1e3 * np.ptp(wt):.0f} ms')
    ws = D['comp']['wind_speed']
    wd = D['comp']['wind_dir']
    print(f'wind (scenario window): speed mean {ws.mean():.2f}, min {ws.min():.2f}, max {ws.max():.2f}, std {ws.std():.2f} m/s; '
          f'direction (ENU) p5 {np.percentile(wd, 5):.0f} deg, p95 {np.percentile(wd, 95):.0f} deg, '
          f'range {wd.min():.0f}..{wd.max():.0f} deg')

    # frame lists for the concat demuxer
    lists = []
    for case, _, _ in CASES:
        lst = f'{work}/{case}.txt'
        with open(lst, 'w') as f:
            for p in D[case]['frames']:
                f.write(f"file '{p}'\nduration {1.0 / FPS:.6f}\n")
            f.write(f"file '{D[case]['frames'][-1]}'\n")
        lists.append(lst)

    # column labels (main part) and title screen
    end = ass_time(T)
    labels = [f'Dialogue: 1,0:00:00.00,{end},Label,,0,0,0,,{{\\pos({int(COL_W * (j + 0.5))},16)\\an8}}{lab}'
              for j, (_, lab, _) in enumerate(CASES)]
    write_ass(f'{work}/labels.ass', labels, W, H)
    wind_line = WIND.format(mean=ws.mean(), max=ws.max())
    tl = ass_time(TITLE_S)
    title = [f'Dialogue: 1,0:00:00.00,{tl},Title,,0,0,0,,{{\\pos({W // 2},{330})\\an5}}{cfg["title"]}']
    if 'sub' in cfg:
        title.append(f'Dialogue: 1,0:00:00.00,{tl},Sub,,0,0,0,,{{\\pos({W // 2},{415})\\an5}}{cfg["sub"]}')
    title += [f'Dialogue: 1,0:00:00.00,{tl},Sub,,0,0,0,,{{\\pos({W // 2},{560})\\an5}}{wind_line}',
              f'Dialogue: 1,0:00:00.00,{tl},Sub,,0,0,0,,{{\\pos({W // 2},{700})\\an5}}Quad tail-sitter  ·  PX4 SITL + Gazebo',
              f'Dialogue: 1,0:00:00.00,{tl},Small,,0,0,0,,{{\\pos({W // 2},{790})\\an5}}'
              'With disturbance compensation  |  Without compensation  |  PX4 (tuned)',
              f'Dialogue: 1,0:00:00.00,{tl},Small,,0,0,0,,{{\\pos({W // 2},{840})\\an5}}{cfg["legend"]}']
    write_ass(f'{work}/title.ass', title, W, H)

    # ffmpeg: title (lavfi + ASS) ++ [3 columns hstack / live plot (stdin)] vstack + labels
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    cmd = [ff, '-y', '-hide_banner', '-loglevel', 'error']
    for lst in lists:
        cmd += ['-f', 'concat', '-safe', '0', '-i', lst]
    cmd += ['-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{PLOT_H}', '-r', str(FPS), '-i', '-']
    cmd += ['-f', 'lavfi', '-i', f'color=c=0x14202c:s={W}x{H}:r={FPS}:d={TITLE_S}']
    fc = (''.join(f'[{j}:v]fps={FPS},scale={COL_W}:{COL_H},setsar=1,trim=end_frame={n_frames},setpts=PTS-STARTPTS[c{j}];'
                  for j in range(3))
          + f'[3:v]setsar=1,trim=end_frame={n_frames},setpts=PTS-STARTPTS[pl];'
          + '[c0][c1][c2]hstack=inputs=3[top];[top][pl]vstack=inputs=2[m0];'
          + f"[m0]ass='{work}/labels.ass':fontsdir='{FONTS}',format=yuv420p[main];"
          + f"[4:v]ass='{work}/title.ass':fontsdir='{FONTS}',setsar=1,format=yuv420p[title];"
          + '[title][main]concat=n=2:v=1:a=0[v]')
    out = f'{OUT}/{k}_aggressive_comparison.mp4'
    cmd += ['-filter_complex', fc, '-map', '[v]', '-r', str(FPS),
            '-c:v', 'libx264', '-preset', 'slow', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', out]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for buf in plot_frames(D, n_frames):
        p.stdin.write(buf)
    p.stdin.close()
    if p.wait() != 0:
        sys.exit('ffmpeg failed')
    print('written', out)

    # review frames: scenario start (t = 1 s) and middle (t = 20 s)
    for tag, ts in (('start', 1.0), ('middle', T / 2)):
        png = f'{OUT}/{k}_aggressive_{tag}.png'
        subprocess.run([ff, '-y', '-hide_banner', '-loglevel', 'error', '-ss', f'{TITLE_S + ts:.3f}', '-i', out,
                        '-frames:v', '1', png], check=True)
        print('written', png)


if __name__ == '__main__':
    main()
