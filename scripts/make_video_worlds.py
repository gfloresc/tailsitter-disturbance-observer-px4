#!/usr/bin/env python3
"""Generate the worlds and the ground model of the disturbance demonstration videos.

usage: make_video_worlds.py [profile] [scale]   (profile from PROFILES; scale multiplies the wind speeds;
                                                 default: the profile chosen for the videos)

Writes into PX4-Autopilot/Tools/simulation/gz (or $IJC_GZ_DIR):
  worlds/ijc_gust.sdf               ijc_calm physics + the disturbance (calibration runs, no camera)
  worlds/ijc_gust_hover_video.sdf   + fixed close camera on the hover reference, 5 s trail, wind arrow,
                                      textured ground, sky, shadows
  worlds/ijc_gust_circle_video.sdf  + near-overhead camera, desired circle as a thin ring at the flight
                                      altitude, desired-position sphere,
                                      one-lap trail, wind arrow
  models/ijc_ground/                textured ground patch (1 m grid)
"""
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

from paths import PX4  # noqa: E402

GZ = os.environ.get('IJC_GZ_DIR', os.path.join(PX4, 'Tools/simulation/gz'))

# ---- disturbance profiles (IjcGustSystem); speeds in m/s (multiplied by the scale argument), wind time in s.
# The wind starts 8 s (simulated) before the scenario start: the videos show wind time 8-48 s.
BASE = dict(mean_direction_deg=70.0,             # towards NNE (ENU): mostly along the body x axis
            turbulence_tau=0.3, rise_time=5.0, seed=7)


def events(t0, period, n, make):
    return [make(k, t0 + period * k) for k in range(n)]


def fronts(dspeed, ddir, rise=0.3, hold=3.0, fall=1.2, t0=10.0, dt0=3.0, drise=0.4, dhold=2.5, dfall=0.8):
    """Step-like gust fronts every 6.5 s and, 3 s after each, a sudden direction change of alternating sign."""
    sp = events(t0, 6.5, 6, lambda k, t: (t, rise, hold, fall, dspeed, 0.0))
    di = events(t0 + dt0, 6.5, 6, lambda k, t: (t, drise, dhold, dfall, 0.0, ddir * (1 if k % 2 == 0 else -1)))
    return sp + di


def pulses(force, torque, t0=12.0, fdur=0.6, tdur=0.4):
    """Short external pulses: horizontal force (random direction) and roll/pitch torque (random sign)."""
    rng = np.random.default_rng(11)
    out = []
    for k in range(6):
        a = rng.uniform(0, 2 * np.pi)
        out.append((t0 + 6.5 * k, fdur, force * np.cos(a), force * np.sin(a), 0.0, 0.0, 0.0, 0.0))
        tx, ty = torque * rng.choice([-1, 1]), torque * rng.choice([-1, 1]) * 0.7
        if torque > 0:
            out.append((t0 + 6.5 * k + 3.2, tdur, 0.0, 0.0, 0.0, tx, ty, 0.0))
    return out


MILD_SINES = dict(speed_sines=[(0.8, 6.5, 0.0), (0.3, 5.1, 1.3), (0.4, 2.3, 0.4), (0.25, 1.3, 2.1)],
                  dir_sines=[(12.0, 9.0, 0.0), (6.0, 2.7, 0.9)],
                  turbulence_sigma=0.3, turbulence_sigma_z=0.12, direction_noise_deg=3.0)
PROFILES = {
    # first aggressive version: multi-frequency sines only (0.5 x the 7 m/s nominal profile)
    'sines': dict(mean_speed=3.5, speed_sines=[(1.25, 6.5, 0.0), (0.5, 5.1, 1.3), (0.6, 2.3, 0.4), (0.4, 1.3, 2.1)],
                  dir_sines=[(35.0, 9.0, 0.0), (15.0, 2.7, 0.9)],
                  turbulence_sigma=0.4, turbulence_sigma_z=0.15, direction_noise_deg=5.0),
    'fronts': dict(mean_speed=3.0, **MILD_SINES, fronts=fronts(2.5, 40.0)),
    'fronts_strong': dict(mean_speed=3.0, **MILD_SINES, fronts=fronts(3.5, 55.0)),
    'dirsteps': dict(mean_speed=3.5, **MILD_SINES,
                     fronts=events(10.0, 5.0, 8, lambda k, t: (t, 0.3, 3.5, 0.5, 0.0, 60.0 * (1 if k % 2 == 0 else -1)))),
    'fronts_pulses': dict(mean_speed=3.0, **MILD_SINES, fronts=fronts(2.5, 40.0), pulses=pulses(5.0, 0.4)),
    'sines_pulses': dict(mean_speed=3.0, **MILD_SINES, pulses=pulses(8.0, 0.6)),
    'fronts_fpulses': dict(mean_speed=3.0, **MILD_SINES, fronts=fronts(2.5, 40.0), pulses=pulses(4.0, 0.0, fdur=0.5)),
    'fronts_sharp': dict(mean_speed=2.8, **MILD_SINES, fronts=fronts(2.8, 45.0, rise=0.2, hold=4.0, fall=1.0, drise=0.3)),
}
# chosen for the videos (largest RMS advantage over PX4 tuned with <= 5 % saturation): fronts_sharp x 0.85
PROFILE = 'fronts_sharp'
GUST_SCALE = 0.85

CAM_W, CAM_H = 640, 780
HOVER_LOOK = '0.1 0 2.6'          # look-at point (from the takeoff point): just above the hover reference
HOVER_CAM = '1.32 -1.574 0.437'   # 2.1 m from it, azimuth -50 deg, elevation 12 deg (all runs stay in frame)
HOVER_ARROW = '-0.7 0.9 1.0'      # low, behind the reference: bottom of the image, seen from above
CIRCLE_CAM = '1.25 -1.46 7.15'    # 7.4 m from the circle centre, 75 deg elevation
CIRCLE_HFOV = 0.84


def gust_plugin(log_file):
    g = dict(BASE, **PROFILES[PROFILE])
    sines = ''.join(f'      <speed_sine amp="{a}" period="{T}" phase="{p}"/>\n' for a, T, p in g['speed_sines'])
    dirs = ''.join(f'      <dir_sine amp_deg="{a}" period="{T}" phase="{p}"/>\n' for a, T, p in g['dir_sines'])
    fr = ''.join(f'      <front t="{t:.3f}" rise="{r}" hold="{h}" fall="{fa}" dspeed="{ds}" ddir_deg="{dd}"/>\n'
                 for t, r, h, fa, ds, dd in g.get('fronts', []))
    pu = ''.join(f'      <pulse t="{t:.3f}" dur="{d}" fx="{fx:.3f}" fy="{fy:.3f}" fz="{fz:.3f}" tx="{tx:.3f}" ty="{ty:.3f}" tz="{tz:.3f}"/>\n'
                 for t, d, fx, fy, fz, tx, ty, tz in g.get('pulses', []))
    if pu:
        pu += '      <pulse_model>quadtailsitter_ijc_0</pulse_model>\n      <pulse_link>base_link</pulse_link>\n'
    return f'''    <!-- disturbance profile '{PROFILE}' x {GUST_SCALE} (IjcGustSystem), runs before the model systems -->
    <plugin filename="libIjcGustSystem.so" name="custom::IjcGustSystem">
      <gz:system_priority>-10</gz:system_priority>
      <scale>{GUST_SCALE}</scale>
      <mean_speed>{g['mean_speed']}</mean_speed>
      <mean_direction_deg>{g['mean_direction_deg']}</mean_direction_deg>
{sines}{dirs}{fr}{pu}      <turbulence_sigma>{g['turbulence_sigma']}</turbulence_sigma>
      <turbulence_sigma_z>{g['turbulence_sigma_z']}</turbulence_sigma_z>
      <turbulence_tau>{g['turbulence_tau']}</turbulence_tau>
      <direction_noise_deg>{g['direction_noise_deg']}</direction_noise_deg>
      <rise_time>{g['rise_time']}</rise_time>
      <seed>{g['seed']}</seed>
      <trigger_topic>/ijc_gust_start</trigger_topic>
      <log_file>{log_file}</log_file>
    </plugin>
'''


def make_ground():
    """32 m x 32 m textured patch: grass-like noise, 1 m grid, 5 m major lines (64 px/m)."""
    d = f'{GZ}/models/ijc_ground'
    os.makedirs(f'{d}/materials/textures', exist_ok=True)
    n, ppm = 2048, 64
    rng = np.random.default_rng(3)
    base = np.array([112, 138, 86], float)
    noise = rng.normal(0, 1, (n // 8, n // 8))
    noise = np.kron(noise, np.ones((8, 8))) * 7 + rng.normal(0, 4, (n, n))
    img = np.clip(base[None, None, :] + noise[..., None] * np.array([0.8, 1.0, 0.6]), 0, 255).astype(np.uint8)
    im = Image.fromarray(img)
    dr = ImageDraw.Draw(im)
    for k in range(0, n + 1, ppm):
        major = (k // ppm) % 5 == 0
        c, w = ((225, 225, 215), 5) if major else ((190, 196, 176), 2)
        dr.line([(k, 0), (k, n)], fill=c, width=w)
        dr.line([(0, k), (n, k)], fill=c, width=w)
    im.save(f'{d}/materials/textures/ground.png')
    with open(f'{d}/model.config', 'w') as f:
        f.write('<?xml version="1.0"?>\n<model><name>ijc_ground</name><version>1.0</version><sdf version="1.9">model.sdf</sdf>'
                '<description>Textured ground patch (32 m, 1 m grid) for the ijc_control videos</description></model>\n')
    with open(f'{d}/model.sdf', 'w') as f:
        f.write('''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="ijc_ground">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>32 32</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr><metal><albedo_map>materials/textures/ground.png</albedo_map><roughness>0.95</roughness><metalness>0</metalness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
''')


def camera(pose_dummy, hfov):
    return f'''    <model name="ijc_cam">
      <static>true</static>
      <pose>{pose_dummy}</pose>
      <link name="link">
        <sensor name="cam" type="camera">
          <always_on>1</always_on>
          <update_rate>30</update_rate>
          <visualize>false</visualize>
          <topic>ijc_cam</topic>
          <camera>
            <horizontal_fov>{hfov}</horizontal_fov>
            <image><width>{CAM_W}</width><height>{CAM_H}</height><format>R8G8B8</format></image>
            <clip><near>0.05</near><far>300</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
'''


def static_visual(name, geometry, color, alpha=0.0, pose='0 0 -20 0 0 0', vpose='0 0 0 0 0 0', emissive=0.3, shadows=True):
    c = ' '.join(f'{x:.3f}' for x in color)
    e = ' '.join(f'{x * emissive:.3f}' for x in color)
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="v">
          <pose>{vpose}</pose>
          <cast_shadows>{'true' if shadows else 'false'}</cast_shadows>
          <geometry>{geometry}</geometry>
          <transparency>{alpha}</transparency>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse><emissive>{e} 1</emissive></material>
        </visual>
      </link>
    </model>
'''


def trail_models(n=50):   # n x 0.1 s
    return ''.join(static_visual(f'ijc_trail_{i}', '<sphere><radius>0.02</radius></sphere>', (1.0, 0.3, 0.0),
                                 alpha=0.15 + 0.7 * i / n, shadows=False) for i in range(n))


def arrow_models(n=25, seg=0.1):
    blue = (0.1, 0.3, 1.0)
    s = ''.join(static_visual(f'ijc_arrow_seg_{i}', f'<cylinder><radius>0.03</radius><length>{seg}</length></cylinder>', blue,
                              vpose=f'{seg / 2} 0 0 0 1.5708 0', emissive=0.4, shadows=False) for i in range(n))
    return s + static_visual('ijc_arrow_head', '<cone><radius>0.08</radius><length>0.22</length></cone>', blue,
                             vpose='0.11 0 0 0 1.5708 0', emissive=0.4, shadows=False)


def ring_model(name='ijc_circle', center=(0.0, -2.0), R=2.0, n=90, z=0.012, width=0.07, height=0.004, alpha=0.0,
               color=(1.0, 0.9, 0.1), pose='0 0 0 0 0 0'):
    c = ' '.join(f'{x:.3f}' for x in color)
    e = ' '.join(f'{0.4 * x:.3f}' for x in color)
    vis = []
    for k in range(n):
        a = 2 * math.pi * k / n
        x, y = center[0] + R * math.cos(a), center[1] + R * math.sin(a)
        L = 2 * math.pi * R / n * 1.05
        vis.append(f'''        <visual name="s{k}"><pose>{x:.4f} {y:.4f} {z} 0 0 {a + math.pi / 2:.4f}</pose><cast_shadows>false</cast_shadows>
          <geometry><box><size>{L:.4f} {width} {height}</size></box></geometry><transparency>{alpha}</transparency>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse><emissive>{e} 1</emissive></material></visual>''')
    return f'''    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
{chr(10).join(vis)}
      </link>
    </model>
'''


def video_plugin(look_at, cam_offset, reference, arrow_anchor, arrow_scale, trail=50, extra=''):
    ref = '      <reference_model>ijc_ref</reference_model>\n      <reference_height>2.5</reference_height>\n' if reference else \
          '      <reference_model></reference_model>\n'
    return f'''    <plugin filename="libIjcVideoSystem.so" name="custom::IjcVideoSystem">
      <model_name>quadtailsitter_ijc_0</model_name>
      <link_name>base_link</link_name>
      <follow>false</follow>
      <camera_model>ijc_cam</camera_model>
      <look_at_offset>{look_at}</look_at_offset>
      <camera_offset>{cam_offset}</camera_offset>
{ref}      <trail_prefix>ijc_trail_</trail_prefix>
      <trail_count>{trail}</trail_count>
      <trail_period>0.1</trail_period>
      <arrow_segment_prefix>ijc_arrow_seg_</arrow_segment_prefix>
      <arrow_segment_count>25</arrow_segment_count>
      <arrow_segment_length>0.1</arrow_segment_length>
      <arrow_head_model>ijc_arrow_head</arrow_head_model>
      <arrow_anchor>{arrow_anchor}</arrow_anchor>
      <arrow_scale>{arrow_scale}</arrow_scale>
{extra}    </plugin>
'''


def main():
    global PROFILE, GUST_SCALE
    if len(sys.argv) > 1:
        PROFILE = sys.argv[1]
    if len(sys.argv) > 2:
        GUST_SCALE = float(sys.argv[2])
    base = open(f'{GZ}/worlds/ijc_calm.sdf').read()
    make_ground()
    anchor = '    <plugin filename="libIjcWrenchSystem.so" name="custom::IjcWrenchSystem">'
    assert base.count(anchor) == 1
    logs = 'logs/video'   # relative: IjcGustSystem resolves it against $IJC_ROOT (scripts/paths.sh)

    # calibration world
    w = base.replace('<world name="ijc_calm">', '<world name="ijc_gust">\n    <!-- generated by scripts/make_video_worlds.py -->', 1)
    w = w.replace(anchor, gust_plugin(f'{logs}/wind_ijc_gust.csv') + anchor)
    w = w.replace('<gravity>0 0 -9.80665</gravity>', '<gravity>0 0 -9.80665</gravity>\n    <wind><linear_velocity>0 0 0</linear_velocity></wind>')
    open(f'{GZ}/worlds/ijc_gust.sdf', 'w').write(w)

    for kind in ('hover', 'circle'):
        name = f'ijc_gust_{kind}_video'
        v = base.replace('<world name="ijc_calm">', f'<world name="{name}">\n    <!-- generated by scripts/make_video_worlds.py -->', 1)
        v = v.replace(anchor, gust_plugin(f'{logs}/wind_{name}.csv') + anchor)
        v = v.replace('<gravity>0 0 -9.80665</gravity>', '<gravity>0 0 -9.80665</gravity>\n    <wind><linear_velocity>0 0 0</linear_velocity></wind>')
        v = v.replace('<shadows>true</shadows>', '<shadows>true</shadows>\n      <sky></sky>')
        v = v.replace('<ambient>0.4 0.4 0.4 1</ambient>\n      <background>', '<ambient>0.25 0.25 0.25 1</ambient>\n      <background>')   # darker shadows
        v = v.replace('<ambient>0.8 0.8 0.8 1</ambient>\n            <diffuse>0.8 0.8 0.8 1</diffuse>',
                      '<ambient>0.42 0.52 0.34 1</ambient>\n            <diffuse>0.42 0.52 0.34 1</diffuse>')
        if kind == 'hover':
            plug = video_plugin(HOVER_LOOK, HOVER_CAM, True, HOVER_ARROW, 0.17)
            models = (camera('3 -2 3 0 0 0', 0.80)
                      + static_visual('ijc_ref', '<sphere><radius>0.12</radius></sphere>', (0.0, 0.85, 0.1), alpha=0.45, shadows=False)
                      + trail_models() + arrow_models())
        else:
            # near-overhead camera (75 deg elevation, from the south-east) centred on the circle centre at
            # the flight altitude; desired circle as a thin ring at the flight altitude (placed relative to
            # the takeoff point; no copy on the ground: even faint it read as a second ring), a sphere at the instantaneous desired
            # position, one-lap trail (200 x 0.1 s), wind arrow at the centre of the ring
            extra = ('      <attach model="ijc_ring_air" offset="0 -2 2.5"/>\n'
                     '      <desired_model>ijc_des</desired_model>\n      <desired_topic>/ijc_gust_start</desired_topic>\n'
                     '      <desired_lead>8.0</desired_lead>\n      <desired_radius>2.0</desired_radius>\n'
                     '      <desired_period>20.0</desired_period>\n      <desired_ramp>5.0</desired_ramp>\n'
                     '      <reference_height>2.5</reference_height>\n')
            plug = video_plugin('0 -2 2.5', CIRCLE_CAM, False, '0 -2 2.45', 0.23, trail=200, extra=extra)
            models = (camera('4 -6 9 0 0 0', CIRCLE_HFOV)
                      + ring_model('ijc_ring_air', center=(0.0, 0.0), z=0.0, width=0.03, height=0.03, alpha=0.35,
                                   pose='0 0 -20 0 0 0')
                      + static_visual('ijc_des', '<sphere><radius>0.08</radius></sphere>', (0.0, 0.85, 0.1), alpha=0.25,
                                      shadows=False)
                      + trail_models(200) + arrow_models())
        v = v.replace(anchor, plug + anchor)
        # 3 x 3 tiles of the 32 m textured patch (96 m), grid continuous across tiles
        ground = ''.join(f'    <include>\n      <name>ijc_ground_{i}_{j}</name>\n      <uri>model://ijc_ground</uri>\n'
                         f'      <pose>{32 * i} {32 * j} 0.002 0 0 0</pose>\n    </include>\n' for i in (-1, 0, 1) for j in (-1, 0, 1))
        light = '    <light name="sunUTC" type="directional">'
        assert v.count(light) == 1
        # sun almost overhead: the vehicle's shadow falls (0.3 m off) below it, on the desired circle
        sun = '<direction>0.001 0.625 -0.78</direction>'
        assert v.count(sun) == 1
        v = v.replace(sun, '<direction>-0.08 0.1 -0.99</direction>')
        v = v.replace(light, ground + models + light)
        open(f'{GZ}/worlds/{name}.sdf', 'w').write(v)
    print(f'worlds written (profile {PROFILE}, scale {GUST_SCALE})')


if __name__ == '__main__':
    main()
