#!/usr/bin/env python3
"""Record the frames of the Gazebo follow camera (topic /ijc_cam) with their simulation time.

usage: record_camera.py <out_dir>      (runs until SIGINT/SIGTERM)
Writes <out_dir>/<sim_time_ns>.jpg and <out_dir>/frames.csv (file, sim_time_s).
"""
import os
import queue
import signal
import sys
import threading
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from PIL import Image as PImage

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
q = queue.Queue(maxsize=600)
stop = threading.Event()
dropped = [0]


def on_image(msg):
    try:
        q.put_nowait((msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec, msg.width, msg.height, bytes(msg.data)))
    except queue.Full:
        dropped[0] += 1


def writer():
    with open(os.path.join(out, 'frames.csv'), 'w') as index:
        index.write('file,sim_time_s\n')
        while not (stop.is_set() and q.empty()):
            try:
                ns, w, h, data = q.get(timeout=0.2)
            except queue.Empty:
                continue
            name = f'{ns:015d}.jpg'
            PImage.fromarray(np.frombuffer(data, np.uint8).reshape(h, w, 3)).save(os.path.join(out, name), quality=92)
            index.write(f'{name},{ns * 1e-9:.6f}\n')


t = threading.Thread(target=writer)
t.start()
node = Node()
node.subscribe(Image, '/ijc_cam', on_image)
signal.signal(signal.SIGTERM, lambda *a: stop.set())
signal.signal(signal.SIGINT, lambda *a: stop.set())
while not stop.is_set():
    time.sleep(0.2)
t.join()
print(f'recorder: dropped {dropped[0]} frames (queue full)', flush=True)
