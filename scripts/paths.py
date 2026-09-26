"""Common paths of the Python scripts: the repository root is the parent of scripts/.

ROOT  repository root (logs/, results/ live here)
PX4   PX4-Autopilot tree: $PX4_DIR if set, else <root>/PX4-Autopilot (created by setup.sh)
"""
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PX4 = os.environ.get('PX4_DIR', os.path.join(ROOT, 'PX4-Autopilot'))
