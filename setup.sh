#!/usr/bin/env bash
# Reproducible setup: official PX4 v1.17.0 + the patches and files of this repository, then build.
#
#   ./setup.sh               clone into ./PX4-Autopilot, patch, copy our files, build px4_sitl_default
#   ./setup.sh --no-build    same without the build
#   PX4_DIR=/path ./setup.sh use another location (the scripts then need the same PX4_DIR)
#
# Prerequisites (once, see README): Ubuntu 22.04, PX4's Tools/setup/ubuntu.sh (toolchain and
# Gazebo Harmonic), Python packages of requirements.txt. This script does not use sudo.
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$ROOT/px4/VERSION"
PX4_DIR=${PX4_DIR:-$ROOT/PX4-Autopilot}
BUILD=1
[ "${1:-}" = "--no-build" ] && BUILD=0

if [ -e "$PX4_DIR" ]; then
	echo "$PX4_DIR already exists: remove it (or set PX4_DIR) to set up from scratch."
	exit 1
fi

echo "== cloning PX4-Autopilot $PX4_TAG (with submodules)"
git clone --branch "$PX4_TAG" "$PX4_REPO" "$PX4_DIR"
cd "$PX4_DIR"
if [ "$(git rev-parse HEAD)" != "$PX4_COMMIT" ]; then
	echo "tag $PX4_TAG does not point to the expected commit $PX4_COMMIT"; exit 1
fi
git submodule update --init --recursive --jobs 8
if [ "$(git -C "$GZ_MODELS_PATH" rev-parse HEAD)" != "$GZ_MODELS_COMMIT" ]; then
	echo "$GZ_MODELS_PATH is not at the expected commit $GZ_MODELS_COMMIT"; exit 1
fi

echo "== applying patches"
git apply "$ROOT/px4/patches/px4-$PX4_TAG.patch"
git -C "$GZ_MODELS_PATH" apply "$ROOT/px4/patches/px4-gazebo-models.patch"

echo "== copying our files (px4/overlay)"
cp -r "$ROOT/px4/overlay/." "$PX4_DIR/"

echo "== generating the ground texture of the video/demo worlds"
PX4_DIR="$PX4_DIR" python3 -c "import sys; sys.path.insert(0, '$ROOT/scripts'); import make_video_worlds as m; m.make_ground()"

if [ $BUILD = 1 ]; then
	echo "== building px4_sitl_default"
	make px4_sitl_default
fi
echo "== done: PX4 in $PX4_DIR (v1.17.0 + our changes, uncommitted in its working tree)"
