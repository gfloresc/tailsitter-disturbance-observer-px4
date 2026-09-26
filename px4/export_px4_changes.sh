#!/usr/bin/env bash
# Regenerate px4/patches/ and px4/overlay/ from a PX4 working tree (for maintainers).
#   px4/export_px4_changes.sh [PX4_DIR]      (default: $PX4_DIR or <repo>/PX4-Autopilot)
# Compares the working tree (tracked files, committed or not) with the base commits of px4/VERSION:
#   modified files of PX4           -> patches/px4-<tag>.patch
#   modified files of the submodule -> patches/px4-gazebo-models.patch
#   new files (both)                -> overlay/<same path as in the PX4 tree>
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck disable=SC1091
source "$HERE/VERSION"
SRC=${1:-${PX4_DIR:-$(dirname "$HERE")/PX4-Autopilot}}
GZ="$SRC/$GZ_MODELS_PATH"

for d in "$SRC" "$GZ"; do
	if [ -n "$(git -C "$d" ls-files --others --exclude-standard)" ]; then
		echo "untracked files in $d (add them to git or remove them):"; git -C "$d" ls-files --others --exclude-standard; exit 1
	fi
done
if [ -n "$(git -C "$SRC" diff --name-only --diff-filter=DR "$PX4_COMMIT")$(git -C "$GZ" diff --name-only --diff-filter=DR "$GZ_MODELS_COMMIT")" ]; then
	echo "deleted/renamed files are not supported by this export"; exit 1
fi

rm -rf "$HERE/patches" "$HERE/overlay"
mkdir -p "$HERE/patches" "$HERE/overlay"
git -C "$SRC" diff --binary --diff-filter=M "$PX4_COMMIT" -- . ":(exclude)$GZ_MODELS_PATH" > "$HERE/patches/px4-$PX4_TAG.patch"
git -C "$GZ" diff --binary --diff-filter=M "$GZ_MODELS_COMMIT" > "$HERE/patches/px4-gazebo-models.patch"
git -C "$SRC" diff --name-only --diff-filter=A "$PX4_COMMIT" | while read -r f; do
	mkdir -p "$HERE/overlay/$(dirname "$f")"; cp -p "$SRC/$f" "$HERE/overlay/$f"
done
git -C "$GZ" diff --name-only --diff-filter=A "$GZ_MODELS_COMMIT" | while read -r f; do
	mkdir -p "$HERE/overlay/$GZ_MODELS_PATH/$(dirname "$f")"; cp -p "$GZ/$f" "$HERE/overlay/$GZ_MODELS_PATH/$f"
done
echo "patches:"; for p in "$HERE"/patches/*.patch; do echo "  $(basename "$p"): $(grep -c '^diff --git' "$p") files"; done
echo "overlay: $(find "$HERE/overlay" -type f | wc -l) files"
