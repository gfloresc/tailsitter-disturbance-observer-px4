#!/usr/bin/env bash
# Comparison videos (results/videos/): record the camera runs, then compose with ffmpeg.
#   make_videos.sh [hover circle S1 S2]   cases comp / nocomp / px4 (default: hover circle)
#     hover, circle: aggressive gusts, fixed camera, live error plot (compose_aggressive.py)
#     S1, S2:        first version, follow camera (compose_video.py)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/paths.sh"   # S, ROOT, PX4 (IJC_ROOT exported)
GUST="fronts_sharp 0.85"   # disturbance profile and scale chosen with calib_gust.sh (results/videos/README.md)
for SC in ${*:-hover circle}; do
	case $SC in
	hover|circle)
		python3 $S/make_video_worlds.py $GUST
		$S/record_videos.sh $SC comp nocomp px4
		python3 $S/compose_aggressive.py $SC ;;
	*)
		$S/record_videos.sh $SC comp nocomp px4
		python3 $S/compose_video.py $SC ;;
	esac
done
