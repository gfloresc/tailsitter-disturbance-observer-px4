# Render Gazebo on the NVIDIA GPU (hybrid Intel + NVIDIA laptop, PRIME "on-demand" mode).
# Sourced by demo.sh (GUI, GLX) and record_videos.sh (offscreen camera, EGL). Without the variables the
# GUI renders on the Intel GPU (Mesa); if the NVIDIA driver is not loaded nothing is changed.
if nvidia-smi -L > /dev/null 2>&1; then
	export __NV_PRIME_RENDER_OFFLOAD=1                  # GLX (GUI) on the NVIDIA GPU
	export __GLX_VENDOR_LIBRARY_NAME=nvidia
	export __VK_LAYER_NV_optimus=NVIDIA_only
	if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then   # EGL (--headless-rendering)
		export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
	fi
	GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
else
	GPU_NAME="default (NVIDIA driver not loaded)"
fi
