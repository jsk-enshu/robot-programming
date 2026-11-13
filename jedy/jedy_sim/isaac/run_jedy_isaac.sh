#!/usr/bin/env bash
# Run jedy_isaac.py headless in the Isaac Sim container.
#
#   USD=/path/jedy_fullbody.usdc OUT=/path/out ./run_jedy_isaac.sh
#
# NET_CONTAINER: share the network namespace of the container that runs the
# ROS 2 side (default jedy-dev), so DDS discovery is local. Its /dev/shm is NOT
# shared, and Fast DDS would pick shared memory for a same-host peer and then
# deliver nothing (discovery works, data never arrives), so this side is
# restricted to UDP. Set NET_CONTAINER= to use the host network instead.
# ISAAC_ARGS: extra arguments for jedy_isaac.py (e.g. "--ground-z -10").
# GPU: which GPU Isaac may use (default 2; on a shared multi-GPU machine,
# set it to one nobody else is using).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# 5.0.0: the image these scripts were verified on; 6.0.0 drops isaacsim.core.api.
ISAAC_TAG="${ISAAC_TAG:-5.0.0}"
USD="${USD:?set USD to the .usdc from make_jedy_usd.py}"
OUT="${OUT:?set OUT to a directory for isaac_view.png}"
GPU="${GPU:-2}"
CACHE="${CACHE:-${HOME}/docker/isaac-sim}"
NET_CONTAINER="${NET_CONTAINER-jedy-dev}"
mkdir -p "$OUT" "${CACHE}/kit" "${CACHE}/ov" "${CACHE}/pip" "${CACHE}/glcache" \
         "${CACHE}/computecache" "${CACHE}/logs" "${CACHE}/data"
if [ -n "$NET_CONTAINER" ]; then
  NET=(--network "container:${NET_CONTAINER}")
else
  NET=(--network host)
fi
exec docker run --name jedy-isaac --rm \
  --gpus "\"device=${GPU}\"" "${NET[@]}" \
  --entrypoint bash \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
  -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -v "${CACHE}/kit:/isaac-sim/kit/cache:rw" \
  -v "${CACHE}/ov:/root/.cache/ov:rw" \
  -v "${CACHE}/pip:/root/.cache/pip:rw" \
  -v "${CACHE}/glcache:/root/.cache/nvidia/GLCache:rw" \
  -v "${CACHE}/computecache:/root/.nv/ComputeCache:rw" \
  -v "${CACHE}/logs:/root/.nvidia-omniverse/logs:rw" \
  -v "${CACHE}/data:/root/.local/share/ov/data:rw" \
  -v "${HERE}:/jedy_isaac:ro" \
  -v "$(dirname "$USD"):/usd:ro" \
  -v "${OUT}:/out:rw" \
  "nvcr.io/nvidia/isaac-sim:${ISAAC_TAG}" \
  -lc "export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib\"; \
       ./python.sh /jedy_isaac/jedy_isaac.py --usd /usd/$(basename "$USD") --out /out ${ISAAC_ARGS:-}"
