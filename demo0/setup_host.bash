#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ENV_FILE="${ROOT_DIR}/demo0/.env"
XAUTH="${HOME}/.docker.xauth"

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker is not installed or is not on PATH." >&2
    exit 1
fi

if ! docker compose version >/dev/null 2>&1; then
    echo "Docker Compose v2 is required (docker compose)." >&2
    exit 1
fi

HOST_USER_VALUE=${USER:-$(id -un)}
HOST_UID_VALUE=$(id -u)
HOST_GID_VALUE=$(id -g)
HOST_XDG_RUNTIME_VALUE=${XDG_RUNTIME_DIR:-/run/user/${HOST_UID_VALUE}}
CONTAINER_XDG_RUNTIME_VALUE="/tmp/runtime-${HOST_USER_VALUE}"
DISPLAY_VALUE=${DISPLAY:-:0}

touch "${HOME}/.bash_history"

mkdir -p "$(dirname "$XAUTH")"
touch "$XAUTH"
chmod 666 "$XAUTH"

if command -v xauth >/dev/null 2>&1 && [ -n "${DISPLAY_VALUE}" ]; then
    xauth nlist "${DISPLAY_VALUE}" 2>/dev/null \
        | sed -e 's/^..../ffff/' \
        | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
else
    echo "Warning: xauth is missing; Gazebo/RViz may not open. Install it with: sudo apt install xauth" >&2
fi

PULSE_DIR_VALUE="${HOST_XDG_RUNTIME_VALUE}/pulse"
if [ ! -S "${PULSE_DIR_VALUE}/native" ]; then
    PULSE_DIR_VALUE="/tmp/mulinex-demo0-empty-pulse"
    mkdir -p "$PULSE_DIR_VALUE"
    echo "Warning: no PulseAudio/PipeWire socket found. Simulation will run, but audio playback will fail." >&2
fi

cat > "$ENV_FILE" <<ENVEOF
HOST_USER=${HOST_USER_VALUE}
HOST_UID=${HOST_UID_VALUE}
HOST_GID=${HOST_GID_VALUE}
DISPLAY=${DISPLAY_VALUE}
CONTAINER_XDG_RUNTIME_DIR=${CONTAINER_XDG_RUNTIME_VALUE}
PULSE_DIR=${PULSE_DIR_VALUE}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}
XAUTH_FILE=${XAUTH}
LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-0}
ENVEOF

echo "Host environment written to ${ENV_FILE}"
