#!/usr/bin/env bash
# ROS 2 environment hooks are not guaranteed to be compatible with `set -u`.
set -eo pipefail

# Source ROS and the workspace without depending on an interactive shell.
source "/opt/ros/${ROS_DISTRO}/setup.bash"

workspace_name="${WORKSPACE:-mulinex_ws}"
workspace_setup="${HOME}/${workspace_name}/install/setup.bash"

if [ -f "${workspace_setup}" ]; then
    source "${workspace_setup}"
fi

# GUI and audio tools may require a private runtime directory.
if [ -n "${XDG_RUNTIME_DIR:-}" ]; then
    mkdir -p "${XDG_RUNTIME_DIR}"
    chmod 700 "${XDG_RUNTIME_DIR}"
fi

exec "$@"
