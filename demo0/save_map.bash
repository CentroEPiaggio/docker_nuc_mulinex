#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/mulinex_ws/install/setup.bash"

output="${HOME}/mulinex_ws/src/socialbot_demo0/maps/my_demo0_map"
ros2 run nav2_map_server map_saver_cli -f "$output"
echo "Map saved as ${output}.pgm and ${output}.yaml"
