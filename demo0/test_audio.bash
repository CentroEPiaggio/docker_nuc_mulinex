#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "${HOME}/mulinex_ws/install/setup.bash"

share=$(ros2 pkg prefix --share socialbot_demo0)
paplay "${share}/assets/audio/introduction.wav"
