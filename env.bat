#!/bin/bash

# ===== User configuration =====
HUMBLE_PATH="/opt/ros/humble"
ROS2_PATH="$HOME/ros2_ws"

# ===== Environment setup =====
source "$HUMBLE_PATH/setup.bash"
source "$ROS2_PATH/install/setup.bash"

# Add src to PYTHONPATH so python can find the modules
ENV_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export PYTHONPATH="$PYTHONPATH:$ENV_DIR/src"
