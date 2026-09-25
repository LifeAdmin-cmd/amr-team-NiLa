#!/bin/bash

# ===== User configuration =====
HUMBLE_PATH="/opt/ros/humble"
ROS2_PATH="$HOME/ros2_ws"

# Allow use to override linuxbrew/other pythons shadowing system python (breaks rclpy for me)
# uncomment if not needed.
export PATH="/usr/bin:$PATH"

if [ -f "$HUMBLE_PATH/setup.bash" ]; then
    source "$HUMBLE_PATH/setup.bash"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    source "/opt/ros/iron/setup.bash"
fi

if [ -f "$ROS2_PATH/install/setup.bash" ]; then
    source "$ROS2_PATH/install/setup.bash"
fi

# Add src to PYTHONPATH so python can find the modules
ENV_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export PYTHONPATH="$PYTHONPATH:$ENV_DIR/../src"

export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_network_config.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
