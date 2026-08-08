#!/bin/bash

SCRIPT_DIR="$(dirname "$0")"	
source "$SCRIPT_DIR/env.bat"


# Start Gazebo in first tab
gnome-terminal --tab --title="Gazebo" -- bash -c "
source '$SCRIPT_DIR/env.bat'
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
"

# Give Gazebo time to start
sleep 10

# Start controller in second tab
gnome-terminal --tab --title="Controller" -- bash -c "
source '$SCRIPT_DIR/env.bat'
python3 '$SCRIPT_DIR/src/controller.py'
exec bash
"
