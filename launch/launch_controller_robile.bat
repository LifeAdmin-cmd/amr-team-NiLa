#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/../config/env.bat"

# Ensure ROS_DOMAIN_ID is preserved across tabs
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_DOMAIN_ID="$DOMAIN_ID"

echo "Launching controller, MCL, and mapping nodes with ROS_DOMAIN_ID=$ROS_DOMAIN_ID..."

# Start controller in first tab
gnome-terminal --tab --title="Controller (Domain $ROS_DOMAIN_ID)" -- bash -c "
source '$SCRIPT_DIR/../config/env.bat'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
python3 '$SCRIPT_DIR/../src/controller.py'
exec bash
"

# Start MCL node in second tab
gnome-terminal --tab --title="Localisation (MCL - Domain $ROS_DOMAIN_ID)" -- bash -c "
source '$SCRIPT_DIR/../config/env.bat'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
python3 '$SCRIPT_DIR/../src/localisation/mcl_node.py'
exec bash
"

# Start Mapping node in third tab
gnome-terminal --tab --title="Mapping (Domain $ROS_DOMAIN_ID)" -- bash -c "
source '$SCRIPT_DIR/../config/env.bat'
export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
python3 '$SCRIPT_DIR/../src/mapping/mapping_node.py'
exec bash
"
