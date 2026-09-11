#!/bin/bash

SCRIPT_DIR="$(dirname "$0")"
source "$SCRIPT_DIR/env.bat"


# Start controller in first tab
gnome-terminal --tab --title="Controller" -- bash -c "
source '$SCRIPT_DIR/env.bat'
python3 '$SCRIPT_DIR/src/controller.py'
exec bash
"

# Start MCL node in second tab
gnome-terminal --tab --title="Localisation (MCL)" -- bash -c "
source '$SCRIPT_DIR/env.bat'
python3 '$SCRIPT_DIR/src/localisation/mcl_node.py'
exec bash
"

# Start Mapping node in third tab
gnome-terminal --tab --title="Mapping" -- bash -c "
source '$SCRIPT_DIR/env.bat'
python3 '$SCRIPT_DIR/src/mapping/mapping_node.py'
exec bash
"
