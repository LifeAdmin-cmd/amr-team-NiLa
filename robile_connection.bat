#!/usr/bin/env bash
###############################################################################
# Robile connection & control script
#
# One file for everything: network setup, SSH, simulation, verification,
# Rviz2, and keyboard teleop. Run it, then press a number for that step.
#
# Usage:
#   chmod +x robile_connection.sh
#   ./robile_connection.sh
###############################################################################
set -o pipefail

# --- Source the existing ROS2 workspace environment (env.bat) --------------
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ENV_SCRIPT="$SCRIPT_DIR/env.bat"
LAUNCH_CONTROLLER_SCRIPT="$SCRIPT_DIR/launch_controller_robile.bat"

if [ -f "$ENV_SCRIPT" ]; then
    # shellcheck disable=SC1090
    source "$ENV_SCRIPT"
    echo "Sourced $ENV_SCRIPT"
else
    echo "WARNING: env.bat not found next to this script at $ENV_SCRIPT"
    echo "ros2 commands in this menu will likely fail without it."
fi

CONFIG_FILE="$HOME/ros2_network_config.xml"
BASHRC="$HOME/.bashrc"
MARKER_START="# >>> robile setup >>>"
MARKER_END="# <<< robile setup <<<"

ROBILE_IPS=(0 "192.168.0.101" "192.168.0.102" "192.168.0.103" "192.168.0.104")
ROBILE_USER="studentkelo"
ROBILE_PASS="area5142"

pause() { read -rp "Press Enter to return to the menu..." _; }

# --- Step 1: network config + .bashrc env vars ------------------------------
step1_network_config() {
    echo "== Network configuration =="
    local iface
    if [ -n "${ROBILE_IFACE:-}" ]; then
        iface="$ROBILE_IFACE"
    else
        iface="$(ip route show default 2>/dev/null | awk '/default/ {print $5; exit}')"
    fi

    if [ -z "$iface" ]; then
        read -rp "Couldn't auto-detect an interface. Run 'ip a' in another terminal and type the interface name connected to Robile5G: " iface
    else
        read -rp "Detected interface '$iface'. Press Enter to accept, or type a different one: " override
        [ -n "$override" ] && iface="$override"
    fi

    cat > "$CONFIG_FILE" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomTcpTransport</transport_id>
            <type>TCPv4</type>
            <interfaceWhiteList>
                <address>${iface}</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="CustomTcpTransportParticipant">
        <rtps>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>CustomTcpTransport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
EOF
    echo "Wrote $CONFIG_FILE using interface '$iface'"

    if grep -qF "$MARKER_START" "$BASHRC" 2>/dev/null; then
        sed -i "/$MARKER_START/,/$MARKER_END/d" "$BASHRC"
    fi
    cat >> "$BASHRC" <<EOF
$MARKER_START
export FASTRTPS_DEFAULT_PROFILES_FILE=$CONFIG_FILE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
alias robile1='ssh -x ${ROBILE_USER}@${ROBILE_IPS[1]}'
alias robile2='ssh -x ${ROBILE_USER}@${ROBILE_IPS[2]}'
alias robile3='ssh -x ${ROBILE_USER}@${ROBILE_IPS[3]}'
alias robile4='ssh -x ${ROBILE_USER}@${ROBILE_IPS[4]}'
$MARKER_END
EOF
    echo "Updated $BASHRC with env vars + SSH aliases."
    echo "Run 'source ~/.bashrc' (or restart your terminal) to pick these up."
    pause
}

# --- Step 2: SSH into a robot -----------------------------------------------
step2_ssh() {
    echo "== SSH into a Robile =="
    read -rp "Which robot number (1-4)? " num
    if [[ ! "$num" =~ ^[1-4]$ ]]; then
        echo "Invalid choice."; pause; return
    fi
    local ip="${ROBILE_IPS[$num]}"
    echo "Connecting to Robile$num at $ip (password: $ROBILE_PASS)..."

    echo "Checking reachability first (ping)..."
    if ! ping -c 2 -W 2 "$ip" >/dev/null 2>&1; then
        echo ""
        echo "Could not reach $ip. Before trying SSH, check:"
        echo "  - Are you connected to the 'Robile5G' wifi network (not another network)?"
        echo "  - Is Robile$num powered on and finished booting?"
        echo "  - Try manually: ping $ip"
        pause
        return
    fi
    echo "Host is reachable, connecting via SSH..."

    echo "Tip: once connected, run step 3's command manually inside tmux,"
    echo "or just run: tmux new -s bringup   then   ros2 launch robile_bringup robot.launch.py"
    ssh -x "${ROBILE_USER}@${ip}"
    echo "SSH session ended."
    pause
}

# --- Step 3: launch controller/localisation/mapping on the real robot ------
step3_sim() {
    echo "== Launch controller, localisation (MCL) and mapping =="
    if [ ! -f "$LAUNCH_CONTROLLER_SCRIPT" ]; then
        echo "ERROR: launch_controller_robile.bat not found at $LAUNCH_CONTROLLER_SCRIPT"
        pause
        return
    fi
    if [ ! -x "$LAUNCH_CONTROLLER_SCRIPT" ]; then
        chmod +x "$LAUNCH_CONTROLLER_SCRIPT"
    fi
    echo "Running: $LAUNCH_CONTROLLER_SCRIPT"
    echo "This opens gnome-terminal tabs for Controller, MCL, and Mapping (real robot, no Gazebo)."
    "$LAUNCH_CONTROLLER_SCRIPT"
    pause
}

# --- Step 4: set ROS_DOMAIN_ID and verify topics -----------------------------
step4_verify() {
    echo "== Verify communication =="
    read -rp "Robot number to match ROS_DOMAIN_ID (1-4): " num
    if [[ ! "$num" =~ ^[1-4]$ ]]; then
        echo "Invalid choice."; pause; return
    fi
    export ROS_DOMAIN_ID="$num"
    echo "ROS_DOMAIN_ID set to $num for this script's shell."
    echo "NOTE: this only applies inside this script. In your own terminal run:"
    echo "  export ROS_DOMAIN_ID=$num"
    echo ""
    echo "Checking topics..."
    if ! ros2 topic list; then
        echo "Retrying after restarting the ros2 daemon..."
        ros2 daemon stop
        ros2 daemon start
        ros2 topic list
    fi
    pause
}

# --- Step 5: Rviz2 ------------------------------------------------------------
step5_rviz() {
    echo "== Launch Rviz2 =="
    echo "Opening rviz2. Once it opens: Open Config -> robile_gazebo/config/robile.rviz"
    rviz2
    pause
}

# --- Step 6: keyboard teleop (WASD-style driving) ---------------------------
step6_teleop() {
    echo "== Keyboard teleop =="
    echo "This uses the standard ROS2 teleop_twist_keyboard node, which publishes"
    echo "Twist messages to /cmd_vel. Its default key layout is:"
    echo "    u  i  o          i = forward   , = backward"
    echo "    j  k  l          j = turn left  l = turn right"
    echo "    m  ,  .          k = stop"
    echo "(If your project ships its own WASD-mapped teleop node instead, tell me"
    echo "its package/executable name and I'll wire that in here instead.)"
    echo ""
    if ! ros2 pkg list 2>/dev/null | grep -q teleop_twist_keyboard; then
        echo "teleop_twist_keyboard not found. Install it with:"
        echo "  sudo apt install ros-\$ROS_DISTRO-teleop-twist-keyboard"
        pause
        return
    fi
    read -rp "Robot number to match ROS_DOMAIN_ID (1-4, blank to skip): " num
    if [[ "$num" =~ ^[1-4]$ ]]; then
        export ROS_DOMAIN_ID="$num"
        echo "ROS_DOMAIN_ID set to $num for this session."
    fi
    echo "Launching teleop... use i/j/k/l/, to drive, Ctrl+C to stop."
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    pause
}

# --- Menu ---------------------------------------------------------------------
show_menu() {
    clear
    cat <<'MENU'
========================================
      Robile Connection & Control
========================================
 1) Set up network config + .bashrc (run once per machine)
 2) SSH into a robot (Robile1-4)
 3) Launch Controller + MCL + Mapping on real robot (via launch_controller_robile.bat)
 4) Set ROS_DOMAIN_ID + verify topic communication
 5) Launch Rviz2
 6) Launch keyboard teleop (drive the robot)
 0) Exit
========================================
MENU
    read -rp "Press a number: " choice
    case "$choice" in
        1) step1_network_config ;;
        2) step2_ssh ;;
        3) step3_sim ;;
        4) step4_verify ;;
        5) step5_rviz ;;
        6) step6_teleop ;;
        0) echo "Bye."; exit 0 ;;
        *) echo "Invalid option."; pause ;;
    esac
}

while true; do
    show_menu
done
