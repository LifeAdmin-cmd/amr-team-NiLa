#!/usr/bin/env bash
###############################################################################
# Robile Connection, Bringup & Autonomous Node Launch Script
#
# Automated single-command execution:
#   1. Automatically configures FastRTPS XML network whitelist for active Wi-Fi.
#   2. Sets and exports ROS_DOMAIN_ID (default: 4, or argument: 1-4).
#   3. Verifies DDS communication / checks robot network reachability.
#   4. Documents & automates starting the control task (bringup) on the robot.
#   5. Automatically launches Controller, MCL, and Mapping nodes.
#
# Usage:
#   ./robile_connection.bat [ROBOT_ID]           (e.g., ./robile_connection.bat 4)
#   ./robile_connection.bat -r 3                 (specify robot 3)
#   ./robile_connection.bat --remote-bringup     (auto-start robot drivers via SSH)
#   ./robile_connection.bat --ssh                (open direct SSH shell to robot)
#   ./robile_connection.bat --rviz               (launch RViz2 alongside nodes)
#   ./robile_connection.bat --teleop             (launch keyboard teleop)
#   ./robile_connection.bat --no-launch          (configure network and domain only)
#   ./robile_connection.bat --menu               (legacy interactive menu)
#   ./robile_connection.bat --help               (display help)
###############################################################################
set -o pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ENV_SCRIPT="$SCRIPT_DIR/env.bat"
LAUNCH_CONTROLLER_SCRIPT="$SCRIPT_DIR/launch_controller_robile.bat"

# Source the existing ROS2 workspace environment
if [ -f "$ENV_SCRIPT" ]; then
    # shellcheck disable=SC1090
    source "$ENV_SCRIPT"
else
    echo "WARNING: env.bat not found next to this script at $ENV_SCRIPT"
fi

CONFIG_FILE="$HOME/ros2_network_config.xml"
BASHRC="$HOME/.bashrc"
MARKER_START="# >>> robile setup >>>"
MARKER_END="# <<< robile setup <<<"

ROBILE_IPS=(0 "192.168.0.101" "192.168.0.102" "192.168.0.103" "192.168.0.104")
ROBILE_USER="studentkelo"
ROBILE_PASS="area5142"

# Default settings
DEFAULT_ROBOT_ID="4"
ROBOT_ID=""
CUSTOM_IFACE=""
ACTION_MODE="auto" # auto, remote_bringup, ssh, rviz, teleop, menu, no_launch
LAUNCH_RVIZ_FLAG=false

# ─────────────────────────────────────────────────────────────────────────────
# Help & Documentation
# ─────────────────────────────────────────────────────────────────────────────
show_help() {
    cat <<EOF
Usage: ./robile_connection.bat [ROBOT_ID] [OPTIONS]

Automated connection and deployment script for physical Robile robots.
Automatically runs network config, sets ROS_DOMAIN_ID, checks connectivity,
and launches the controller, MCL localisation, and mapping nodes.

Arguments:
  ROBOT_ID                   Robot number (1-4). Defaults to 4 (or \$ROS_DOMAIN_ID).

Options:
  -r, --robot <1-4>          Specify the target Robile ID.
  -i, --iface <interface>    Override the detected network interface (e.g. wlp1s0).
  --remote-bringup           Start the robot hardware control task via SSH (tmux).
  --ssh                      Open an interactive SSH shell to the robot.
  --rviz                     Launch RViz2 configured for Robile visualization.
  --teleop                   Launch keyboard teleoperation node (cmd_vel).
  --no-launch                Configure network and domain only, don't spawn nodes.
  --menu                     Open the legacy interactive menu.
  -h, --help                 Show this help message.

Examples:
  ./robile_connection.bat                # Connect to default Robile (4) and launch all nodes
  ./robile_connection.bat 3              # Connect to Robile 3 and launch all nodes
  ./robile_connection.bat 4 --rviz       # Connect to Robile 4, launch nodes and RViz2
  ./robile_connection.bat 4 --remote-bringup # Start robot onboard drivers via SSH and launch
  ./robile_connection.bat --ssh          # SSH into the robot terminal
EOF
}

# ─────────────────────────────────────────────────────────────────────────────
# Step 1: Automated Network Configuration
# ─────────────────────────────────────────────────────────────────────────────
setup_network_config() {
    echo "== [1/4] Network Configuration =="
    local iface="$CUSTOM_IFACE"

    if [ -z "$iface" ]; then
        if [ -n "${ROBILE_IFACE:-}" ]; then
            iface="$ROBILE_IFACE"
        else
            # Try default route interface first
            iface="$(ip route show default 2>/dev/null | awk '/default/ {print $5; exit}')"
            # Fall back to first active wireless interface
            if [ -z "$iface" ]; then
                iface="$(ip -br link 2>/dev/null | awk '$1 ~ /^wl/ && $2 == "UP" {print $1; exit}')"
            fi
            # Fall back to any UP non-virtual interface
            if [ -z "$iface" ]; then
                iface="$(ip -br link 2>/dev/null | awk '$2 == "UP" && $1 !~ /^(lo|docker|br-)/ {print $1; exit}')"
            fi
        fi
    fi

    if [ -z "$iface" ]; then
        echo "⚠️  Could not auto-detect a network interface. Defaulting to 'wlp1s0'."
        iface="wlp1s0"
    else
        echo "✓ Detected network interface: '$iface'"
    fi

    # Check if config file needs writing/updating
    local need_write=true
    if [ -f "$CONFIG_FILE" ]; then
        if grep -q "<address>${iface}</address>" "$CONFIG_FILE" 2>/dev/null; then
            need_write=false
            echo "✓ FastRTPS configuration file up-to-date at $CONFIG_FILE"
        fi
    fi

    if [ "$need_write" = true ]; then
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
        echo "✓ Wrote FastRTPS profile to $CONFIG_FILE using interface '$iface'"
    fi

    # Ensure environment variables are active in this shell
    export FASTRTPS_DEFAULT_PROFILES_FILE="$CONFIG_FILE"
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

    # Update ~/.bashrc persistence if missing
    if ! grep -qF "$MARKER_START" "$BASHRC" 2>/dev/null; then
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
        echo "✓ Added FastRTPS exports and SSH aliases to $BASHRC"
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Step 2: Set ROS_DOMAIN_ID
# ─────────────────────────────────────────────────────────────────────────────
setup_ros_domain() {
    echo "== [2/4] ROS_DOMAIN_ID Configuration =="
    if [ -z "$ROBOT_ID" ]; then
        if [ -n "${ROS_DOMAIN_ID:-}" ] && [[ "$ROS_DOMAIN_ID" =~ ^[1-4]$ ]]; then
            ROBOT_ID="$ROS_DOMAIN_ID"
        elif [ -n "${ROBOT_NUM:-}" ] && [[ "$ROBOT_NUM" =~ ^[1-4]$ ]]; then
            ROBOT_ID="$ROBOT_NUM"
        else
            ROBOT_ID="$DEFAULT_ROBOT_ID"
            echo "ℹ️  No robot number specified. Defaulting to Robile $ROBOT_ID (ROS_DOMAIN_ID=$ROBOT_ID)."
            echo "   (To use another robot, pass it as argument: ./robile_connection.bat 3)"
        fi
    fi

    export ROS_DOMAIN_ID="$ROBOT_ID"
    export ROBOT_NAME="robile${ROBOT_ID}"
    echo "✓ Target Robot: Robile$ROBOT_ID (IP: ${ROBILE_IPS[$ROBOT_ID]})"
    echo "✓ Exported ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
}

# ─────────────────────────────────────────────────────────────────────────────
# Step 3: Robot Connectivity & DDS Topic Verification
# ─────────────────────────────────────────────────────────────────────────────
verify_connectivity_and_topics() {
    echo "== [3/4] Checking Robot Connectivity & Topics =="
    local ip="${ROBILE_IPS[$ROBOT_ID]}"

    # Ping check
    if ping -c 1 -W 2 "$ip" >/dev/null 2>&1; then
        echo "✓ Robot at $ip is reachable via ping."
    else
        echo "⚠️  Could not ping robot at $ip."
        echo "   • Are you connected to Wi-Fi 'Robile5G'?"
        echo "   • Is Robile$ROBOT_ID powered on and finished booting?"
        echo "   • You can test manually: ping $ip"
    fi

    # ROS 2 Daemon & topic check
    echo "Checking ROS 2 topics on domain $ROS_DOMAIN_ID..."
    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"

    if [ -z "$topics" ]; then
        echo "Restarting ROS 2 daemon..."
        ros2 daemon stop >/dev/null 2>&1 || true
        ros2 daemon start >/dev/null 2>&1 || true
        topics="$(ros2 topic list 2>/dev/null || true)"
    fi

    if echo "$topics" | grep -qE "(/scan|/odom|/cmd_vel)"; then
        echo "✓ Found active robot topics:"
        echo "$topics" | grep -E "(/scan|/odom|/cmd_vel|/tf)" | sed 's/^/    /'
    else
        echo "ℹ️  No robot hardware topics (/scan, /odom, /cmd_vel) detected yet."
        echo "   Ensure the control task / bringup is running on the physical robot."
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Documentation & Automation: Control Task on the Robot
# ─────────────────────────────────────────────────────────────────────────────
display_robot_control_task_doc() {
    local ip="${ROBILE_IPS[$ROBOT_ID]}"
    cat <<EOF
───────────────────────────────────────────────────────────────────────────────
🤖 CONTROL TASK ON THE PHYSICAL ROBOT (robile_bringup)
   The physical robot must run its onboard hardware driver stack to publish
   /scan, /odom, and execute /cmd_vel commands via Kelo Tulip wheel drives.

   HOW TO START ON THE ROBOT:
   1. Connect via SSH:
        ssh -x ${ROBILE_USER}@${ip}        (Password: ${ROBILE_PASS})
        or alias: robile${ROBOT_ID}
   2. Start tmux session (keeps drivers alive if Wi-Fi drops):
        tmux new -s bringup
   3. Launch the control task / drivers:
        ros2 launch robile_bringup robot.launch.py
   4. Detach from tmux:
        Press [Ctrl+b], then press [d]

   To re-attach later:  tmux attach -t bringup
   To terminate task:   tmux kill-session -t bringup
───────────────────────────────────────────────────────────────────────────────
EOF
}

remote_bringup_task() {
    local ip="${ROBILE_IPS[$ROBOT_ID]}"
    echo "== Starting Control Task on Robile$ROBOT_ID ($ip) via SSH =="

    if ! ping -c 1 -W 2 "$ip" >/dev/null 2>&1; then
        echo "❌ Cannot reach $ip. Make sure you are on Robile5G Wi-Fi."
        return 1
    fi

    echo "Connecting to $ip to launch bringup inside tmux session 'bringup'..."
    echo "(If prompted for password, enter: $ROBILE_PASS)"

    ssh -t "${ROBILE_USER}@${ip}" "
        if tmux has-session -t bringup 2>/dev/null; then
            echo '✓ Bringup session already running on robot.'
            tmux list-sessions
        else
            echo 'Starting ros2 launch robile_bringup robot.launch.py in tmux...'
            tmux new-session -d -s bringup 'source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=${ROBOT_ID} && export ROBOT_NAME=robile${ROBOT_ID} && ros2 launch robile_bringup robot.launch.py'
            sleep 2
            tmux list-sessions
            echo '✓ Control task launched in background tmux session [bringup].'
        fi
    "
}

# ─────────────────────────────────────────────────────────────────────────────
# Step 4: Launch Local Nodes (Controller, MCL Localisation, Mapping)
# ─────────────────────────────────────────────────────────────────────────────
launch_local_nodes() {
    echo "== [4/4] Launching Controller, Localisation & Mapping Nodes =="

    if [ ! -f "$LAUNCH_CONTROLLER_SCRIPT" ]; then
        echo "ERROR: launch_controller_robile.bat not found at $LAUNCH_CONTROLLER_SCRIPT"
        return 1
    fi

    if [ ! -x "$LAUNCH_CONTROLLER_SCRIPT" ]; then
        chmod +x "$LAUNCH_CONTROLLER_SCRIPT"
    fi

    echo "Opening terminal tabs for Controller, MCL, and Mapping..."
    "$LAUNCH_CONTROLLER_SCRIPT"
    echo "✓ Node tabs launched successfully."

    if [ "$LAUNCH_RVIZ_FLAG" = true ]; then
        echo "Launching RViz2..."
        rviz2 &
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Standalone Utilities: SSH, RViz, Teleop
# ─────────────────────────────────────────────────────────────────────────────
run_ssh() {
    local ip="${ROBILE_IPS[$ROBOT_ID]}"
    echo "Connecting to Robile$ROBOT_ID at $ip (password: $ROBILE_PASS)..."
    ssh -x "${ROBILE_USER}@${ip}"
}

run_rviz() {
    echo "Launching RViz2 with ROS_DOMAIN_ID=$ROS_DOMAIN_ID..."
    echo "Tip: Open Config -> robile_gazebo/config/robile.rviz"
    rviz2
}

run_teleop() {
    echo "Launching keyboard teleop on ROS_DOMAIN_ID=$ROS_DOMAIN_ID..."
    if ! ros2 pkg list 2>/dev/null | grep -q teleop_twist_keyboard; then
        echo "teleop_twist_keyboard not found. Install it with:"
        echo "  sudo apt install ros-\$ROS_DISTRO-teleop-twist-keyboard"
        return 1
    fi
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
}

# ─────────────────────────────────────────────────────────────────────────────
# Legacy Interactive Menu (for backward compatibility via --menu)
# ─────────────────────────────────────────────────────────────────────────────
pause() { read -rp "Press Enter to continue..." _; }

show_menu() {
    while true; do
        clear
        cat <<'MENU'
========================================
      Robile Connection & Control
========================================
 1) Automatically configure network & ROS_DOMAIN_ID
 2) SSH into a robot (Robile 1-4)
 3) Launch Controller + MCL + Mapping (launch_controller_robile.bat)
 4) Check topics & restart ROS 2 daemon
 5) Launch RViz2
 6) Launch keyboard teleop
 7) Remote bringup (start control task on robot via SSH)
 0) Exit
========================================
MENU
        read -rp "Press a number: " choice
        case "$choice" in
            1) setup_network_config; setup_ros_domain; verify_connectivity_and_topics; pause ;;
            2)
                read -rp "Robot number (1-4): " r_id
                if [[ "$r_id" =~ ^[1-4]$ ]]; then
                    ROBOT_ID="$r_id"
                    run_ssh
                fi
                pause
                ;;
            3) launch_local_nodes; pause ;;
            4) verify_connectivity_and_topics; pause ;;
            5) run_rviz; pause ;;
            6) run_teleop; pause ;;
            7) remote_bringup_task; pause ;;
            0) echo "Bye."; exit 0 ;;
            *) echo "Invalid option."; pause ;;
        esac
    done
}

# ─────────────────────────────────────────────────────────────────────────────
# Parse Command-Line Arguments
# ─────────────────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        [1-4])
            ROBOT_ID="$1"
            shift
            ;;
        -r|--robot)
            ROBOT_ID="$2"
            shift 2
            ;;
        -i|--iface)
            CUSTOM_IFACE="$2"
            shift 2
            ;;
        --remote-bringup|--start-robot)
            ACTION_MODE="remote_bringup"
            shift
            ;;
        --ssh)
            ACTION_MODE="ssh"
            shift
            ;;
        --rviz)
            LAUNCH_RVIZ_FLAG=true
            shift
            ;;
        --rviz-only)
            ACTION_MODE="rviz"
            shift
            ;;
        --teleop)
            ACTION_MODE="teleop"
            shift
            ;;
        --no-launch)
            ACTION_MODE="no_launch"
            shift
            ;;
        --menu)
            ACTION_MODE="menu"
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Run './robile_connection.bat --help' for usage."
            exit 1
            ;;
    esac
done

# Validate ROBOT_ID if provided
if [ -n "$ROBOT_ID" ] && [[ ! "$ROBOT_ID" =~ ^[1-4]$ ]]; then
    echo "ERROR: Invalid robot ID '$ROBOT_ID'. Must be 1, 2, 3, or 4."
    exit 1
fi

# ─────────────────────────────────────────────────────────────────────────────
# Main Execution Dispatcher
# ─────────────────────────────────────────────────────────────────────────────
case "$ACTION_MODE" in
    menu)
        setup_ros_domain
        show_menu
        ;;
    ssh)
        setup_ros_domain
        run_ssh
        ;;
    remote_bringup)
        setup_network_config
        setup_ros_domain
        remote_bringup_task
        ;;
    rviz)
        setup_network_config
        setup_ros_domain
        run_rviz
        ;;
    teleop)
        setup_network_config
        setup_ros_domain
        run_teleop
        ;;
    no_launch)
        setup_network_config
        setup_ros_domain
        verify_connectivity_and_topics
        display_robot_control_task_doc
        echo "✓ Network setup and domain verification complete (nodes not launched)."
        ;;
    auto)
        echo "================================================================="
        echo "       Robile Deployment & Automated Connection Pipeline"
        echo "================================================================="
        setup_network_config
        echo ""
        setup_ros_domain
        echo ""
        verify_connectivity_and_topics
        echo ""
        display_robot_control_task_doc
        echo ""
        launch_local_nodes
        echo "================================================================="
        echo "✅ Pipeline complete! Local nodes are running in terminal tabs."
        echo "================================================================="
        ;;
esac
