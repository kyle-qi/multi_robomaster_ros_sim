#!/bin/bash
# Run the multi_robomaster_ros_sim simulator inside Docker using Colima.
#
# USAGE:
#   ./run.sh          — headless mode (no plot, just ROS topics)
#   ./run.sh --plot   — with matplotlib plot via X11 (requires XQuartz on macOS)
#
# PREREQUISITES (macOS with Colima):
#   1. colima start --network-address --network-mode=bridged
#   2. (If using --plot) XQuartz installed and running, with
#      Preferences → Security → "Allow connections from network clients" checked.
#      Then: xhost +$(colima ssh -- hostname -I | awk '{print $1}') 
#
# Without --plot the simulator still publishes all topics and serves the
# gripper action — you just won't see the matplotlib window.

set -e

PLOT_MODE=false
if [[ "$1" == "--plot" ]]; then
    PLOT_MODE=true
fi

# Build the ROS command
ROS_CMD="cd /linked_folder/ros_ws_sim && colcon build && source install/setup.bash && ros2 run multi_robomaster_ros_sim simulator"

if [[ "$(uname)" == "Darwin" ]]; then
    if [[ "$PLOT_MODE" == "true" ]]; then
        # macOS with XQuartz: get host IP visible from Colima VM
        HOST_IP=$(ipconfig getifaddr en0 2>/dev/null || echo "127.0.0.1")
        xhost + "$HOST_IP" 2>/dev/null || true
        DISPLAY_ENV="$HOST_IP:0"
        echo "Using DISPLAY=$DISPLAY_ENV (ensure XQuartz is running)"
    else
        DISPLAY_ENV=""
        echo "Running in headless mode (no plot). Use --plot for GUI."
    fi

    DOCKER_ARGS="-it --rm --pid=host --ipc=host"
    DOCKER_ARGS="$DOCKER_ARGS --volume ./linked_folder:/linked_folder:rw"

    if [[ -n "$DISPLAY_ENV" ]]; then
        DOCKER_ARGS="$DOCKER_ARGS --env DISPLAY=$DISPLAY_ENV"
        DOCKER_ARGS="$DOCKER_ARGS --volume $HOME/.Xauthority:/root/.Xauthority:rw"
    else
        # In headless mode, override matplotlib backend
        ROS_CMD="export MPLBACKEND=Agg && $ROS_CMD"
    fi

    docker run $DOCKER_ARGS \
        --name="dji_robomaster_ros_simulator" dji_robomaster_ros:1.0 \
        /bin/bash -c "$ROS_CMD"
else
    # Linux: host network + native display
    xhost +
    docker run -it --rm \
        --network=host --pid=host --ipc=host \
        --volume ./linked_folder:/linked_folder:rw \
        --volume "$HOME/.Xauthority:/root/.Xauthority:rw" \
        --env="DISPLAY" \
        --name="dji_robomaster_ros_simulator" dji_robomaster_ros:1.0 \
        /bin/bash -c "$ROS_CMD"
fi
