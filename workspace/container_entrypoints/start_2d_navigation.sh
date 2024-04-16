#!/bin/bash

# Function to handle SIGINT and SIGTERM signals
cleanup() {
    # Send SIGINT to the current process group
    kill -SIGINT 0
    exit 0
}

# Set up a trap to call cleanup when SIGINT or SIGTERM is caught
trap cleanup SIGINT SIGTERM

# Launch the ROS 2 nodes in the background
source /opt/ros/humble/setup.bash && \
    source /home/user/ws_livox/install/setup.bash && \
    source /home/user/uros_ws/install/setup.bash && \
    source /home/user/workspace/install/setup.bash && \
    if [ -e /dev/ttyUSB0 ]; then sudo chmod 666 /dev/ttyUSB0; fi && \
    ros2 launch c_megarover 2d_localization_launch.py simulator:=false rviz:=true map_file_path:=/home/user/workspace/maps/map.pcd map_2d_file_path:=/home/user/workspace/maps/map.yaml &
    
PID=$!

# Wait for user to press 'x' to exit
while true; do
    # Disable echo and set read to only need a single character input
    stty -echo -icanon time 0 min 0
    read -n 1 key
    if [[ "$key" == "x" ]]; then
        cleanup
    fi
    # Provide a minimal delay to prevent high CPU usage
    sleep 0.1
done

cleanup

# Wait for the ROS 2 launch process to exit
wait $PID
