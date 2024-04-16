#!/bin/bash

# Define cleanup function
cleanup() {
    echo "Stopping Docker containers..."
    docker compose stop
    echo "Docker containers have been stopped."
}

# Setup trap to catch SIGINT (Ctrl-C) and SIGTERM
trap 'cleanup' SIGINT SIGTERM


echo "Launching Docker container..."
docker compose start  # -d runs it in detached mode

echo "Waiting for the container to initialize..."
sleep 3  # Adjust time as necessary for your container to initialize

echo "Launching ROS2 nodes..."
docker exec megarover-ros2-ros-1 /bin/bash -c "source /opt/ros/humble/setup.bash && \
                                                source /home/user/ws_livox/install/setup.bash && \
                                                source /home/user/uros_ws/install/setup.bash && \
                                                source /home/user/workspace/install/setup.bash && \
                                                if [ -e /dev/ttyUSB0 ]; then sudo chmod 666 /dev/ttyUSB0; fi && \
                                                ros2 launch c_megarover navigation_launch.py simulator:=true rviz:=false map_file_path:=/home/user/workspace/maps/sendagi.pcd map_2d_file_path:=/home/user/workspace/maps/sendagi.yaml"

echo "ROS2 nodes have been launched."


cleanup
