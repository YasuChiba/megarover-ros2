#!/bin/bash

# Define the container name
CONTAINER_NAME="megarover-start-waypoint"

echo "Starting waypoint follower..."

# Step 1: Run Docker container in detached mode and save the container ID
docker run -it --runtime=nvidia --network=host --ipc=host --pid=host --privileged \
        -v $(pwd)/workspace:/home/user/workspace \
        --name $CONTAINER_NAME \
        megarover-ros2-ros \
        /home/user/workspace/container_entrypoints/start_waypoint_follower.sh
        
docker stop $CONTAINER_NAME

# Optional: Wait a bit if you need to ensure that logs or other operations complete
sleep 3

# Step 6: Remove the Docker container
docker rm $CONTAINER_NAME
echo "Container $CONTAINER_NAME removed."
