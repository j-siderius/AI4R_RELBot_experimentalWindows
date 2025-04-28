#!/bin/bash

CONTAINER_NAME=relbot_demo
IMAGE_NAME=ros2-jazzy-ubuntu24
HOST_FOLDER=/home/bavantha/Desktop/RelBot/demo_ws/src
CONTAINER_FOLDER=/ros2_ws/src

# Build the image if not present
if ! docker image inspect $IMAGE_NAME > /dev/null 2>&1; then
    echo "Docker image $IMAGE_NAME not found. Building..."
    docker build -t $IMAGE_NAME .
fi

# Set ROS networking variables if not set already. Adjust ROS_IP with your laptop's IP address.
#: ${ROS_IP:="192.168.2.7"}    # Replace with your laptop's actual IP if different.
#: ${ROS_IP:="10.42.0.1"}    # Using the Ethernet (LAN) interface IP
: ${ROS_DOMAIN_ID:=8}

echo "Using ROS_IP: $ROS_IP, ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# Allow X11 GUI if needed (for visual debugging with rqt_image_view or cv2.imshow)
xhost +local:docker

# No need to map /dev/video devices now, since the robot provides the image stream.
VIDEO_DEVICES=""

# Check if the container is already running
if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Container is already running. Attaching to a new shell..."
    docker exec -it $CONTAINER_NAME bash
elif [ "$(docker ps -aq -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "Starting existing container..."
    docker start $CONTAINER_NAME
    docker exec -it $CONTAINER_NAME bash
else
    echo "Creating and starting new container..."
    docker run -it \
        --name $CONTAINER_NAME \
        --net=host \
        -e DISPLAY=$DISPLAY \
        -e ROS_IP=$ROS_IP \
        -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOST_FOLDER:$CONTAINER_FOLDER \
        --env="QT_X11_NO_MITSHM=1" \
        --privileged \
        $IMAGE_NAME \
        bash
fi
