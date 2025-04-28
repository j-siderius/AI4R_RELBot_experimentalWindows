FROM ubuntu:24.04

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    locales \
    curl gnupg2 lsb-release git \
    python3-pip \
    build-essential \
    software-properties-common

# Locale setup
RUN locale-gen en_US en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# ROS 2 apt repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y ros-jazzy-desktop

# Install colcon via pip
RUN pip3 install --break-system-packages -U colcon-common-extensions
RUN pip3 install --break-system-packages ultralytics opencv-python numpy

# Source ROS setup in every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "#export ROS_DOMAIN_ID=<RELBot_ID> #Please edit the RELBot ID each time you use a new robot"

CMD ["/bin/bash"]

