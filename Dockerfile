FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip git python3-pip
RUN apt-get install -y ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-pcl-ros


RUN apt install -y sudo

ARG USERNAME=user
ARG GROUPNAME=user
ARG UID=1000
ARG GID=1000
ARG PASSWORD=user
RUN groupadd -g $GID $GROUPNAME && \
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME && \
    echo $USERNAME:$PASSWORD | chpasswd && \
    echo "$USERNAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME
WORKDIR /home/$USERNAME/


# Install livox SDK
WORKDIR /home/$USERNAME
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /home/$USERNAME/Livox-SDK2
RUN mkdir build
WORKDIR /home/$USERNAME/Livox-SDK2/build
RUN cmake .. && make -j2 && sudo make install


RUN /bin/bash -c '. /opt/ros/humble/setup.bash;'


RUN sudo apt install -y vim
WORKDIR /home/$USERNAME/

