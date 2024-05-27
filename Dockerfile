FROM nvcr.io/nvidia/l4t-jetpack:r36.2.0

RUN apt update
RUN apt install -y sudo lsb-release

ARG USERNAME=user
ARG GROUPNAME=user
ARG UID=1000
ARG GID=1000
ARG PASSWORD=user
RUN groupadd -g $GID $GROUPNAME && \
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USERNAME && \
    echo $USERNAME:$PASSWORD | chpasswd && \
    echo "$USERNAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN sudo usermod -aG sudo,video $USERNAME
USER $USERNAME
WORKDIR /home/$USERNAME/

# install ros2
RUN sudo apt update && sudo apt install -y locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN sudo apt install -y software-properties-common
RUN sudo add-apt-repository universe
RUN sudo apt update && sudo apt install curl -y
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN sudo apt update

ENV DEBIAN_FRONTEND=noninteractive
RUN sudo ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN sudo apt install -y ros-humble-ros-base
RUN sudo apt install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip git python3-pip
RUN sudo apt install -y ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-pcl-ros
RUN sudo apt install -y vim ros-humble-slam-toolbox \
    ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-nav2-bringup
RUN sudo apt install -y python3-open3d ros-humble-tf-transformations ros-humble-robot-localization
RUN pip install ros2-numpy transforms3d

RUN sudo apt install -y python3-rosdep
RUN sudo apt install -y ros-dev-tools
RUN sudo apt install -y v4l-utils

# Install livox SDK
WORKDIR /home/$USERNAME
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git
WORKDIR /home/$USERNAME/Livox-SDK2
RUN mkdir build
WORKDIR /home/$USERNAME/Livox-SDK2/build
RUN cmake .. && make -j2 && sudo make install


WORKDIR /home/$USERNAME/

# Install livox_ros_driver2
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
RUN /bin/bash -c 'source /opt/ros/humble/setup.sh && /home/user/ws_livox/src/livox_ros_driver2/build.sh humble'

# Install micro-ROS
WORKDIR /home/$USERNAME
RUN mkdir uros_ws
WORKDIR /home/$USERNAME/uros_ws
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
RUN /bin/bash -c 'source /opt/ros/humble/setup.sh && sudo rosdep init && rosdep update'
RUN /bin/bash -c 'source /opt/ros/humble/setup.sh && rosdep install --from-paths src --ignore-src -y'
RUN /bin/bash -c 'source /opt/ros/humble/setup.sh && colcon build'
RUN /bin/bash -c 'source ~/uros_ws/install/setup.sh && ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh'

# realsense
WORKDIR /home/$USERNAME
RUN wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.55.1.tar.gz
RUN tar -zxvf v2.55.1.tar.gz 
RUN rm v2.55.1.tar.gz 
RUN cd librealsense-2.55.1/scripts && sudo ./libuvc_installation.sh -DBUILD_WITH_CUDA=true

RUN sudo apt install -y ros-humble-rmw-cyclonedds-cpp

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/user/ws_livox/install/setup.bash" >> ~/.bashrc
RUN echo "source /home/user/uros_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /home/user/workspace/install/setup.bash" >> ~/.bashrc
RUN echo "sudo chmod 666 /dev/ttyUSB0" >> ~/.bashrc
RUN echo "source /home/user/workspace/install/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc


WORKDIR /home/$USERNAME/workspace

