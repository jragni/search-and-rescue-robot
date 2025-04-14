FROM ghcr.io/sloretz/ros:humble-desktop

# Add a non-root user
ARG USER_UID=1000
ARG USERNAME=ubuntu
ARG USER_GID=$USER_UID

RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
#Add sudo support for the non-root user

RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Add serial permissions
RUN sudo usermod --append --groups dialout $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

RUN sudo apt install -y vim
RUN sudo apt install -y udev

RUN sudo apt install -y python3-pip && \
    pip3 install pyserial


# install lsusb
RUN sudo apt-get update \
    && sudo apt-get install -y usbutils \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# install imu tools
RUN sudo apt-get update \
    && sudo apt-get install -y ros-humble-imu-tools \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# install orbbecSDK related dependencies (RGB-D Camera) and ros navigation package
RUN sudo apt update && sudo apt upgrade -y && \ 
sudo apt install \
libgflags-dev \
nlohmann-json3-dev \
libyaml-cpp-dev \
ros-$ROS_DISTRO-rmw-zenoh-cpp \
ros-$ROS_DISTRO-image-transport \
ros-${ROS_DISTRO}-image-transport-plugins \
ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-publisher \
ros-$ROS_DISTRO-camera-info-manager \
ros-$ROS_DISTRO-point-cloud-transport \
ros-$ROS_DISTRO-diagnostic-updater \
ros-$ROS_DISTRO-diagnostic-msgs \
ros-$ROS_DISTRO-statistics-msgs \
ros-$ROS_DISTRO-backward-ros libdw-dev \
ros-$ROS_DISTRO-navigation2 \
ros-$ROS_DISTRO-robot-localization \
ros-$ROS_DISTRO-nav2-bringup \
ros-$ROS_DISTRO-twist-mux \
ros-$ROS_DISTRO-xacro \
ros-$ROS_DISTRO-turtlebot3* \
ros-$ROS_DISTRO-rtabmap-ros -y

# Rosdep update
RUN rosdep update

# Add dependency for orbbec camera ros2
WORKDIR /library
RUN git clone https://github.com/libuvc/libuvc.git
WORKDIR /library/libuvc
RUN mkdir build && cd build && cmake .. && make -j4 && sudo make install && sudo ldconfig

WORKDIR /

# TODO add bashrc to source that ws
WORKDIR /library_ws/src
RUN git clone https://github.com/Slamtec/sllidar_ros2.git
#RUN git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
RUN git clone https://github.com/orbbec/ros2_astra_camera.git

# copy over rules
#RUN sudo cp OrbbecSDK_ROS2/orbbec_camera/scripts/99-obsensor-libusb.rules \
#/etc/udev/rules.d/
RUN sudo cp ros2_astra_camera/astra_camera/scripts/56-orbbec-usb.rules \
/etc/udev/rules.d/

RUN sudo cp sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/

WORKDIR ..

# build library
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release"

WORKDIR /

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /library_ws/install/setup.bash" >> ~/.bashrc

RUN echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
# TODO add rmw zenoh setup
RUN echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc

WORKDIR /home/ubuntu

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# TODO add build for colcon_ws and source it
