FROM ubuntu:20.04

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Beijing

# Arguments for build
ARG ROS_VERSION=noetic

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    libgtk2.0-dev \
    lsb-release \
    net-tools \
    cmake \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    git \
    wget \
    curl \
    htop \
    xterm \
    libpcap-dev \
    binutils-dev \
    libdw-dev \
    libdwarf-dev \
    gdb && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS
RUN sh -c 'echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y \
    ros-${ROS_VERSION}-ros-base \
    ros-${ROS_VERSION}-nav-msgs \
    ros-${ROS_VERSION}-sensor-msgs \
    ros-${ROS_VERSION}-cv-bridge \
    ros-${ROS_VERSION}-rviz \
    ros-${ROS_VERSION}-pcl-ros \
    ros-${ROS_VERSION}-image-transport-plugins \
    ros-${ROS_VERSION}-rqt-gui \
    ros-${ROS_VERSION}-rqt-common-plugins \
    ros-${ROS_VERSION}-catkin \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosdep \
    python3-catkin-tools && \
    apt-get clean && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/${ROS_VERSION}/setup.bash" >> ~/.bashrc && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install apt dependencies
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_VERSION}-tf-conversions \
    gazebo11 \
    ros-${ROS_VERSION}-gazebo-ros-pkgs \
    ros-${ROS_VERSION}-gazebo-ros-control \
    ros-${ROS_VERSION}-turtlebot3-description \
    ros-${ROS_VERSION}-teleop-twist-keyboard \
    ros-${ROS_VERSION}-velodyne-simulator && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install pip dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-pip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install evo packaging

############################################
# Set up noVNC
############################################
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    dbus-x11 sudo bash net-tools novnc x11vnc xvfb supervisor xfce4 \
    gnome-shell ubuntu-gnome-desktop gnome-session gdm3 tasksel ssh \
    terminator git nano curl wget zip unzip falkon mesa-utils && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy and extract novnc.zip in one step
COPY novnc.zip /novnc.zip
RUN unzip -o /novnc.zip -d /usr/share && rm /novnc.zip

# Copy the rest of the system files
COPY . /system

# Set proper permissions in one step
RUN chmod +x /system/conf.d/websockify.sh /system/supervisor.sh

# Set the default command
CMD ["/system/supervisor.sh"]
