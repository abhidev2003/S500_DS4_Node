#!/bin/bash
export DEBIAN_FRONTEND=noninteractive

echo "Updating apt..."
apt-get update

echo "Installing locales and software-properties-common..."
apt-get install -y curl software-properties-common locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "Adding Universe repo..."
add-apt-repository universe -y

echo "Adding ROS 2 GPG key..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "Adding ROS 2 sources list..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Installing ROS 2 Humble..."
apt-get update
apt-get install -y ros-humble-ros-base python3-argcomplete python3-colcon-common-extensions

echo "Sourcing setup.bash..."
if ! grep -q "source /opt/ros/humble/setup.bash" /home/skypal/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> /home/skypal/.bashrc
fi

echo "SUCCESS_ROS2_INSTALL"
