#!/bin/bash
set -e
echo "Starting ROS 2 Humble Installation on Raspberry Pi..."
echo "skypal1234" | sudo -S dpkg --configure -a
echo "skypal1234" | sudo -S apt update && echo "skypal1234" | sudo -S apt install -y locales curl software-properties-common
echo "skypal1234" | sudo -S locale-gen en_US en_US.UTF-8
echo "skypal1234" | sudo -S update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "skypal1234" | sudo -S add-apt-repository universe -y
echo "skypal1234" | sudo -S curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "skypal1234" | sudo -S apt update
echo "skypal1234" | sudo -S apt install -y ros-humble-ros-base python3-argcomplete python3-colcon-common-extensions
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
echo "ROS 2 Humble installation complete!"
