#!/bin/bash
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y

# Install Visual Studio Code
sudo apt install -y software-properties-common apt-transport-https wget
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt update && sudo apt install -y code

# Install Git
sudo apt install -y git

# Install ROS2 Foxy Fitzroy Desktop
#sudo apt update && sudo apt install locales
#sudo locale-gen en_US en_US.UTF-8
#sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install -y ros-foxy-desktop
source /opt/ros/foxy/setup.bash

# Install pip and  argcomplete
sudo apt install -y python3-pip
pip3 install -U argcomplete

# Install rosdep
sudo apt install -y python3-rosdep2
rosdep update

# in between
sudo apt update && sudo apt upgrade -y

# Install colcon
sudo apt install -y python3-colcon-common-extensions

# At the end again
sudo apt update && sudo apt upgrade -y
