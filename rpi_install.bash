# Go through these bash commands one by one
# Prerequisite is a fresh Ubuntu 20.04 !! 64bit !! OS
# Only the password was changed after first boot
# Everything else is stock

# First Update
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y

# Install ROS2 Foxy Fitzroy Desktop
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install -y ros-foxy-ros-base
source /opt/ros/foxy/setup.bash
# Check to see if installed correctly
echo "ROS_VERSION:        "$ROS_VERSION


# Install rosdep
sudo apt install -y python3-rosdep2
rosdep update


# Install colcon
sudo apt install -y python3-colcon-common-extensions


# Install pip and  argcomplete
sudo apt install -y python3-pip
pip3 install -U argcomplete


# in between
sudo apt update && sudo apt upgrade -y


# Create workspace
mkdir -p ~/colcon_ws/src
cd colcon_ws
colcon build






# Configure .bashrc:
# The the following in between "#----" to the end of your .bashrc script
#----------------------------------------------------------------------
# ROS 2 SETUP
echo "ROS 2 Setup:"

# Source Underlay
cmd="/opt/ros/foxy/setup.bash"
source $cmd
echo "ROS Underlay:       "$cmd

# Source Overlay
# Change to the path of your workspace's "install/local_setup.bash" file
cmd="$HOME/colcon_ws/install/local_setup.bash"
source $cmd
echo "ROS Overlay:        "$cmd

# You can add other workspaces
#cmd="$HOME/colcon_libs/install/local_setup.bash"
#source $cmd
#echo "ROS Overlay:        "$cmd
echo "******************************************************"

# ROS 2 Settings
# Print ROS Variables
#printenv | grep -i ROS
echo "ROS_VERSION:        "$ROS_VERSION
echo "ROS_PYTHON_VERSION: "$ROS_PYTHON_VERSION
echo "ROS_DISTRO:         "$ROS_DISTRO
export ROS_DOMAIN_ID=69
echo "ROS_DOMAIN_ID:      "$ROS_DOMAIN_ID
echo "To change:          sudo nano /opt/ros/foxy/setup.bash"
echo "******************************************************"
echo "Topics active:"
ros2 topic list
echo "******************************************************"
printf "\n"
echo "If your package is not listed (ros2 pkg list):"
echo " * make sure you are sourcing the correct workspace: .bashrc"
echo " * delete build/, install/ and rebuild w/o errors"
# Parsing of git branch
parse_git_branch() {
     git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'
}
export PS1="\u@\h \[\e[32m\]\w \[\e[91m\]\$(parse_git_branch)\[\e[00m\]$ "
#----------------------------------------------------------------------






# Install network tools
sudo apt install -y wireless-tools


# Install Git
sudo apt install -y git

# Setup remote git from inside workspace
cd ~/colcon_ws
git init
git remote add origin https://github.com/patrickw135/pubsub.git
git fetch --all
git reset --hard origin/master


# Install your project specific python packages
# You can use pip3 as the install tool, eg:
pip3 install picamera 
# Or you can use the software installer:
sudo apt install python3-picamera


# again
sudo apt update && sudo apt upgrade -y
