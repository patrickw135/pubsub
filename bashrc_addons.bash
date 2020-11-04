# Add these lines to .bashrc, do this:
# sudo nano ~/.bashrc
# move to end of .bashrc (arrow down)
# Highlight the lines below and copy
# Paste into console at the end of .bashrc
# Console pasting: CTRL+LShift+V

# ROS 2 SETUP
echo "ROS 2 Setup:"

# Source Underlay
source /opt/ros/foxy/setup.bash
#Print Underlay
echo "ROS Underlay:       source /opt/ros/foxy/setup.bash"

# Source all Overlays:
#source ~/ws_overlay_foxy/install/local_setup.bash
source ~/colcon_ws/install/local_setup.bash

#Print Overlays
echo "ROS Overlay:        source <ws>/install/local_setup.bash"
echo "Overlays:           'colcon_ws'"
echo "******************************************************"

# ROS 2 Settings
# Print ROS Variables
#printenv | grep -i ROS
echo "ROS_VERSION:        "$ROS_VERSION
echo "ROS_PYTHON_VERSION: "$ROS_PYTHON_VERSION
echo "ROS_DISTRO:         "$ROS_DISTRO
echo "******************************************************"

# Define DDS Channel
# (this is the channel your ROS system communicates at over your network)
# For this, open the ROS Underlay setup file:
# sudo nano /opt/ros/foxy/setup.bash
# Here, add the following command
# export ROS_DOMAIN_ID=<your_channel>
# eg.: export ROS_DOMAIN_ID=69

# Print DDS Settings
echo "ROS 2 DDS Settings:"
export ROS_DOMAIN_ID=69
echo "ROS_DOMAIN_ID:      "$ROS_DOMAIN_ID
echo "To change:          sudo nano /opt/ros/foxy/setup.bash"
echo "******************************************************"

# Source colcon directory jump:
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/colcon_ws

# Print active Node & Topics
echo "ROS 2 Topics active:"
ros2 topic list
echo "******************************************************"
#echo "ROS 2 Nodes active:"
#ros2 node list
echo "******************************************************"
printf "\n"
