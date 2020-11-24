# Add these lines to .bashrc, do this:
# Add the the following in between "#----" to the end of your .bashrc script

# To do this open .bashrc:
# sudo nano ~/.bashrc

# Move to end of .bashrc (arrow down)
# Highlight the lines below and copy (Ctrl+C)
# Paste into console at the end of .bashrc (Right click, paste or CTRL+LShift+V)



#----------------------------------------------------------------------------------
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
echo "******************************************************"

# Define DDS Channel
# (this is the channel your ROS system communicates at over your local network)
# To change the channel open the ROS Underlay setup file:
# sudo nano /opt/ros/foxy/setup.bash
# Here, add the following command
# export ROS_DOMAIN_ID=<your_channel_nr>
# eg.: export ROS_DOMAIN_ID=69
# Important: no spaces before and after "=" (in bash)

# Print DDS Settings
echo "ROS 2 DDS Settings:"
export ROS_DOMAIN_ID=69
echo "ROS_DOMAIN_ID:      "$ROS_DOMAIN_ID
echo "To change:          sudo nano /opt/ros/foxy/setup.bash"
echo "******************************************************"

# Source colcon directory jump:
#source /usr/share/colcon_cd/function/colcon_cd.sh
#export _colcon_cd_root=~/colcon_ws

# Print active Node & Topics
echo "ROS 2 Topics active:"
ros2 topic list
echo "******************************************************"
#echo "ROS 2 Nodes active:"
#ros2 node list
echo "******************************************************"
printf "\n"
echo "If your package is not listed (ros2 pkg list):"
echo " * make sure you are sourcing the correct workspace: .bashrc"
echo " * delete build/, install/ and rebuild w/o errors"
#----------------------------------------------------------------------------------
