# pubsub - Publisher & Subscriber Package
Publisher & Subscriber package as template package and source code.  

Created for ROS Workshop 2020  
Roverentwicklung für Explorationsaufgaben  
Institute for Space Systems  
University of Stuttgart.

Created by Patrick Winterhalder,  
[IRS](https://www.irs.uni-stuttgart.de/en/), University of Stuttgart.  
  
  
  

## Workshop Prerequisites
* Install [Ubuntu 20.04]()
* Install Visual Studio Code using [Ubuntu Software](https://wiki.ubuntuusers.de/Ubuntu_Software/)
* Install [Git](https://linuxconfig.org/how-to-install-git-on-ubuntu-20-04-lts-focal-fossa-linux) (no account required yet)
* Install [ROS2](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/) ("desktop" on PC, "base" on Raspberry Pi). Do install _argcomplete_, no need for _ROS 1 bridge_ or _RMW implementations_.
* Install and update _rosdep_:
  * `sudo apt install python3-rosdep2 -y`
  * `rosdep update`
  * (`sudo rosdep init`)
* Work through ["Beginner: CLI Tools"](https://index.ros.org/doc/ros2/Tutorials/) tutorial
  * [Configuring your ROS 2 environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/):
    * Source setup files (underlay, overlay)
    * Configure .bashrc (shell startup script)
    * Add colcon_cd to .bashrc (shell startup script)
    * Check environment variables (check for correct installation)
    * Configure ROS_DOMAIN_ID (DDS Network Number)
  * Cover turtlesim, rqt, topics, services, actions

## Install Instructions
* Move to colcon workspace: `cd <workspace_path>`
* Clone repository: `git clone git://github.com/patrickw135/pubsub.git .` (include the . at the end)
* Build workspace: `colcon build`  
__Note:__ Only the files inside _src/_ are of importance.
* Remember to run `source ~/<workspace_path>/install/local_setup.sh` after every build. Best would be to [add this command to _.bashrc_](https://github.com/patrickw135/pubsub/blob/master/bashrc_addons.txt) which is run everytime you start a new console.


## During Workshop
* Create workspace:
  * Install [_colcon_](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#colcon): `sudo apt install python3-colcon-common-extensions -y`
* Create packag inside _~/{workspace_name}/src_:
  * `ros2 pkg create --build-type [ament_cmake, ament-python] <package_name>`
  * Go back up one layer: `cd ..`
  * Build workspace: `colcon build --symlink-install`
* Add [this](https://github.com/patrickw135/pubsub/blob/main/bashrc_addons.txt) to end of .bashrc (`sudo nano .bashrc`), find all instances of "`~/ws_overlay_foxy`" and replace it with your local path to your colcon workspace  
* [Instruction](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md) on how to create a custom message to interface between nodes

