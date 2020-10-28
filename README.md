# pubsub - Publisher & Subscriber Package
Publisher & Subscriber package as template package and source code.  

Created for the ROS Workspace 2020  
Roverentwicklung für Explorationsaufgaben  
Institute for Space Systems  
University of Stuttgart.

Created by [Patrick Winterhalder](),  
[IRS](https://www.irs.uni-stuttgart.de/en/), University of Stuttgart.  
  
  
  

## Workshop Prerequisistes
* Install [Ubuntu 20.04]()
* Install Visual Studio Code using [Ubuntu Software](https://wiki.ubuntuusers.de/Ubuntu_Software/)
* Install [Git](https://linuxconfig.org/how-to-install-git-on-ubuntu-20-04-lts-focal-fossa-linux) (no account required yet)
* Install [ROS2](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/) ("desktop" on PC, "base" on Raspberry Pi)
  * Do install _argcomplete_
  * No need for _ROS 1 bridge_, or _RMW implementations_
* Install and update _rosdep_:
  * `sudo apt install python3-rosdep2 -y`
  * `rosdep update`
* Work through ["Beginner: CLI Tools"](https://index.ros.org/doc/ros2/Tutorials/) tutorial
  * [Configuring your ROS 2 environment](https://index.ros.org/doc/ros2/Tutorials/Configuring-ROS2-Environment/):
    * Source setup files (underlay, overlay)
    * Configure .bashrc (shell startup script)
    * Add colcon_cd to .bashrc (shell startup script)
    * Check environment variables (check for correct installation)
    * Configure ROS_DOMAIN_ID (DDS Network Number)
  * Cover turtlesim, rqt, topics, services, actions

## During Workshop
* Create workspace:
  * Install [_colcon_](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#colcon): `sudo apt install python3-colcon-common-extensions -y`
* Create packag inside _~/{ws}/src_:
  * `ros2 pkg create --build-type ament_cmake <package_name>`
  * Go back up one layer: `cd ..`
  * Build workspace: `colcon build --symlink-install`
* Add [this](https://github.com/patrickw135/pubsub/blob/main/bashrc_addons.txt) to end of .bashrc (`sudo nano .bashrc`), find all instances of "`~/ws_overlay_foxy`" and replace it with your local path to your colcon workspace  
