# How to create custom ROS topics
This is a short instruction on how to create custom interfaces using ROS topics.  

For this at least two packages will be required:
* The python package contains your scripts (eg. ROS nodes)
* The CMake package contains the custom msg/srv/act files  

The CMake package is required because the custom msg/srv/act files cannot be created inside the python package as this is not supported yet.  

__Table of Content__  
* [CMake Package (eg. /pubsub_msg)](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#cmake-package-eg-pubsub_msg)  
  * [Create CMake Package](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#1-create-cmake-package)
  * [Create Message Files](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#2-create-message-files)
  * [Configure CMakeLists.txt](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#3-configure-cmakeliststxt)
  * [Configure package.xml](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#4-configure-packagexml)
  * [Build Package](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#5-build-package)
  * [Source newly built workspace](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#6-source-newly-built-workspace)
  * [Check functionality](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#7-check-functionality)
* [Python Package (eg. /pubsub)](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#python-package-eg-pubsub)
  * [Create Python Package](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#1-create-python-package)
  * [Write Python Scripts](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#2-write-python-scripts)
  * [Configure package.xml](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#3-configure-packagexml)
  * [Build Package](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#4-build-package)
  * [Source newly built workspace](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#5-source-newly-built-workspace)
  * [Run scripts](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#6-run-scripts)
* [Sources](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#sources)

## CMake Package (eg. /pubsub_msg)
This package makes up the basis for custom ROS interfaces and contains all custom msg/srv/act files. Additionally, the special files (_CMakeLists.txt_ and _package.xml_) describe how these interface files are to be used.  

This package must be created as a CMake package: `ros2 pkg create --build-type ament-cmake <package_name>`  

* msg/srv/act directory:
  * This directory contains the custom msg files (eg. CustomMsg1.msg)
* CMakeLists.txt:
  * This file describes how to build this package
  * Configure this file according to this [instruction](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#cmakelists-txt)
* package.xml:
  * This file contains meta information about this package
  * Configure this file according to this [instruction](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#package-xml)

__ANLEITUNG schreiben__  
__Wie müssen alle Dateien verändert werden?__

### 1. Create CMake Package

### 2. Create Message Files

### 3. Configure CMakeLists.txt

### 4. Configure package.xml

### 5. Build Package

### 6. Source newly built workspace

### 7. Check functionality
Check functionality of your messages by creataing a topic using your newly created message:  
* CustomMsg1:  
`ros2 topic pub /chatter1 pubsub_msg/CustomMsg1 "{temperature: {24.1234, 25.9876}, pressure: {1012.556, 1013.987}, humidity: {0.002, 0.001}}" --rate 1`
* Response:  
_publisher: beginning loop  
publishing #1: pubsub_msg.msg.CustomMsg1(temperature=[24.12339973449707, 25.987600326538086], pressure=[1012.5560302734375, 1013.9869995117188], humidity=[0.0020000000949949026, 0.0010000000474974513])..._  

* CustomMsg2:  
`ros2 topic pub /chatter2 pubsub_msg/CustomMsg2 "{pitch_ctrl: 33.33, yaw_ctrl: 0.5}" --rate 1`
* Response:  
_publisher: beginning loop  
publishing #1: pubsub_msg.msg.CustomMsg2(pitch_ctrl=33.33, yaw_ctrl=0.5)..._



## Python Package (eg. /pubsub)
This package contains your scripts, programs and libraries. After building the workspace (`colcon build`) the custom messages are available to all other packages.  

This package can be created as a CMake (C++) package or as a python package depending on your coding preference.
* C++:    `ros2 pkg create --build-type ament-cmake <package_name>`
* Python: `ros2 pkg create --build-type ament-python <package_name>`  


<package_name> directory:
* This directory contains your python scripts (eg. listener.py)
* Also place the non-standard libraries in this directory and import the library in your python scripts


__ANLEITUNG schreiben__  
__Wie importiert man die msg files__ 

### 1. Create Python Package

### 2. Write Python Scripts

### 3. Configure package.xml

### 4. Build Package

### 5. Source newly built workspace

### 6. Run scripts
* Talker:  
`ros2 run pubsub talker`
* Listener:
`ros2 run pubsub listener`

The talker console should print the sent data while the listener console should print the received data. These should match.


## Sources  
[ROS2 Tutorial](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#creating-custom-ros-2-msg-and-srv-files)  
[theconstructsim custom messages](https://www.theconstructsim.com/ros2-tutorials-7-how-to-create-a-ros2-custom-message-new/)  
