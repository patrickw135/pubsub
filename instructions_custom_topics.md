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

This will result in an empty package structure:
* msg/srv/act directory:
  * This directory contains the custom msg files (eg. CustomMsg1.msg)
* CMakeLists.txt:
  * This file describes how to build this package
  * Configure this file according to this [instruction](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#cmakelists-txt)
* package.xml:
  * This file contains meta information about this package
  * Configure this file according to this [instruction](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#package-xml)


### 1. Create CMake Package
* Move to your colcon workspace's src directory: `cd <workspace_path>/src`
* (For example: `cd ~/colcon_ws/src`)
* Create CMake package: `ros2 pkg create --build-type ament_cmake <package_name>
* (Here: `ros2 pkg create --build-type ament_cmake pubsub_msg`)


### 2. Create Message Files
* If not already available create msg directory inside package directory.  
Resulting structure: <workspace_name>/src/<package_name>/msg
* Move to newly created msg direcrory
* Create your own custom message files, eg. _CustomMsg1.msg_.  
Give your files comprehensible names, eg. _Epossetvalues.msg_


### 3. Configure CMakeLists.txt
Open CMakeLists.txt and ad these lines before `if(BUILD_TESTING)`:  
`find_package(builtin_interfaces REQUIRED)`  
`find_package(rosidl_default_generators REQUIRED)`  
`find_package(std_msgs REQUIRED)`  
`find_package(rclcpp REQUIRED)`  
Then also add custom lines depending your package, here the custom message/service files are added:  
`rosidl_generate_interfaces(${PROJECT_NAME}`  
 `  "msg/CustomMsg1.msg"`  
 `  "msg/CustomMsg2.msg"`  
 `  DEPENDENCIES builtin_interfaces`  
 ` )`

### 4. Configure package.xml
In order to let the build system know what this package depends on add these lines to _package.xml_:  
```xml
   <!-- ADD THESE LINES: START HERE -->
   <build_depend>builtin_interfaces</build_depend>
   <build_depend>rosidl_default_generators</build_depend>
   <exec_depend>builtin_interfaces</exec_depend>
   <exec_depend>rosidl_default_runtime</exec_depend>
   <member_of_group>rosidl_interface_packages</member_of_group>
   <!-- END HERE -->
```

### 5. Build Package
* Move back to the workspace's most top layer: `cd ~/<workspace_path>`
* Build workspace: `colcon build`
* Sucessful response:  
_Starting >>> pubsub  
Starting >>> pubsub_msg  
Finished <<< pubsub [0.85s]                                              
Finished <<< pubsub_msg [1.09s]  
Summary: 2 packages finished [1.56s]_


### 6. Source newly built workspace
* Run: `source ~/<workspace_path>/install/local_setup.bash`
* If you already [updated your .bashrc file](https://github.com/patrickw135/pubsub/blob/master/bashrc_addons.txt) you can close all open consoles and start a new console (Ctrl+Alt+T). This will source your workspace automatically, as .bashrc is run every time you start a console.  
__Important__: If you use multiple workspaces make sure you have the wanted workspace defined in .bashrc! Otherwise the changes introduced when building will not be available.


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


### 1. Create Python Package
* Move to your workspace's source directory: `cd <workspace_path>/src`
* Create python package: `rps2 pkg create --build-type ament_python <pkg_name>`

### 2. Write Python Scripts
When using custom interfaces in python scripts these must be imported into the python script first  
```python
from <package_name>.msg import <message_name>
```
replacing `<package_name>` with the package containing the custom message and `<message_name>` with the message file name (excluding the file ending .msg).  
However, in order to be able to import the custom message types, `<message_name>` must first be known to the ROS system. This was established when creating the [CMake package](https://github.com/patrickw135/pubsub/blob/master/instructions_custom_topics.md#cmake-package-eg-pubsub_msg) containing the custom message. Additionally, you must add this dependency to the _package.xml_ of this package as stated in the next chapter.

### 3. Configure package.xml
In addition to importing the message type into your python script you must also configure _package.xml_ adding the package dependency of where you inherite the custom message from. Add this line to _package.xml_:  
```xml
   <depend>std_msgs</depend>
   <!-- CUSTOM LINE -->
   <!-- This is custom for the package you depend on -->
   <exec_depend>package_name</exec_depend>
```
exchanging _package_name_ with the source package of the custom message type (here _pubsub_msg_), e.g.:  
```xml
   <depend>std_msgs</depend>
   <!-- CUSTOM LINE -->
   <!-- This is custom for the package you depend on -->
   <exec_depend>pubsub_msg</exec_depend>
```

### 4. Build Package
Now you can build the Python package.  
* Move to your workspace's root: `cd ~/<workspace_path>`
* Build workspace: `colcon build --symlink-install`

### 5. Source newly built workspace
* Run: `source ~/<workspace_path>/install/local_setup.bash`
* If you already [updated your .bashrc file](https://github.com/patrickw135/pubsub/blob/master/bashrc_addons.txt) you can close all open consoles and start a new console (Ctrl+Alt+T). This will source your workspace automatically, as .bashrc is run every time you start a console.  
__Important__: If you use multiple workspaces make sure you have the wanted workspace defined in .bashrc! Otherwise the changes introduced when building will not be available.

### 6. Run scripts
* Run Talker: `ros2 run pubsub talker`
* Run Listener: `ros2 run pubsub listener`

The talker console should print the sent data while the listener console should print the received data. These should match.

If anything is unclear, compare this instruction material to the files in `/pubsub` and `/pubsub_msg`.  

## Sources  
[ROS2 Tutorial](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#creating-custom-ros-2-msg-and-srv-files)  
[theconstructsim custom messages](https://www.theconstructsim.com/ros2-tutorials-7-how-to-create-a-ros2-custom-message-new/)  
