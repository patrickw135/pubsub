# How to create custom ROS topics
This is a short instruction on how to create custom interfaces using ROS topics.  

For this at least two packages will be required:
* The python package contains your scripts (eg. ROS nodes)
* The CMake package contains the custom msg/srv/act files  

Up to now, the custom msg/srv/act files cannot be created inside python packages as this is not supported yet.  

## CMake Package (eg. /pubsub_msg)
This package makes up the basis for custom ROS interfaces and contains all custom msg/srv/act files. Additionally, the special files (_CMakeLists.txt_ and _package.xml_) describe how these interface files are to be used.  

This package must be created as a CMake package: `ros2 pkg create --build-type ament-cmake <package_name>`  

* msg/srv/act directory:
  * This directory contains the custom msg files (eg. CustomMsg1.msg)
* CMakeLists.txt:
  * This file describes how to build this package
* package.xml:
  * This file contains meta information about this package

__ANLEITUNG schreiben__  
__Wie müssen alle Dateien verändert werden?__


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
