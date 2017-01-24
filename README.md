# Introduction
This repository hosts the code of the Tango Ros Streamer application. It is an Android application for [Tango](https://get.google.com/tango/)-enabled devices.
Its main purpose is to provide Tango sensor data to the [ROS](http://wiki.ros.org/) ecosystem in order to easily use the Tango functionalities on robots.

This work is developed by [Intermodalics](http://www.intermodalics.eu/) in collaboration with [Ekumen](http://www.ekumenlabs.com/).
Do not hesitate to give us feedback if something is broken or if you think it lacks some features. The best way to do this is by adding issues to this repository.

# Installation
```
sudo apt-get install python-catkin-tools
mkdir -p ~/tango_ros_ws/src
cd ~/tango_ros_ws/src
git clone --recursive git@github.com:Intermodalics/tango_ros.git
cd ~/tango_ros_ws
catkin build
```  
To build the app: see [here](https://github.com/Intermodalics/tango_ros/blob/master/TangoRosStreamer/README.md).  
