# Introduction
This repository hosts the code of the Tango Ros Streamer application.  
It is an Android application for [Tango](https://get.google.com/tango/)-enabled devices.
Its main purpose is to provide Tango sensor data to the [ROS](http://wiki.ros.org/) ecosystem in order to easily use the Tango functionalities on robots.  
You can read the ROS wiki [here](http://wiki.ros.org/tango_ros_streamer).

This work is developed by [Intermodalics](http://www.intermodalics.eu/) in collaboration with [Ekumen](http://www.ekumenlabs.com/) and [Google Tango](https://get.google.com/tango/).  
Do not hesitate to give us feedback if something is broken or if you think it lacks some features. The best way to do this is by adding issues to this repository.

# Kickstart
The app is available in Google's Play Store: https://play.google.com/store/apps/details?id=eu.intermodalics.tango_ros_streamer

# Step-by-step instructions
For this guide, we assume that you have a clean installation of Ubuntu 14.04 and we will use ROS Indigo. However, the app should also work on Ubuntu 16.04 in combination with ROS Kinetic.

## Install ROS
Follow this [guide](http://wiki.ros.org/indigo/Installation/Ubuntu) and go for the Desktop -- Full Install:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Building roscpp_android
Next, we need to build ```roscpp_android```. A more detailed guide is available [here](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages).

```
sudo apt-get update
sudo apt-get install curl git
curl -sSL https://get.docker.com/ | sudo sh
mkdir ~/ros-android-ndk
cd ~/ros-android-ndk
git clone https://github.com/ekumenlabs/roscpp_android.git
cd roscpp_android
./do_docker.sh --portable
```

Note that the last command take will take long time to complete. Once this has ended, copy the the content of the output ```roscpp_android_ndk``` folder into ```tango_ros/third_party/roscpp_android_ndk/```, except the ```Android.mk``` file and the ```share``` folder.

```
cp -r ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/lib/ ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
cp -r ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/include/ ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
cp ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/Application.mk ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
```

## Install Java 
```
sudo apt-get update
sudo apt-get install openjdk-6-jdk
echo "export JAVA_HOME=/usr/lib/jvm/java-6-openjdk-amd64/" >> ~/.bashrc
```

## Build the dependencies
```
sudo apt-get install git python-catkin-tools
mkdir -p ~/tango_ros_ws/src
cd ~/tango_ros_ws/src
git clone --recursive git@github.com:Intermodalics/tango_ros.git
cd ~/tango_ros_ws
catkin build
```  

## Android Studio
Download from [here](https://developer.android.com/studio/index.html), unzip and run:

```
cd
unzip ~/Downloads/android-studio-ide-145.3537739-linux.zip
./android-studio/bin/studio.sh
```

After lauch, open the Android SDK Manager. From SDK Platforms, install Android 4.4 and 5.1 (API level 19 + 22). From SDK Tools, install `NDK` and `Android SDK Build-Tools 21.1.2`. By default, this will be installed to ```$HOME/Android/Sdk```. 

Now extend your PATH variable so that ```ndk-build``` can be executed:
```
echo "export PATH=$PATH:$HOME/Android/Sdk/ndk-bundle" >> ~/.bashrc
source ~/.bashrc
```

## Building the app
In Android Studio, choose "Import project" and select the app folder (```~/tango_ros_ws/src/tango_ros/TangoRosStreamer```).

For more details on building and running the app, see [here](https://github.com/Intermodalics/tango_ros/blob/master/TangoRosStreamer/README.md).  
