# Introduction
This repository hosts the code of the Tango Ros Streamer application.  
It is an Android application for [Tango](https://get.google.com/tango/)-enabled devices.
Its main purpose is to provide Tango sensor data to the [ROS](http://wiki.ros.org/) ecosystem in order to easily use the Tango functionalities on robots.  
You can read the ROS wiki [here](http://wiki.ros.org/tango_ros_streamer).

This work is developed by [Intermodalics](http://www.intermodalics.eu/) in collaboration with [Ekumen](http://www.ekumenlabs.com/) and [Google Tango](https://get.google.com/tango/).  
Do not hesitate to give us feedback if something is broken or if you think it lacks some features. The best way to do this is by adding issues to this repository.

# Known projects using TangoRosStreamer
* [tangobot](https://github.com/ekumenlabs/tangobot/) : An android application to navigate with the [Turtlebot](http://www.turtlebot.com/) using Tango.

# Kickstart
The app is available in Google's Play Store: https://play.google.com/store/apps/details?id=eu.intermodalics.tango_ros_streamer  
It can be installed on any Tango-enabled device. Note that the minimum Tango version required to run Tango Ros Streamer is Yildun (you will find the Tango release history [here](https://developers.google.com/tango/release-notes)). To check the Tango version of your device go to Settings->Apps->Tango Core. 

# Running the app

* Launch a roscore on your desktop.

* On the first run, the app will ask you to set some settings. Press DONE once the set-up is completed.
![screenshot_2017-01-19-16-41-47](https://cloud.githubusercontent.com/assets/12640723/22114676/a08ee398-de6a-11e6-84b3-4c72d7398942.png)

* You can enable/disable published data at runtime via the app switch buttons located in a right drawer.
![screenshot_2017-01-19-14-20-37](https://cloud.githubusercontent.com/assets/12640723/22108292/b9b1990a-de52-11e6-9426-0662b9b1cd65.png)

* You can run rviz with the config file located at ```tango_ros/TangoRosStreamer/tango_ros.rviz``` to visualize Tango data (device pose, pointcloud, images,...).

# Installation from source
For this guide, we assume that you have a clean installation of Ubuntu 14.04 and we will use ROS Indigo. However, the app should also work on Ubuntu 16.04 in combination with ROS Kinetic.

## Install ROS
Follow this the guide for [Indigo (14.04)](http://wiki.ros.org/indigo/Installation/Ubuntu) or [Kinetic (16.04)](http://wiki.ros.org/kinetic/Installation/Ubuntu) and go for the Desktop -- Full Install:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install ros-indigo-rosjava-build-tools
sudo apt-get install ros-indigo-genjava
```
Replace indigo by kinetic if you are using Ubuntu 16.04.

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

Note that the last command will take long time to complete. Once this has ended, copy the the content of the output ```roscpp_android_ndk``` folder into ```tango_ros/third_party/roscpp_android_ndk/```, except the ```Android.mk``` file and the ```share``` folder.

```
cp -r ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/lib/ ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
cp -r ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/include/ ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
cp ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/Application.mk ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
```

## Building the app
First clone the repository at the correct location:
```
mkdir -p ~/tango_ros_ws/src
cd ~/tango_ros_ws/src
git clone --recursive git@github.com:Intermodalics/tango_ros.git
cd ~/tango_ros_ws
``` 

Create a local.properties file
```
touch ~/tango_ros_ws/src/tango_ros/TangoRosStreamer/local.properties
```
In this file, write the path to your Android SDK and NDK in the following way:
```
ndk.dir=/opt/android-ndk-r10b
sdk.dir=/opt/android-sdk-linux
```

Install catkin tools if necessary.
```
sudo apt-get install git python-catkin-tools
```

Build the app and generate its .apk file.
```
catkin build --no-jobserver
``` 

Plug your device to your desktop and install the app on your device using [adb](http://developer.android.com/studio/command-line/adb.html).
```
adb install -r -d ~/tango_ros_ws/src/tango_ros/TangoRosStreamer/app/build/outputs/apk/app-debug.apk
```

# Developping with Android Studio

We recommend using Android Studio as a development tool. 

## Installation of Android Studio
The steps detailed below are based on this [installation guide](http://wiki.ros.org/android/kinetic/Android%20Studio/Download).

For Android Studio we need Java, so let's install this first. On Ubuntu 14.04, we need to install openjdk-7-jdk, while on Ubuntu 16.04 we recommend to use openjdk-8-jdk instead.
```
sudo apt-get update
sudo apt-get install openjdk-7-jdk
echo "export JAVA_HOME=/usr/lib/jvm/java-7-openjdk-amd64/" >> ~/.bashrc
```

Subsequently, download Android Studio from [here](https://developer.android.com/studio/index.html) and unzip (for example) to /opt/android-studio:

```
cd /opt
sudo unzip ~/Downloads/android-studio-ide-145.3537739-linux.zip
```

Let's add it to the path for convenience:
```
echo export PATH=\${PATH}:/opt/android-sdk/tools:/opt/android-sdk/platform-tools:/opt/android-studio/bin >> ~/.bashrc
```

Pick a directory where to store the Android SDK, for example here:
```
sudo mkdir /opt/android-sdk
sudo chown $(whoami) /opt/android-sdk
echo export ANDROID_HOME=/opt/android-sdk >> ~/.bashrc
```

## Building the app with Android Studio

Launch Android Studio:
```
source ~/.bashrc
source ~/tango_ros_ws/devel/setup.bash
studio.sh
```

After launch, open the Configure -- SDK Manager. From SDK Platforms, install Android 4.4 and 5.1 (API level 19 + 22). From SDK Tools, install `NDK` and `Android SDK Build-Tools 21.1.2`. 

Now extend your PATH variable so that ```ndk-build``` can be executed:
```
echo "export PATH=\{$ANDROID_HOME}/ndk-bundle:\${PATH}" >> ~/.bashrc
source ~/.bashrc
```

In Android Studio, choose "Import project" and select the app folder (```~/tango_ros_ws/src/tango_ros/TangoRosStreamer```). 

In your local.properties file check that the paths to your Android SDK and NDK are set properly. The Gradle set-up relies on the following variables that need to set up. This can be done using the auto-generated local.properties file or gradle properties in the HOME folder (~/.gradle/gradle.properties).  
Example:
```
ndk.dir=/opt/android-ndk-r10b
sdk.dir=/opt/android-sdk-linux
```

Plug your device to your desktop and press the green arrow in Android Studio. It will build and install the app on the device.
