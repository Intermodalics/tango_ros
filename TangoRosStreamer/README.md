## Building the Tango Ros Streamer application

### Installing third-party dependencies

#### The roscpp\_android\_ndk

* Follow this [tutorial](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) to build the roscpp android ndk in portable mode.

* Once it ended succesfully, copy the the content of the output ```roscpp_android_ndk``` folder into ```tango_ros/third_party/roscpp_android_ndk/```, except the ```Android.mk``` file and the ```share``` folder.
```
cp -r ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/lib/ ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
cp -r ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/include/ ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
cp ~/ros-android-ndk/roscpp_android/output/roscpp_android_ndk/Application.mk ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/
```

### Building the app with Android Studio

* Download Android Studio [here](https://developer.android.com/studio/index.html).

* When starting Android Studio import the project by selecting the TangoRosStreamer directory.

* In your local.properties file check that the paths to your Android SDK and NDK are set properly. The Gradle set-up relies on the following variables that need to set up. This can be done using the auto-generated local.properties file or gradle properties in the HOME folder (~/.gradle/gradle.properties).  
Example:
```
ndk.dir=/opt/android-ndk-r10b 
sdk.dir=/opt/android-sdk-linux
```

* Plug an Android device to your desktop. Your device should be Tango-enabled.

* Check the Tango Core version of your device (Settings->Apps->Tango Core). The minimum version required to run Tango Ros Streamer is Yildun (you will find the Tango release history [here](https://developers.google.com/tango/release-notes)).

* Press the green arrow in Android Studio to build and install the app on the device.

### Running the app

* Launch a roscore on your desktop.

* On the first run, the app will ask you to set some settings. Press DONE once the set-up is completed.
![screenshot_2017-01-19-16-41-47](https://cloud.githubusercontent.com/assets/12640723/22114676/a08ee398-de6a-11e6-84b3-4c72d7398942.png)

* You can enable/disable published data at runtime via the app switch buttons located in a right drawer.
![screenshot_2017-01-19-14-20-37](https://cloud.githubusercontent.com/assets/12640723/22108292/b9b1990a-de52-11e6-9426-0662b9b1cd65.png)

* You can run rviz with the config file located at ```tango_ros/TangoRosStreamer/tango_ros.rviz``` to visualize Tango data (device pose, pointcloud, images,...).
