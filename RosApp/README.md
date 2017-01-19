## Tango Ros Streamer App

### The roscpp\_android\_ndk

* Download the roscpp\_android\_ndk [here](http://wiki.ros.org/android_ndk/Tutorials/Building%20The%20Example%20Applications%20using%20the%20Binary%20Distribution).
(See the second section *Get the files*)

* Unpack it and copy the content of the ```roscpp\_android\_ndk``` folder inside ```tango_ros/third_party/roscpp_android_ndk/```, except the ```Android.mk``` file.
```
tar -xvzf ~/Downloads/roscpp_android_ndk.tar.gz -C ~/Downloads/
rsync -av --progress ~/Downloads/roscpp_android_ndk/ ~/tango_ros_ws/src/tango_ros/third_party/roscpp_android_ndk/ --exclude Android.mk
```

### The OpenCV sdk

* Download the OpenCV sdk [here](http://docs.opencv.org/2.4/doc/tutorials/introduction/android_binary_package/O4A_SDK.html#get-the-opencv4android-sdk) (tested with version 3.1.0).

* Unpack it and copy the content of the ```OpenCV-android-sdk/sdk/native/``` folder into ```tango_ros/third_party/OpenCV_sdk_native/```.
```
unzip ~/Downloads/OpenCV-3.1.0-android-sdk.zip -d ~/Downloads/
mkdir ~/tango_ros_ws/src/tango_ros/third_party/OpenCV_sdk_native
cp -r ~/Downloads/OpenCV-android-sdk/sdk/native/* ~/tango_ros_ws/src/tango_ros/third_party/OpenCV_sdk_native/
```

### Building the app with android studio

* Download Android Studio [here](https://developer.android.com/studio/index.html).

* When starting Android Studio import the project by selecting the RosApp directory.

* In your local.properties file check that the paths to your android sdk and ndk are set properly.  
Example:  
ndk.dir=/opt/android-ndk-r10b  
sdk.dir=/opt/android-sdk-linux  

* Plug an android device to your desktop. Your device should be Tango-enabled.

* Check the TangoCore version of your device (Settings->Apps->Tango Core). The minimum version required to run Tango Ros Streamer is Yildun.

* Press the green arrow in Android Studio to build and install the app on the device.

### Running the app

* Launch a roscore on your desktop.

* For the first run, the app will ask you to set some settings. Press DONE once you did it.
![screenshot_2017-01-19-14-19-52](https://cloud.githubusercontent.com/assets/12640723/22108286/b1cd420c-de52-11e6-9130-b65bf4be3f94.png)

* You can enable/disable published data via the app switch buttons located in a right drawer.
![screenshot_2017-01-19-14-20-37](https://cloud.githubusercontent.com/assets/12640723/22108292/b9b1990a-de52-11e6-9426-0662b9b1cd65.png)

* You can run rviz with the config file located at ```tango_ros/RosApp/tango_ros.rviz``` to visualize Tango data (device pose, pointcloud, images).
