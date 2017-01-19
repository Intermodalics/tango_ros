## TangoRos App

### The roscpp\_android\_ndk

* Download the roscpp\_android\_ndk [here](http://wiki.ros.org/android_ndk/Tutorials/Building%20The%20Example%20Applications%20using%20the%20Binary%20Distribution).
(See the second section *Get the files*)

* The roscpp\_android\_ndk should be placed inside ```tango_ros/third_party/```.

* Replace the original Android.mk file of the roscpp\_android\_ndk by the one present in the Github repository in ```tango_ros/third_party/roscpp_android_ndk/```.

### The OpenCV sdk

* Download the OpenCV sdk [here](http://docs.opencv.org/2.4/doc/tutorials/introduction/android_binary_package/O4A_SDK.html#get-the-opencv4android-sdk) (choose version 3.1.0).

* Unpack it and copy the content of ```OpenCV-android-sdk/sdk/native/``` into ```tango_ros/third_party/OpenCV_sdk_native/```.

### Miniglog

* From the tango_ros directory:  
```$ cd third_party/miniglog```  
```$ ./build.sh```  

### Building the app with android studio

* Download Android Studio [here](https://developer.android.com/studio/index.html).

* When starting Android Studio import the project by selecting the RosApp directory.

* In your local.properties file check that the paths to your android sdk and ndk are set properly.  
Example:  
ndk.dir=/opt/android-ndk-r10b  
sdk.dir=/opt/android-sdk-linux  

* Plug an android device to your desktop.

* Check the TangoCore version of your device (Settings->Apps->Tango Core). The app was tested on Yildun release.

* Press the green arrow in Android Studio to build and install the app on the device.

### Running the app

* Launch a roscore on your desktop.

* Enter the ros master URI when the application is asking and press _Connect_.
![screenshot_2016-11-25-08-56-08](https://cloud.githubusercontent.com/assets/12640723/20618636/89512010-b2f0-11e6-9343-770e0170a22c.png)

* Open rviz with the config file located at RosApp/tango_ros.rviz to visualize the different tango data (device pose, pointcloud, images). You can enable/disable published data thanks to the app's switch buttons, press _Apply_ to applie your changes.
![screenshot_2016-11-25-08-57-21](https://cloud.githubusercontent.com/assets/12640723/20618637/8b53b1e8-b2f0-11e6-8f21-618fe99f238c.png)


