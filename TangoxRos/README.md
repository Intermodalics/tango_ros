## TangoRos App

### The roscpp\_android\_ndk

* Download the roscpp\_android\_ndk [here](http://wiki.ros.org/android_ndk/Tutorials/Building%20The%20Example%20Applications%20using%20the%20Binary%20Distribution).
(See the second section *Get the files*)

* The roscpp\_android\_ndk should be placed inside the TangoxRos directory.

* Replace the original Android.mk file of the roscpp\_android\_ndk by the one present in the Github repository under TangoxRos/roscpp_android_ndk/.

### The OpenCV sdk

* Download the OpenCV sdk [here](http://docs.opencv.org/2.4/doc/tutorials/introduction/android_binary_package/O4A_SDK.html#get-the-opencv4android-sdk).

* Unpack it and copy the content of OpenCV-android-sdk/sdk/native/ into TangoxRos/OpenCV_sdk_native.

### Building the app with android studio

* Download Android Studio (version 2.2 seems to not work properly with this project, you can download previous versions [here](http://tools.android.com/system/app/pages/subPages?path=/download/studio/builds)).

* When starting Android Studio import the project by selecting the TangoxRos directory.

* In your local.properties file check that the paths to your android sdk and ndk are set properly.  
Example:  
ndk.dir=/opt/android-ndk-r10b  
sdk.dir=/opt/android-sdk-linux  

* Plug an android device to your desktop and press the green arrow in Android Studio to build and install the app on the device.

### Running the app

* Launch a roscore on your desktop.

* Enter the ros master URI when the application is asking and press connect.

* Open rviz with the config file located at TangoxRos/tango_ros.rviz to visualize the different tango data (device pose, point cloud, image).
