## TangoRos App

### Installing the roscpp\_android\_ndk

* Follow the tutorial here: http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages to install the roscpp\_android\_ndk.

* The roscpp\_android\_ndk should be inside the TangoRos directory.

* In `TangoRos/sample_app/custom_rules.xml`, set the path to the roscpp\_android\_ndk.  
By default it is set to:  
`<env key="NDK_MODULE_PATH" path="/home/intermodalics/TangoApps/TangoRos/" />`

* In `TangoRos/sample_app/local.properties`, set the path to the Android SDK.  
By default it is set to:  
`sdk.dir=/opt/android-sdk-linux`

### Building the app

* Run the following command inside the sample\_app directory:  
`ant debug`


### Installing the app

* Run the following command inside the sample\_app directory:  
`adb install -r bin/sample_app-debug.apk`


### More Help
This page http://wiki.ros.org/android_ndk contains several tutorials on how to use native ros code for Android.  
This page https://developer.android.com/reference/android/app/NativeActivity.html explains how to write a Native Activity for Android.  
Enjoy!
