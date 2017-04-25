#! /bin/bash -x
adb root
adb remount

LIB_DIR="$HOME/tango_ros_ws/src/tango_ros/TangoRosStreamer/nodelet_manager/src/main/obj/local/armeabi-v7a"
TEST_DIR="$HOME/tango_ros_ws/src/tango_ros/TangoRosStreamer/tango_ros_test/src/main/libs/armeabi-v7a"
DEST_DIR="/data/local/tmp"
MASTER_URI="im-desktop-005:11311"
DEVICE_IP="192.168.168.185"
DESKTOP_TEST_DIR="$HOME/tango_ros_ws/devel/lib/tango_ros_native"

# Push libs and test executable to the device.
adb push $LIB_DIR/libminiglog.a $DEST_DIR/
adb push $LIB_DIR/libroscpp_android_ndk.a $DEST_DIR/
adb push $LIB_DIR/libtango_3d_reconstruction.so $DEST_DIR/
adb push $LIB_DIR/libtango_ros_native.a $DEST_DIR/
adb push $LIB_DIR/libtango_support_api.so $DEST_DIR/
adb push $TEST_DIR/tango_ros_test $DEST_DIR/
adb shell chmod 775 $DEST_DIR/test_tango_ros_native

# Start the test on device.
adb shell LD_LIBRARY_PATH=$DEST_DIR $DEST_DIR/tango_ros_test __master:=http://$MASTER_URI __ip:=$DEVICE_IP & DEVICE_TEST=&!

# Sleep some time to be sure that the test on device is running.
sleep 5
# Start the test on desktop.
$DESKTOP_TEST_DIR/tango_ros_native_test & DESKTOP_TEST=$!

# Wait till both tests are finished.
wait $DEVICE_TEST
wait $DESKTOP_TEST