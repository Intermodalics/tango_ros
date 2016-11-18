#! /bin/bash

LIB_DIR="$HOME/TangoApps/TangoxRos/app/src/main/libs/armeabi-v7a"
DEST_DIR="/data/local/tmp"
MASTER_URI="im-desktop-005:11311"
DEVICE_IP="192.168.168.184"

CMD="adb push $LIB_DIR/libtango_ros_android.so $DEST_DIR/"
echo $CMD
$CMD

CMD="adb push $LIB_DIR/libtango_ros_native.so $DEST_DIR/"
echo $CMD
$CMD

CMD="adb push $LIB_DIR/libtango_support_api.so $DEST_DIR/"
echo $CMD
$CMD

CMD="adb push $LIB_DIR/test_tango_ros_native $DEST_DIR/"
echo $CMD
$CMD

CMD="adb shell chmod 775 $DEST_DIR/test_tango_ros_native"
echo $CMD
$CMD

CMD="adb shell LD_LIBRARY_PATH=$DEST_DIR $DEST_DIR/test_tango_ros_native __master:=http://$MASTER_URI __ip:=$DEVICE_IP"
echo $CMD
$CMD & DEVICE_TEST=&!
sleep 3

DESKTOP_TEST_DIR="$HOME/TangoApps/tango_ros_catkin/devel/lib/tango_ros_native"
CMD="$DESKTOP_TEST_DIR/tango_ros_native-test_tango_ros"
echo $CMD
$CMD & DESKTOP_TEST=$!

wait $DEVICE_TEST
wait $DESKTOP_TEST