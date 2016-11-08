package eu.intermodalics.tangoxros;

import android.app.Activity;
import android.os.IBinder;
import android.util.Log;

/**
 * Interfaces between native C++ code and Java code.
 */
public class JNIInterface {
    static {
        if (TangoInitializationHelper.loadTangoSharedLibrary() ==
                TangoInitializationHelper.ARCH_ERROR) {
            Log.e("TangoJNINative", "ERROR! Unable to load libtango_client_api.so!");
        }
        System.loadLibrary("tango_ros_native");
    }

    /**
     * Initializes ROS given the ros master URI and the device IP address.
     * @param masterUri URI of the ROS master, format is __master:=http://local-desktop:11311.
     * @param ipAddress IP address of the device, format is __ip:=192.168.168.185.
     * @return true is initialization was successful.
     */
    public static native boolean initRos(String masterUri, String ipAddress);

    /**
     * @return true if ROS is OK, false if it has shut down.
     */
    public static native boolean isRosOk();

    /**
     * Initializes the tango-ros node. Specifically it:
     *   * creates the node and its publishers.
     *   * check that the tango version is correct.
     * initRos should always be called before.
     * @param callerActivity the caller activity of this function.
     */
    public static native void initNode(Activity callerActivity);

    /**
     * Called when the Tango service is connected successfully.
     *
     * @param nativeTangoServiceBinder The native binder object.
     */
    public static native void onTangoServiceConnected(IBinder nativeTangoServiceBinder);

    /**
     * Disconnects from Tango.
     */
    public static native void tangoDisconnect();

    /**
     * Publishes the available tango data (device pose, point cloud, images).
     */
    public static native void publish();
}