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

    public static native boolean initRos(String masterUri, String ipAddress);

    public static native boolean isRosOk();

    /**
     * Interfaces to native OnCreate function.
     *
     * @param callerActivity the caller activity of this function.
     */
    public static native void initNode(Activity callerActivity, PublisherConfiguration publisherConfiguration);

    /**
     * Called when the Tango service is connected successfully.
     *
     * @param nativeTangoServiceBinder The native binder object.
     */
    public static native void onTangoServiceConnected(IBinder nativeTangoServiceBinder);

    public static native void tangoDisconnect();

    public static native void publish();
}