package org.ros.node;

import android.app.Activity;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

/**
 * A java wrapper to load and run a native-code ROS node.
 *
 * Note: there are no actual native methods declared in this class. We only define an interface. The native methods should be declared in the child class.
 *
 * @author ecorbellini@creativa77.com.ar (Ernesto Corbellini)
 * File extracted from rosjava_core repository: https://github.com/rosjava/rosjava_core.
 */
public abstract class NativeNodeMain extends AbstractNodeMain {
    public static final int SUCCESS = 0;
    public static final int ROS_CONNECTION_ERROR = 1;
    public static final String ROS_CONNECTION_ERROR_MSG = "ECONNREFUSED";

    private Log log = LogFactory.getLog(NativeNodeMain.class);
    private String libName;
    private String masterUri = null;
    private String hostName = null;
    private String nodeName = null;
    private String[] remappingArguments;
    private boolean shuttingDown = false;

    /**
     *  @param libName
     *    The name of the library to load.
     *
     *  @param remappings
     *    A string array with ROS argument remapping pairs in each element.
     **/
    public NativeNodeMain(String libName, String[] remappings) {
        this.libName = libName;

        // if no remapping is needed, create an empty array
        if (remappings == null) {
            remappingArguments = new String[0];
        }

        log.info("Trying to load native library '" + libName + "'...");
        try
        {
            System.loadLibrary(libName);
        }
        catch (SecurityException e)
        {
            log.info("Error loading library! SecurityException");
        }
        catch (UnsatisfiedLinkError e)
        {
            log.info("Error loading library! UnsatisfiedLinkError");
        }
        catch (NullPointerException e)
        {
            log.info("Error loading library! NullPointerException");
        }
    }

    /**
     *  @param libName
     *    The name of the library to load.
     **/
    public NativeNodeMain(String libName) {
        this(libName, null);
    }

    // An interface to give feedback to the activity on node errors.
    public interface CallbackListener {
        public void onNativeNodeExecutionError(int errorCode);
    }
    CallbackListener callbackListener;

    /**
     *
     * @param activity The activity owning this node.
     */
    public void attachCallbackListener(Activity activity) {
        callbackListener = (CallbackListener) activity;
    }

    // These methods define the execution model interface for this node.
    protected abstract int execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);
    protected abstract void shutdown();

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        // retain important ROS info
        masterUri = connectedNode.getMasterUri().toString();
        hostName = connectedNode.getUri().getHost();
        nodeName = getDefaultNodeName().toString();
        // create a new thread to execute the native code.
        new Thread() {
            @Override
            public void run() {
                int errorCode = execute(masterUri, hostName, nodeName, remappingArguments);
                if(errorCode != SUCCESS) {
                    callbackListener.onNativeNodeExecutionError(errorCode);
                }

                // node execution has finished so we propagate the shutdown sequence only if we aren't already shutting down for other reasons
                if(!shuttingDown) {
                    connectedNode.shutdown();
                }
            }
        }.start();
    }

    @Override
    public void onShutdown(Node node) {
        shuttingDown = true;
        shutdown();
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
        if (throwable.getMessage().contains(ROS_CONNECTION_ERROR_MSG)) {
            callbackListener.onNativeNodeExecutionError(ROS_CONNECTION_ERROR);
        }
    }
}