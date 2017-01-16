package org.ros.node;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

/**
 * A java wrapper to load and run a native-code ROS node.
 *
 * Note: there are no actual native methods declared in this class. We only define an interface. The native methods should be declared in the child class.
 *
 * @author ecorbellini@creativa77.com.ar (Ernesto Corbellini)
 * File extracted from rosjava_core repository: https://github.com/rosjava/rosjava_core.
 *
 * @note Standard code for NativeNodeMain was modified to use error codes for this particular application on Execute function.
 * This code is a candidate to be upstreamed as a contribution to RosJava, providing a generic API that fits this particular application.
 */
public abstract class NativeNodeMainBeta extends AbstractNodeMain {
    private Log log = LogFactory.getLog(NativeNodeMainBeta.class);
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
    public NativeNodeMainBeta(String libName, String[] remappings) {
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
    public NativeNodeMainBeta(String libName) {
        this(libName, null);
    }

    // These methods define the execution model interface for this node.
    protected abstract int execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);
    protected abstract int shutdown();

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
                int returnCode = execute(masterUri, hostName, nodeName, remappingArguments);
                onPostNativeNodeExecution(returnCode);

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
        int returnCode = shutdown();

        onPostNativeNodeShutdown(returnCode);
    }

    public void onPostNativeNodeExecution(int returnCode) {
      // Override this method to retrieve return code
    }

    public void onPostNativeNodeShutdown(int returnCode) {
      // Override this method to retrieve return code
    }
}