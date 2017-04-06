// Copyright 2017 Intermodalics All Rights Reserved.
package eu.intermodalics.nodelet_manager;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMain;
import org.ros.node.Node;

import java.util.Arrays;
import java.util.List;

public class NodeletManager extends NativeNodeMain {
    // TODO(mcopejans) Move constants below to separate package.
    public static final int ROS_CONNECTION_ERROR = 1;
    public static final String ROS_CONNECTION_FAILURE_ERROR_MSG = "ECONNREFUSED";
    public static final String ROS_CONNECTION_UNREACHABLE_ERROR_MSG = "ENETUNREACH";
    public static final String ROS_CONNECTION_TIMEOUT_ERROR_MSG = "No response after waiting for 10000 milliseconds.";
    public static final String ROS_WRONG_HOST_NAME_ERROR_MSG = "No address associated with hostname";

    // Node specific.
    public static final String NODE_NAME = "nodelet_manager";
    public static final String DEFAULT_LIB_NAME = "nodelet_manager";

    private List<String> mErrorMessages;
    private CallbackListener mCallbackListener;

    public NodeletManager() {
        super(DEFAULT_LIB_NAME);
        mErrorMessages = Arrays.asList(
                ROS_CONNECTION_FAILURE_ERROR_MSG,
                ROS_CONNECTION_UNREACHABLE_ERROR_MSG,
                ROS_CONNECTION_TIMEOUT_ERROR_MSG,
                ROS_WRONG_HOST_NAME_ERROR_MSG);
    }

    public NodeletManager(String libName) {
        super(libName);
    }

    /**
     * Static helper function which loads shared library to link native functions.
     */
    public static final boolean loadNodeletManagerSharedLibrary() {
        try {
            System.loadLibrary(DEFAULT_LIB_NAME);
        } catch (UnsatisfiedLinkError e) {
            Log.e(NodeletManager.class.getName(),
                    "Error loading shared library: " + DEFAULT_LIB_NAME, e);
            return false;
        }
        return true;
    }

    @Override
    public native int execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);

    @Override
    public native int shutdown();

    public interface CallbackListener {
        void onNodeletManagerError(int returnCode);
    }

    public void attachCallbackListener(CallbackListener callbackListener) {
        mCallbackListener = callbackListener;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
        for (String errorMessage : mErrorMessages) {
            if (throwable.getMessage().contains(errorMessage)) {
                executeOnErrorHook(ROS_CONNECTION_ERROR);
                return;
            }
        }
        if (this.executeReturnCode != 0) {
            executeOnErrorHook(this.executeReturnCode);
        }
    }

    private void executeOnErrorHook(int returnCode) {
        if (mCallbackListener != null) {
            mCallbackListener.onNodeletManagerError(returnCode);
        }
    }
}
