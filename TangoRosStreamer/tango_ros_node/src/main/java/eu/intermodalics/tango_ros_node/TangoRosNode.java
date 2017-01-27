/*
 * Copyright 2016 Intermodalics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package eu.intermodalics.tango_ros_node;

import android.content.ServiceConnection;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMainBeta;
import org.ros.node.Node;

import java.util.Arrays;
import java.util.List;

/**
 *  * To run the TangoRosNode correctly:
 * - Load the Tango Shared library using {@link TangoInitializationHelper#loadTangoSharedLibrary}. Return code shall be
 * different from {@link TangoInitializationHelper#ARCH_ERROR}.
 * - Bind to the Tango Service using {@link TangoInitializationHelper#bindTangoService} providing a {@link ServiceConnection}.
 * See {@link TangoInitializationHelper.DefaultTangoServiceConnection} for a default handle.
 * - Create and execute node in the standard RosJava way.
 */
public class TangoRosNode extends NativeNodeMainBeta {
    public static final int ROS_CONNECTION_ERROR = 1;
    public static final String ROS_CONNECTION_FAILURE_ERROR_MSG = "ECONNREFUSED";
    public static final String ROS_CONNECTION_UNREACHABLE_ERROR_MSG = "ENETUNREACH";
    public static final String ROS_CONNECTION_TIMEOUT_ERROR_MSG = "No response after waiting for 10000 milliseconds.";
    public static final String ROS_WRONG_HOST_NAME_ERROR_MSG = "No address associated with hostname";
    public static final String NODE_NAME = "tango";
    public static final String DEFAULT_LIB_NAME = "tango_ros_node";

    private CallbackListener mCallbackListener;
    private List<String> errorMessages;

    public TangoRosNode() {
        super(DEFAULT_LIB_NAME);
        errorMessages = Arrays.asList(
                ROS_CONNECTION_FAILURE_ERROR_MSG,
                ROS_CONNECTION_UNREACHABLE_ERROR_MSG,
                ROS_CONNECTION_TIMEOUT_ERROR_MSG,
                ROS_WRONG_HOST_NAME_ERROR_MSG);
    }

    public TangoRosNode(String libName) {
        super(libName);
    }

    @Override
    public native int execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);

    @Override
    public native int shutdown();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    public interface CallbackListener {
        void onTangoRosErrorHook(int returnCode);
    }

    public void attachCallbackListener(CallbackListener callbackListener) {
        mCallbackListener = callbackListener;
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
        for (String errorMessage : errorMessages) {
            if (throwable.getMessage().contains(errorMessage)) {
                executeOnErrorHook(ROS_CONNECTION_ERROR);
            }
        }
        if (this.executeReturnCode != 0) {
            executeOnErrorHook(this.executeReturnCode);
        }
    }

    private void executeOnErrorHook(int returnCode) {
        if (mCallbackListener != null) {
            mCallbackListener.onTangoRosErrorHook(returnCode);
        }
    }
}
