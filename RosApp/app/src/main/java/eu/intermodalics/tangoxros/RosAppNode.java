/*
 * Copyright 2017 Intermodalics All Rights Reserved.
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

package eu.intermodalics.tangoxros;

import org.ros.node.Node;

/**
 * A wrapper for {@link TangoRosNode} extending app-specific functionality only.
 */
public class RosAppNode extends TangoRosNode {
    public static final int SUCCESS = 0;
    public static final int ROS_CONNECTION_ERROR = 1;
    public static final String ROS_CONNECTION_FAILURE_ERROR_MSG = "ECONNREFUSED";
    public static final String ROS_WRONG_HOST_NAME_ERROR_MSG = "No address associated with hostname";

    private CallbackListener callbackListener;

    @Override
    public void onPostNativeNodeExecution(int returnCode) {
        if (callbackListener != null) {
            callbackListener.onPostNativeNodeExecution(returnCode);
        }
    }

    public interface CallbackListener {
        void onPostNativeNodeExecution(int returnCode);
    }

    public void attachCallbackListener(CallbackListener callbackListener) {
        this.callbackListener = callbackListener;
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
        if (throwable.getMessage().contains(ROS_CONNECTION_FAILURE_ERROR_MSG) ||
                throwable.getMessage().contains(ROS_WRONG_HOST_NAME_ERROR_MSG)) {
            onPostNativeNodeExecution(ROS_CONNECTION_ERROR);
        }
    }
}
