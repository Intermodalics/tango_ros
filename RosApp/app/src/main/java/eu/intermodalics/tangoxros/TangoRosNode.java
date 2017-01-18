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

package eu.intermodalics.tangoxros;

import android.app.Activity;
import android.os.IBinder;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMainBeta;
import org.ros.node.Node;

public class TangoRosNode extends NativeNodeMainBeta {
    public static final int ROS_CONNECTION_ERROR = 1;
    public static final String ROS_CONNECTION_FAILURE_ERROR_MSG = "ECONNREFUSED";
    public static final String ROS_WRONG_HOST_NAME_ERROR_MSG = "No address associated with hostname";
    public static final String NODE_NAME = "tango";
    public static final String DEFAULT_LIB_NAME = "tango_ros_android_lib";
    private CallbackListener mCallbackListener;

    public TangoRosNode() {
        super(DEFAULT_LIB_NAME);
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

    /**
     * Binds to the tango service.
     *
     * @param nativeTangoServiceBinder The native binder object.
     * @return true if binding to the Tango service ended successfully.
     */
    public native boolean setBinderTangoService(IBinder nativeTangoServiceBinder);

    /**
     * Check that the tango version is correct.
     *
     * @param callerActivity the caller activity of this function.
     * @return true if the version of tango is ok.
     */
    public native boolean isTangoVersionOk(Activity callerActivity);

    public interface CallbackListener {
        void onTangoRosErrorHook(int returnCode);
    }

    public void attachCallbackListener(CallbackListener callbackListener) {
        mCallbackListener = callbackListener;
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        super.onError(node, throwable);
        if (throwable.getMessage().contains(ROS_CONNECTION_FAILURE_ERROR_MSG) ||
                throwable.getMessage().contains(ROS_WRONG_HOST_NAME_ERROR_MSG)) {
            executeOnErrorHook(ROS_CONNECTION_ERROR);
        } else if (this.executeReturnCode != 0) {
            executeOnErrorHook(this.executeReturnCode);
        }
    }

    private void executeOnErrorHook(int returnCode) {
        if (mCallbackListener != null) {
            mCallbackListener.onTangoRosErrorHook(returnCode);
        }
    }
}
