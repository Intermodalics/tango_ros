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

package eu.intermodalics.tango_ros_common;

import android.app.Activity;
import android.os.SystemClock;

import org.apache.commons.logging.Log;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Rosjava node that implements a service server.
 */
public class TangoServiceServerNode extends AbstractNodeMain {
    private static final String TAG = TangoServiceServerNode.class.getSimpleName();
    private static final String NODE_NAME = "tango_service_server_node";
    private static final String REQUEST_PERMISSION_SRV_NAME = "request_permission";

    ConnectedNode mConnectedNode;
    private Log mLog;
    CallbackListener mCallbackListener;

    AtomicBoolean mRequestPermissionAnswered = new AtomicBoolean(false);
    AtomicBoolean mRequestPermissionGranted = new AtomicBoolean(false);

    public interface CallbackListener {
        void onRequestPermissionServiceCalled(tango_ros_messages.RequestPermissionRequest request,
                                              tango_ros_messages.RequestPermissionResponse response);
    }

    public TangoServiceServerNode(Activity activity) {
        mCallbackListener = (CallbackListener) activity;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    public void onStart(ConnectedNode connectedNode) {
        mConnectedNode = connectedNode;
        mLog = connectedNode.getLog();

        ServiceServer<tango_ros_messages.RequestPermissionRequest, tango_ros_messages.RequestPermissionResponse> server =
                mConnectedNode.newServiceServer(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(REQUEST_PERMISSION_SRV_NAME),
                        tango_ros_messages.RequestPermission._TYPE,
                        new ServiceResponseBuilder<tango_ros_messages.RequestPermissionRequest, tango_ros_messages.RequestPermissionResponse>() {
                    @Override
                    public void build(tango_ros_messages.RequestPermissionRequest request, tango_ros_messages.RequestPermissionResponse response) {
                        mRequestPermissionAnswered.set(false);
                        mRequestPermissionGranted.set(false);
                        mCallbackListener.onRequestPermissionServiceCalled(request, response);
                        while (!mRequestPermissionAnswered.get()) {
                            mLog.info("Waiting for user to answer permission request.");
                            SystemClock.sleep(100);
                        }
                        response.setGranted(mRequestPermissionGranted.get());
                    }
                });
    }

    public void onRequestPermissionAnswered(boolean granted) {
        mRequestPermissionGranted.set(granted);
        mRequestPermissionAnswered.set(true);
    }
}