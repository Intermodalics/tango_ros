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

package eu.intermodalics.tango_ros_streamer;

import android.app.Activity;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import java.util.Arrays;
import java.util.Map;

import dynamic_reconfigure.BoolParameter;
import dynamic_reconfigure.Config;
import dynamic_reconfigure.Reconfigure;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;
import eu.intermodalics.tango_ros_node.TangoRosNode;

/**
 * RosJava node that handles interactions with the ros parameter server.
 */
public class ParameterNode extends AbstractNodeMain implements NodeMain {
    private static final String NODE_NAME = "parameter_node";

    private Activity mCreatorActivity;
    private SharedPreferences mSharedPreferences;
    private ConnectedNode mConnectedNode;
    private Log mLog;
    private final String[] mParamNames;

    /**
     * Constructor of the ParameterNode class.
     * @param activity The activity running the node.
     * @param paramNames Names of the (non-dynamic) ROS parameters (without namespace).
     */
    public ParameterNode(Activity activity, String[] paramNames) {
        mCreatorActivity = activity;
        mParamNames = paramNames;
    }

    @Override
    public GraphName getDefaultNodeName() { return GraphName.of(NODE_NAME); }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        mSharedPreferences = PreferenceManager.getDefaultSharedPreferences(mCreatorActivity);
        mConnectedNode = connectedNode;
        mLog = connectedNode.getLog();
        // Set ROS params according to preferences.
        for (String paramName : mParamNames) {
            String stringValue = mSharedPreferences.getString(paramName, "1");
            connectedNode.getParameterTree().set(BuildTangoRosNodeNamespaceName(paramName), Integer.parseInt(stringValue));
        }
    }

    private String BuildTangoRosNodeNamespaceName(String paramName) {
        return "/" + TangoRosNode.NODE_NAME + "/" + paramName;
    }
}
