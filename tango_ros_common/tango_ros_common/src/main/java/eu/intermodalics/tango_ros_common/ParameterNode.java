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
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import eu.intermodalics.tango_ros_common.NodeNamespaceHelper;

import org.apache.commons.logging.Log;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;

import java.util.HashMap;

/**
 * RosJava node that handles interactions with the ros parameter server.
 */
public class ParameterNode extends AbstractNodeMain implements NodeMain {
    private static final String NODE_NAME = "parameter_node";

    private Activity mCreatorActivity;
    private SharedPreferences mSharedPreferences;
    private ConnectedNode mConnectedNode;
    private Log mLog;
    private final HashMap<String, String> mParamNames;

    /**
     * Constructor of the ParameterNode class.
     * @param activity The activity running the node.
     * @param paramNames Names of the (non-dynamic) ROS parameters (without namespace).
     */
    public ParameterNode(Activity activity, HashMap<String, String> paramNames) {
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
        setPreferencesFromParameterServer();
    }

    // Set ROS params according to preferences.
    public void uploadPreferencesToParameterServer() {
        for (String paramName : mParamNames.keySet()) {
            if (mParamNames.get(paramName) == "boolean") {
                Boolean booleanValue = mSharedPreferences.getBoolean(paramName, false);
                mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), booleanValue);
            }
            if (mParamNames.get(paramName) == "int_as_string") {
                String stringValue = mSharedPreferences.getString(paramName, "0");
                mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), Integer.parseInt(stringValue));
            }
            if (mParamNames.get(paramName) == "string") {
                String stringValue = mSharedPreferences.getString(paramName, "");
                mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), stringValue);
            }
        }
    }

    // Set app preferences according to ROS params.
    public void setPreferencesFromParameterServer() {
        SharedPreferences.Editor editor = mSharedPreferences.edit();
        for (String paramName : mParamNames.keySet()) {
            if (mParamNames.get(paramName) == "boolean") {
                Boolean booleanValue = mConnectedNode.getParameterTree().getBoolean(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), false);
                editor.putBoolean(paramName, booleanValue);
            }
            if (mParamNames.get(paramName) == "int_as_string") {
                Integer intValue = mConnectedNode.getParameterTree().getInteger(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), 0);
                editor.putString(paramName, intValue.toString());
            }
            if (mParamNames.get(paramName) == "string") {
                String stringValue = mConnectedNode.getParameterTree().getString(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), "");
                editor.putString(paramName, stringValue);
            }
        }
        editor.commit();
    }
}
