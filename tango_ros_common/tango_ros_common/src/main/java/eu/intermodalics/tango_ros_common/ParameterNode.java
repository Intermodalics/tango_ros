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
import android.os.Build;
import android.preference.PreferenceManager;

import org.apache.commons.logging.Log;
import org.ros.exception.ParameterClassCastException;
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
        setIntParam("android_api_level", Build.VERSION.SDK_INT);
        syncLocalPreferencesWithParameterServer();
    }

    /**
     * First download ROS parameters from parameter server, then upload local settings to parameter
     * server.
     */
    public void syncLocalPreferencesWithParameterServer() {
        setPreferencesFromParameterServer();
        uploadPreferencesToParameterServer();
    }

    // Set ROS params according to preferences.
    public void uploadPreferencesToParameterServer() {
        mLog.info("Upload preferences to parameter server.");
        for (String paramName : mParamNames.keySet()) {
            uploadPreferenceToParameterServer(paramName);
        }
    }

    public void uploadPreferenceToParameterServer(String paramName) {
        if (mSharedPreferences == null) {
            // Cannot use mLog here because it is null.
            System.out.println("Shared preferences are null, failed to edit.");
            return;
        }
        String valueType = mParamNames.get(paramName);
        if (valueType == "boolean") {
            Boolean booleanValue = mSharedPreferences.getBoolean(paramName, false);
            mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), booleanValue);
        } else if (valueType == "int_as_string") {
            String stringValue = mSharedPreferences.getString(paramName, "0");
            mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), Integer.parseInt(stringValue));
        } else if (valueType == "string") {
            String stringValue = mSharedPreferences.getString(paramName, "");
            mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), stringValue);
        }
    }

    // Set app preferences according to ROS params.
    public void setPreferencesFromParameterServer() {
        if (mSharedPreferences == null) {
            // Cannot use mLog here because it is null.
            System.out.println("Shared preferences are null, failed to edit.");
            return;
        }
        SharedPreferences.Editor editor = mSharedPreferences.edit();
        for (String paramName : mParamNames.keySet()) {
            if (mConnectedNode.getParameterTree().has(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName))) {
                try {
                    if (mParamNames.get(paramName) == "boolean") {
                        Boolean booleanValue = getBoolParam(paramName);
                        editor.putBoolean(paramName, booleanValue);
                    }
                    if (mParamNames.get(paramName) == "int_as_string") {
                        Integer intValue = getIntParam(paramName);
                        editor.putString(paramName, intValue.toString());
                    }
                    if (mParamNames.get(paramName) == "string") {
                        String stringValue = getStringParam(paramName);
                        editor.putString(paramName, stringValue);
                    }
                } catch (ParameterClassCastException e) {
                    if (mParamNames.get(paramName) == "boolean") {
                        try {
                            Integer intValue = getIntParam(paramName);
                            editor.putBoolean(paramName, !intValue.equals(0));
                        } catch (ParameterClassCastException e2) {
                            mLog.error("Preference " + paramName + " can not be set from parameter server.", e2);
                        }
                    } else {
                        mLog.error("Preference " + paramName + " can not be set from parameter server.", e);
                    }
                }
            }
        }
        editor.commit();
    }

    public void changeSettingsToLocalizeInMap(final String mapUuid, final String prefCreateNewMapKey,
                                              final String prefLocalizationModeKey, final String prefLocalizationMapUuidKey) {
        SharedPreferences.Editor editor = mSharedPreferences.edit();
        editor.putBoolean(prefCreateNewMapKey, false);
        editor.putString(prefLocalizationModeKey, "3");
        editor.putString(prefLocalizationMapUuidKey, mapUuid);
        editor.commit();
        uploadPreferencesToParameterServer();
    }

    public Boolean getBoolParam(String paramName) {
        return mConnectedNode.getParameterTree().getBoolean(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), false);
    }

    public Integer getIntParam(String paramName) {
        return mConnectedNode.getParameterTree().getInteger(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), 0);
    }

    public String getStringParam(String paramName) {
        return mConnectedNode.getParameterTree().getString(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), "");
    }

    public void setBoolParam(String paramName, Boolean value) {
         mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), value);
    }

    public void setIntParam(String paramName, Integer value) {
        mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), value);
    }

    public void setStringParam(String paramName, String value) {
        mConnectedNode.getParameterTree().set(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(paramName), value);
    }
}
