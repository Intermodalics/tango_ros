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
import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;

import java.util.Map;


public class ParameterNode extends AbstractNodeMain implements NodeMain, SharedPreferences.OnSharedPreferenceChangeListener {

    private static final String NODE_NAME = "parameter_node";
    private final String POSE_PARAM_NAME;
    private final String POINT_CLOUD_PARAM_NAME;
    private final String CAMERA_COLOR_PARAM_NAME;
    private final String CAMERA_FISHEYE_PARAM_NAME;

    private PublisherConfiguration publishConfig = null;
    private ParameterTree parameterTree = null;
    private Activity creator;

    public ParameterNode(Activity activity, String poseParamName, String pointcloudParamName,
                         String camcolorParamName, String camFisheyeParamName) {
        creator = activity;
        POSE_PARAM_NAME = poseParamName;
        POINT_CLOUD_PARAM_NAME = pointcloudParamName;
        CAMERA_COLOR_PARAM_NAME = camcolorParamName;
        CAMERA_FISHEYE_PARAM_NAME = camFisheyeParamName;
    }

    @Override
    public GraphName getDefaultNodeName() { return GraphName.of(NODE_NAME); }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        parameterTree = connectedNode.getParameterTree();

        if (publishConfig != null) {
            uploadPreferencesToParameterServer();
        }
        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(creator);
        pref.registerOnSharedPreferenceChangeListener(this);
    }

    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences,
                                          String key) {
        Log.d(NODE_NAME, "################### UPDATING ROS PARAMETER SERVER WITH PREFERENCES ###################");

        Map<String,?> prefKeys = sharedPreferences.getAll();
        Object prefValue = prefKeys.get(key);

        if (prefValue.getClass().equals(Boolean.class)) {
            uploadSingleBooleanParameter(key,sharedPreferences.getBoolean(key, false));
        }
    }

    public void setPublishConfig(PublisherConfiguration publishConfig) {
        this.publishConfig = publishConfig;
    }

    public void uploadPreferencesToParameterServer(PublisherConfiguration configuration) {
        if (parameterTree == null) {
            return;
        }

        setPublishConfig(configuration);
        uploadPreferencesToParameterServer();
    }

    public PublisherConfiguration fetchPreferencesFromParameterServer() {
        publishConfig.publishDevicePose = fetchSingleBooleanParameter(POSE_PARAM_NAME);
        publishConfig.publishPointCloud = fetchSingleBooleanParameter(POINT_CLOUD_PARAM_NAME);
        if (fetchSingleBooleanParameter(CAMERA_COLOR_PARAM_NAME) == true) {
            publishConfig.publishCamera |= PublisherConfiguration.CAMERA_COLOR;
        } else {
            publishConfig.publishCamera &= ~PublisherConfiguration.CAMERA_COLOR;
        }
        if (fetchSingleBooleanParameter(CAMERA_FISHEYE_PARAM_NAME) == true) {
            publishConfig.publishCamera |= PublisherConfiguration.CAMERA_FISHEYE;
        } else {
            publishConfig.publishCamera &= ~PublisherConfiguration.CAMERA_FISHEYE;
        }

        return publishConfig;
    }

    private void uploadPreferencesToParameterServer() {
        uploadSingleBooleanParameter(POSE_PARAM_NAME, publishConfig.publishDevicePose);
        uploadSingleBooleanParameter(POINT_CLOUD_PARAM_NAME, publishConfig.publishPointCloud);
        uploadSingleBooleanParameter(CAMERA_COLOR_PARAM_NAME, (publishConfig.publishCamera & PublisherConfiguration.CAMERA_COLOR) != 0);
        uploadSingleBooleanParameter(CAMERA_FISHEYE_PARAM_NAME, (publishConfig.publishCamera & PublisherConfiguration.CAMERA_FISHEYE) != 0);
    }

    private void uploadSingleBooleanParameter(String paramName, boolean paramValue) {
        parameterTree.set(buildFullParameterName(paramName), paramValue);
    }

    private boolean fetchSingleBooleanParameter(String paramName) {
        return parameterTree.getBoolean(buildFullParameterName(paramName));
    }

    private String buildFullParameterName(String paramName) {
        return TangoRosNode.NODE_NAME + "/" + paramName;
    }
}
