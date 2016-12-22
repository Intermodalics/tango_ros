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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;

import java.util.Map;


public class ParameterNode extends AbstractNodeMain implements NodeMain, SharedPreferences.OnSharedPreferenceChangeListener {

    private static final String NODE_NAME = "parameter_node";
    private final String[] PARAM_NAMES;

    private ParameterTree mParameterTree = null;
    private Activity mCreatorActivity;

    public ParameterNode(Activity activity, String poseParamName, String pointCloudParamName,
                         String camColorParamName, String camFisheyeParamName) {
        mCreatorActivity = activity;
        PARAM_NAMES = new String[] {poseParamName, pointCloudParamName, camColorParamName, camFisheyeParamName};
    }

    @Override
    public GraphName getDefaultNodeName() { return GraphName.of(NODE_NAME); }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        mParameterTree = connectedNode.getParameterTree();
        SharedPreferences sharedPreferences = PreferenceManager.getDefaultSharedPreferences(mCreatorActivity);
        uploadPreferencesToParameterServer(sharedPreferences);

        // Listen to changes in the shared preferences.
        sharedPreferences.registerOnSharedPreferenceChangeListener(this);
        // Add one listener for each parameter on the parameter server.
        for (String s : PARAM_NAMES) {
            addParameterServerListener(s);
        }
    }

    /**
     * Callback that syncs Parameter Server with changes coming from the UI (app --> server).
     * @param sharedPreferences Reference to SharedPreferences containing the preference change.
     * @param key Particular preference that changed.
     */
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        Map<String,?> prefKeys = sharedPreferences.getAll();
        Object prefValue = prefKeys.get(key);

        if (prefValue.getClass().equals(Boolean.class)) {
            uploadSingleBooleanParameter(key,sharedPreferences.getBoolean(key, false));
        }
    }

    /**
     * Syncs the Parameter Server with the all the local shared preferences (app --> server).
     * @param sharedPreferences Reference to the preferences to sync.
     */
    private void uploadPreferencesToParameterServer(SharedPreferences sharedPreferences) {
        Map<String,?> prefKeys = sharedPreferences.getAll();

        for (Map.Entry<String,?> entry : prefKeys.entrySet()) {
            if (entry.getValue().getClass().equals(Boolean.class)) {
                uploadSingleBooleanParameter(entry.getKey(), ((Boolean)entry.getValue()).booleanValue());
            }
        }
    }

    /**
     * Adds listener to update the UI with changes coming from Parameter Server (app <-- UI)
     * @param paramName Parameter to which the listener has to be added.
     */
    private void addParameterServerListener(final String paramName) {
        mParameterTree.addParameterListener(buildFullParameterName(paramName), new ParameterListener() {

            @Override
            public void onNewValue(Object value) {
                SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(mCreatorActivity);
                SharedPreferences.Editor editor = sharedPref.edit();

                if (value.getClass().equals(Boolean.class)) {
                    Boolean bool = (Boolean) value;
                    editor.putBoolean(paramName, bool.booleanValue());
                }
                editor.commit();
            }
        });
    }

    private void uploadSingleBooleanParameter(String paramName, boolean paramValue) {
        mParameterTree.set(buildFullParameterName(paramName), paramValue);
    }

    private String buildFullParameterName(String paramName) {
        return "/" + TangoRosNode.NODE_NAME + "/" + paramName;
    }
}
