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

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterListener;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.util.Map;

import dynamic_reconfigure.BoolParameter;
import dynamic_reconfigure.Config;
import dynamic_reconfigure.ReconfigureRequest;
import dynamic_reconfigure.ReconfigureResponse;

/**
 * RosJava node that handles interactions with Parameter Server.
 * It provides callbacks to update the app's state with changes on the Parameter Server,
 * and to update the Parameter Server with changes applied on the app's preferences by the user.
 */
public class ParameterNode extends AbstractNodeMain implements NodeMain, SharedPreferences.OnSharedPreferenceChangeListener {

    private static final String NODE_NAME = "parameter_node";
    private static final String RECONFIGURE_SRV_TYPE = "dynamic_reconfigure/Reconfigure";
    private static final String RECONFIGURE_SRV_NAME = "set_parameters";
    private final String[] mParamNames;

    private ParameterTree mParameterTree = null;
    private Activity mCreatorActivity;
    private SharedPreferences mSharedPreferences;
    private ConnectedNode mConnectedNode;

    public ParameterNode(Activity activity, String... paramNames) {
        mCreatorActivity = activity;
        mParamNames = paramNames;
    }

    @Override
    public GraphName getDefaultNodeName() { return GraphName.of(NODE_NAME); }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        mParameterTree = connectedNode.getParameterTree();
        mSharedPreferences = PreferenceManager.getDefaultSharedPreferences(mCreatorActivity);
        // Overwrite preferences in server with local preferences.
        uploadPreferencesToParameterServer(mSharedPreferences);

        // Listen to changes in the shared preferences.
        mSharedPreferences.registerOnSharedPreferenceChangeListener(this);
        // Add one listener for each parameter on the parameter server.
        for (String s : mParamNames) {
            addParameterServerListener(s);
        }

        mConnectedNode = connectedNode;
    }

    /**
     * Callback that syncs Parameter Server with changes coming from the UI (app --> server).
     * @param sharedPreferences Reference to SharedPreferences containing the preference change.
     * @param key Particular preference that changed.
     */
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, final String key) {
        Map<String,?> prefKeys = sharedPreferences.getAll();
        Object prefValue = prefKeys.get(key);

        if (prefValue instanceof Boolean) {
            final Boolean bool = (Boolean) prefValue;
            new Thread() {
                @Override
                public void run() {
                    dynamicReconfigure(key, bool.booleanValue());
                }
            }.start();
        }
    }

    /**
     * Syncs the Parameter Server with all the current local shared preferences (app --> server).
     * @param sharedPreferences Reference to the preferences to sync.
     */
    private void uploadPreferencesToParameterServer(SharedPreferences sharedPreferences) {
        Map<String,?> prefKeys = sharedPreferences.getAll();

        for (Map.Entry<String,?> entry : prefKeys.entrySet()) {
            if (entry.getValue() instanceof Boolean) {
                Boolean bool = (Boolean) entry.getValue();
                uploadSingleBooleanParameter(entry.getKey(), bool.booleanValue());
            }
        }
    }

    /**
     * Adds listener to update the UI with changes coming from Parameter Server (app <-- server)
     * @param paramName Parameter to which the listener has to be added.
     */
    private void addParameterServerListener(final String paramName) {
        mParameterTree.addParameterListener(buildFullParameterName(paramName), new ParameterListener() {

            @Override
            public void onNewValue(Object value) {
                SharedPreferences.Editor editor = mSharedPreferences.edit();

                if (value instanceof Boolean) {
                    Boolean bool = (Boolean) value;
                    editor.putBoolean(paramName, bool.booleanValue());
                }
                editor.commit();
            }
        });
    }

    /**
     * Calls ROS Dynamic Reconfigure service to set a given boolean parameter.
     * This service request triggers bound callbacks for Dynamic Reconfigure on its response, if any.
     * @param paramName Name of the parameter to set.
     * @param paramValue New value for the given parameter name.
     */
    private void dynamicReconfigure(String paramName, boolean paramValue) {
        ReconfigureRequest srv_req = mConnectedNode.getServiceRequestMessageFactory().newFromType(RECONFIGURE_SRV_TYPE);
        Config config = mConnectedNode.getTopicMessageFactory().newFromType(Config._TYPE);
        BoolParameter boolParameter = mConnectedNode.getTopicMessageFactory().newFromType(BoolParameter._TYPE);

        boolParameter.setName(paramName);
        boolParameter.setValue(paramValue);
        config.getBools().add(boolParameter);
        srv_req.setConfig(config);
        try {
            ServiceClient<ReconfigureRequest, ReconfigureResponse> serviceClient =
                    mConnectedNode.newServiceClient(buildFullParameterName(RECONFIGURE_SRV_NAME),
                            RECONFIGURE_SRV_TYPE);

            serviceClient.call(srv_req, new ServiceResponseListener<ReconfigureResponse>() {
                @Override
                public void onSuccess(ReconfigureResponse reconfigureResponse) {
                    Log.i(NODE_NAME, "Dynamic Reconfigure success");
                }

                @Override
                public void onFailure(RemoteException e) {
                    Log.e(NODE_NAME, "Dynamic Reconfigure failure: " + e.getMessage());
                }
            });

        } catch (ServiceNotFoundException e) {
            Log.e(NODE_NAME, "Service not found: " + e.getMessage());
        } catch (Exception e) {
            Log.e(NODE_NAME, "Error while calling Dynamic Reconfigure Service: " + e.getMessage());
        }
    }

    private void uploadSingleBooleanParameter(String paramName, boolean paramValue) {
        mParameterTree.set(buildFullParameterName(paramName), paramValue);
    }

    private String buildFullParameterName(String paramName) {
        return "/" + TangoRosNode.NODE_NAME + "/" + paramName;
    }
}
