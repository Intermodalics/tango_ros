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
 * RosJava node that handles interactions with Dynamic Reconfigure.
 * It provides callbacks to update the app's state with changes on Dynamic Reconfigure,
 * and to update Dynamic Reconfigure with changes applied on the app's preferences by the user.
 */
public class ParameterNode extends AbstractNodeMain implements NodeMain, SharedPreferences.OnSharedPreferenceChangeListener {
    private static final String NODE_NAME = "parameter_node";
    private static final String RECONFIGURE_TOPIC_NAME = "parameter_updates";
    private static final String RECONFIGURE_SRV_NAME = "set_parameters";

    private Activity mCreatorActivity;
    private SharedPreferences mSharedPreferences;
    private ConnectedNode mConnectedNode;
    private Log mLog;
    private final String[] mDynamicParamNames;
    private final String[] mParamNames;
    private boolean mParameterNodeCalledDynamicReconfigure = false;

    /**
     * Constructor of the ParameterNode class.
     * @param activity The activity running the node.
     * @param dynamicParamNames Names of the dynamic reconfigure parameters (without namespace).
     * @param paramNames Names of the (non-dynamic) ROS parameters (without namespace).
     */
    public ParameterNode(Activity activity, String[] dynamicParamNames, String[] paramNames) {
        mCreatorActivity = activity;
        mDynamicParamNames = dynamicParamNames;
        mParamNames = paramNames;
    }

    @Override
    public GraphName getDefaultNodeName() { return GraphName.of(NODE_NAME); }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        mConnectedNode = connectedNode;
        mLog = connectedNode.getLog();
        mSharedPreferences = PreferenceManager.getDefaultSharedPreferences(mCreatorActivity);

        // Overwrite preferences in server with local preferences.
        uploadPreferencesToDynamicReconfigure(mSharedPreferences);

        // Listen to changes in the shared preferences.
        mSharedPreferences.registerOnSharedPreferenceChangeListener(this);

        // Listen to updates of the dynamic reconfigure server via the update parameters topic.
        Subscriber<Config> subscriber = mConnectedNode.newSubscriber(BuildTangoRosNodeNamespaceName(RECONFIGURE_TOPIC_NAME), Config._TYPE);
        subscriber.addMessageListener(new MessageListener<Config>() {
            @Override
            public void onNewMessage(Config config) {
                if (!mParameterNodeCalledDynamicReconfigure) {
                    SharedPreferences.Editor editor = mSharedPreferences.edit();
                    for (BoolParameter boolParam : config.getBools()) {
                        if (Arrays.asList(mDynamicParamNames).contains(boolParam.getName())) {
                            editor.putBoolean(boolParam.getName(), boolParam.getValue());
                        }
                    }
                    editor.commit();
                }
            }
        });

        // Set ROS params according to preferences.
        for (String paramName : mParamNames) {
            if (paramName.equals(mCreatorActivity.getString(R.string.pref_create_new_map_key))) {
                Boolean booleanValue = mSharedPreferences.getBoolean(paramName, true);
                connectedNode.getParameterTree().set(BuildTangoRosNodeNamespaceName(paramName), booleanValue);
            }
            if (paramName.equals(mCreatorActivity.getString(R.string.pref_localization_mode_key))) {
                String stringValue = mSharedPreferences.getString(paramName, "1");
                connectedNode.getParameterTree().set(BuildTangoRosNodeNamespaceName(paramName), Integer.parseInt(stringValue));
            }
            if (paramName.equals(mCreatorActivity.getString(R.string.pref_localization_map_uuid_key))) {
                String stringValue = mSharedPreferences.getString(paramName, "");
                connectedNode.getParameterTree().set(BuildTangoRosNodeNamespaceName(paramName), stringValue);
            }
        }
    }

    /**
     * Callback that syncs Parameter Server with changes coming from the UI (app --> server).
     * @param sharedPreferences Reference to SharedPreferences containing the preference change.
     * @param key Particular preference that changed.
     */
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, final String key) {
        Map<String,?> prefKeys = sharedPreferences.getAll();
        Object prefValue = prefKeys.get(key);
        if (Arrays.asList(mDynamicParamNames).contains(key)) {
            if (prefValue instanceof Boolean) {
                final Boolean bool = (Boolean) prefValue;
                new Thread() {
                    @Override
                    public void run() {
                        callDynamicReconfigure(key, bool.booleanValue());
                    }
                }.start();
            }
        }
    }

    /**
     * Syncs the Parameter Server with all the current local shared preferences (app --> server).
     * @param sharedPreferences Reference to the preferences to sync.
     */
    private void uploadPreferencesToDynamicReconfigure(SharedPreferences sharedPreferences) {
        Map<String,?> prefKeys = sharedPreferences.getAll();

        for (Map.Entry<String,?> entry : prefKeys.entrySet()) {
            if (Arrays.asList(mDynamicParamNames).contains(entry.getKey())) {
                if (entry.getValue() instanceof Boolean) {
                    Boolean bool = (Boolean) entry.getValue();
                    callDynamicReconfigure(entry.getKey(), bool.booleanValue());
                }
            }
        }
    }

    /**
     * Calls ROS Dynamic Reconfigure service to set a given boolean parameter.
     * This service request triggers bound callbacks for Dynamic Reconfigure on its response, if any.
     * @param paramName Name of the parameter to set.
     * @param paramValue New value for the given parameter name.
     */
    private void callDynamicReconfigure(String paramName, boolean paramValue) {
        ReconfigureRequest srv_req = mConnectedNode.getServiceRequestMessageFactory().newFromType(Reconfigure._TYPE);
        Config config = mConnectedNode.getTopicMessageFactory().newFromType(Config._TYPE);
        BoolParameter boolParameter = mConnectedNode.getTopicMessageFactory().newFromType(BoolParameter._TYPE);

        boolParameter.setName(paramName);
        boolParameter.setValue(paramValue);
        config.getBools().add(boolParameter);
        srv_req.setConfig(config);
        try {
            ServiceClient<ReconfigureRequest, ReconfigureResponse> serviceClient =
                    mConnectedNode.newServiceClient(BuildTangoRosNodeNamespaceName(RECONFIGURE_SRV_NAME),
                            Reconfigure._TYPE);

            serviceClient.call(srv_req, new ServiceResponseListener<ReconfigureResponse>() {
                @Override
                public void onSuccess(ReconfigureResponse reconfigureResponse) {
                    mLog.info("Dynamic Reconfigure success");
                    mParameterNodeCalledDynamicReconfigure = false;
                }

                @Override
                public void onFailure(RemoteException e) {
                    mLog.error("Dynamic Reconfigure failure: " + e.getMessage());
                }
            });
            mParameterNodeCalledDynamicReconfigure = true;
        } catch (ServiceNotFoundException e) {
            mLog.error("Service not found: " + e.getMessage());
        } catch (Exception e) {
            mLog.error("Error while calling Dynamic Reconfigure Service: " + e.getMessage());
        }
    }

    private String BuildTangoRosNodeNamespaceName(String paramName) {
        return "/" + TangoRosNode.NODE_NAME + "/" + paramName;
    }
}
