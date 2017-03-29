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

import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import tango_ros_messages.SaveMap;
import tango_ros_messages.SaveMapRequest;
import tango_ros_messages.SaveMapResponse;

/**
 * Rosjava node that implements a service client.
 */
public class SaveMapServiceClientNode extends AbstractNodeMain {
    private static final String TAG = SaveMapServiceClientNode.class.getSimpleName();
    private static final String NODE_NAME = "save_map_node";
    private static final String SAVEMAP_SRV_NAME = "/tango/save_map";

    ConnectedNode mconnectedNode;
    private Log mLog;
    private boolean mSuccess = false;
    private String mMessage = "";

    public SaveMapServiceClientNode() {
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    public void onStart(ConnectedNode connectedNode) {
        mconnectedNode = connectedNode;
        mLog = connectedNode.getLog();
    }

    public void callService(String mapName) {
        SaveMapRequest saveMapRequest = mconnectedNode.getServiceRequestMessageFactory().newFromType(SaveMap._TYPE);
        saveMapRequest.setMapName(mapName);
        try {
            ServiceClient<SaveMapRequest, SaveMapResponse> saveMapService =
                    mconnectedNode.newServiceClient(SAVEMAP_SRV_NAME, SaveMap._TYPE);
            saveMapService.call(saveMapRequest, new ServiceResponseListener<SaveMapResponse>() {
                @Override
                public void onSuccess(SaveMapResponse saveMapResponse) {
                    mLog.info("Save map service call success");
                    mSuccess = saveMapResponse.getSuccess();
                    mMessage = saveMapResponse.getMessage();
                }
                @Override
                public void onFailure(RemoteException e) {
                    mLog.error("Save map service failure: " + e.getMessage());
                }
            });
        } catch (ServiceNotFoundException e) {
            mLog.error("Service '" + SAVEMAP_SRV_NAME + "' not found!");
        } catch (Exception e) {
            mLog.error("Error while calling Save map Service: " + e.getMessage());
        }
    }

    public boolean getSuccess() {
        return mSuccess;
    }

    public String getMessage() {
        return mMessage;
    }
}