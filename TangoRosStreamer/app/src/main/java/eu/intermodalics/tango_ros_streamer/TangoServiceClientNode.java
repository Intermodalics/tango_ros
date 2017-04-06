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

import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;

import java.util.List;
import java.util.concurrent.Callable;

import eu.intermodalics.tango_ros_streamer.common.NodeNamespaceHelper;

import tango_ros_messages.GetMapUuids;
import tango_ros_messages.GetMapUuidsRequest;
import tango_ros_messages.GetMapUuidsResponse;
import tango_ros_messages.SaveMap;
import tango_ros_messages.SaveMapRequest;
import tango_ros_messages.SaveMapResponse;
import tango_ros_messages.TangoConnect;
import tango_ros_messages.TangoConnectRequest;
import tango_ros_messages.TangoConnectResponse;

import std_msgs.Int8;

/**
 * Rosjava node that implements a service client.
 */
public class TangoServiceClientNode extends AbstractNodeMain {
    private static final String TAG = TangoServiceClientNode.class.getSimpleName();
    private static final String NODE_NAME = "tango_service_client_node";
    private static final String SAVE_MAP_SRV_NAME = "/tango/save_map";
    private static final String GET_MAP_UUIDS_SRV_NAME = "/tango/get_map_uuids";
    private static final String TANGO_CONNECT_SRV_NAME = "/tango/connect";
    private static final String TANGO_STATUS_TOPIC_NAME = "status";

    ConnectedNode mConnectedNode;
    private Log mLog;
    CallbackListener mCallbackListener;

    public interface CallbackListener {
        void onSaveMapServiceCallFinish(boolean success, String message);
        void onTangoConnectServiceFinish(int response, String message);
        void onTangoDisconnectServiceFinish(int response, String message);
        void onGetMapUuidsFinish(List<String> mapUuids, List<String> mapNames);
        void onTangoStatus(int status);
    }

    public TangoServiceClientNode(Activity activity) {
        mCallbackListener = (CallbackListener) activity;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    public void onStart(ConnectedNode connectedNode) {
        mConnectedNode = connectedNode;
        mLog = connectedNode.getLog();

        Subscriber<Int8> subscriber = mConnectedNode.newSubscriber(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(TANGO_STATUS_TOPIC_NAME), Int8._TYPE);
        subscriber.addMessageListener(new MessageListener<Int8>() {
            @Override
            public void onNewMessage(Int8 status) {
                mCallbackListener.onTangoStatus(status.getData());
            }
        });
    }

    public boolean wrapCallROSService(String serviceName, Callable<Boolean> callable) {
        if (mConnectedNode == null) {
            // Using system logger since node logger is not ready yet.
            System.out.println("Client node not ready: " + serviceName);
            return false;
        }

        try {
            callable.call();
        } catch (ServiceNotFoundException e) {
            mLog.error("Service '" + serviceName + "' not found!");
            return false;
        } catch (Exception e) {
            mLog.error("Error while calling: " + serviceName + ", message: " + e.getMessage());
            return false;
        }

        return true;
    }

    public Boolean tangoConnect(final byte connectRequest) throws ServiceNotFoundException {
        ServiceClient<TangoConnectRequest, TangoConnectResponse> tangoConnectService =
                mConnectedNode.newServiceClient(TANGO_CONNECT_SRV_NAME, TangoConnect._TYPE);

        TangoConnectRequest tangoConnectRequest = mConnectedNode.getServiceRequestMessageFactory().newFromType(TangoConnect._TYPE);
        tangoConnectRequest.setRequest(connectRequest);
        tangoConnectService.call(tangoConnectRequest, new ServiceResponseListener<TangoConnectResponse>() {
            @Override
            public void onSuccess(TangoConnectResponse tangoConnectResponse) {
                if (connectRequest == TangoConnectRequest.CONNECT) {
                    mCallbackListener.onTangoConnectServiceFinish(tangoConnectResponse.getResponse(), tangoConnectResponse.getMessage());
                } else if (connectRequest == TangoConnectRequest.DISCONNECT) {
                    mCallbackListener.onTangoDisconnectServiceFinish(tangoConnectResponse.getResponse(), tangoConnectResponse.getMessage());
                } else {
                    mLog.error("Request not recognized: " + connectRequest);
                }
            }

            @Override
            public void onFailure(RemoteException e) {
                mCallbackListener.onTangoConnectServiceFinish(TangoConnectResponse.TANGO_ERROR, e.getMessage());
            }
        });

        return true;
    }

    public Boolean saveMap(String mapName) throws ServiceNotFoundException {
        ServiceClient<SaveMapRequest, SaveMapResponse> saveMapService =
                mConnectedNode.newServiceClient(SAVE_MAP_SRV_NAME, SaveMap._TYPE);

        SaveMapRequest saveMapRequest = mConnectedNode.getServiceRequestMessageFactory().newFromType(SaveMap._TYPE);
        saveMapRequest.setMapName(mapName);
        saveMapService.call(saveMapRequest, new ServiceResponseListener<SaveMapResponse>() {
            @Override
            public void onSuccess(SaveMapResponse saveMapResponse) {
                mCallbackListener.onSaveMapServiceCallFinish(saveMapResponse.getSuccess(), saveMapResponse.getMessage());
            }
            @Override
            public void onFailure(RemoteException e) {
                mCallbackListener.onSaveMapServiceCallFinish(false, e.getMessage());
            }
        });

        return true;
    }

    public Boolean getMapUuids() throws ServiceNotFoundException {
        ServiceClient<GetMapUuidsRequest, GetMapUuidsResponse> saveMapService =
                mConnectedNode.newServiceClient(SAVE_MAP_SRV_NAME, SaveMap._TYPE);

        GetMapUuidsRequest request = mConnectedNode.getServiceRequestMessageFactory().newFromType(GetMapUuids._TYPE);
        saveMapService.call(request, new ServiceResponseListener<GetMapUuidsResponse>() {
            @Override
            public void onSuccess(GetMapUuidsResponse saveMapResponse) {
                mCallbackListener.onGetMapUuidsFinish(saveMapResponse.getMapUuids(),
                        saveMapResponse.getMapNames());
            }
            @Override
            public void onFailure(RemoteException e) {
                mCallbackListener.onGetMapUuidsFinish(null, null);
            }
        });

        return true;
    }

    public Boolean callTangoConnectService(final byte connectRequest) {
        final String serviceName = TANGO_CONNECT_SRV_NAME;
        return wrapCallROSService(serviceName, new Callable<Boolean>() {
            public Boolean call() throws ServiceNotFoundException {
                return tangoConnect(connectRequest);
            }
        });
    }

    public Boolean callSaveMapService(final String mapName) {
        final String serviceName = SAVE_MAP_SRV_NAME;
        return wrapCallROSService(serviceName, new Callable<Boolean>() {
            public Boolean call() throws ServiceNotFoundException {
                return saveMap(mapName);
            }
        });
    }

    public Boolean callGetMapUuidsService() {
        final String serviceName = GET_MAP_UUIDS_SRV_NAME;
        return wrapCallROSService(serviceName, new Callable<Boolean>() {
            public Boolean call() throws ServiceNotFoundException {
                return getMapUuids();
            }
        });
    }
}