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
 * Rosjava node that implements a client for tango ROS services.
 */
public class TangoServiceClientNode extends AbstractNodeMain {
    private static final String TAG = TangoServiceClientNode.class.getSimpleName();
    private static final String NODE_NAME = "tango_service_client_node";
    private static final String SAVE_MAP_SRV_NAME = "save_map";
    private static final String GET_MAP_UUIDS_SRV_NAME = "get_map_uuids";
    private static final String TANGO_CONNECT_SRV_NAME = "connect";
    private static final String TANGO_STATUS_TOPIC_NAME = "status";

    ConnectedNode mConnectedNode;
    private Log mLog;
    CallbackListener mCallbackListener = null;

    public interface CallbackListener {
        void onSaveMapServiceCallFinish(boolean success, String message, String mapName, String mapUuid);
        void onTangoConnectServiceFinish(int response, String message);
        void onTangoDisconnectServiceFinish(int response, String message);
        void onTangoReconnectServiceFinish(int response, String message);
        void onGetMapUuidsFinish(List<String> mapUuids, List<String> mapNames);
        void onTangoStatus(int status);
    }

    public TangoServiceClientNode() {}

    public void seCallbackListener (CallbackListener callbackListener) {
        mCallbackListener = callbackListener;
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
                if (mCallbackListener == null) {
                    mLog.warn("No callback listener was set.");
                    return;
                }
                 mCallbackListener.onTangoStatus(status.getData());
            }
        });
    }

    /**
     * Wrapper function for ROS service call to avoid code duplication in try catch block.
     * @param serviceName Name of the ROS service to be called. Used for error logging.
     * @param callable Callable object on which function 'call' is defined.
     * @return true if calling the service was successful.
     */
    private boolean wrapCallROSService(String serviceName, Callable<Boolean> callable) {
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

    private Boolean tangoConnect(final byte connectRequest) throws ServiceNotFoundException {
        ServiceClient<TangoConnectRequest, TangoConnectResponse> tangoConnectService =
                mConnectedNode.newServiceClient(NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(TANGO_CONNECT_SRV_NAME), TangoConnect._TYPE);

        TangoConnectRequest tangoConnectRequest = mConnectedNode.getServiceRequestMessageFactory().newFromType(TangoConnect._TYPE);
        tangoConnectRequest.setRequest(connectRequest);
        tangoConnectService.call(tangoConnectRequest, new ServiceResponseListener<TangoConnectResponse>() {
            @Override
            public void onSuccess(TangoConnectResponse tangoConnectResponse) {
                if (mCallbackListener == null) {
                    mLog.warn("No callback listener was set.");
                    return;
                }
                if (connectRequest == TangoConnectRequest.CONNECT) {
                    mCallbackListener.onTangoConnectServiceFinish(tangoConnectResponse.getResponse(), tangoConnectResponse.getMessage());
                } else if (connectRequest == TangoConnectRequest.DISCONNECT) {
                    mCallbackListener.onTangoDisconnectServiceFinish(tangoConnectResponse.getResponse(), tangoConnectResponse.getMessage());
                } else if (connectRequest == TangoConnectRequest.RECONNECT) {
                    mCallbackListener.onTangoReconnectServiceFinish(tangoConnectResponse.getResponse(), tangoConnectResponse.getMessage());
                } else {
                    mLog.error("Request not recognized: " + connectRequest);
                }
            }

            @Override
            public void onFailure(RemoteException e) {
                if (mCallbackListener == null) {
                    mLog.warn("No callback listener was set.");
                    return;
                }
                if (connectRequest == TangoConnectRequest.CONNECT) {
                    mCallbackListener.onTangoConnectServiceFinish(TangoConnectResponse.TANGO_ERROR, e.getMessage());
                } else if (connectRequest == TangoConnectRequest.DISCONNECT) {
                    mCallbackListener.onTangoDisconnectServiceFinish(TangoConnectResponse.TANGO_ERROR, e.getMessage());
                } else if (connectRequest == TangoConnectRequest.RECONNECT) {
                    mCallbackListener.onTangoReconnectServiceFinish(TangoConnectResponse.TANGO_ERROR, e.getMessage());
                } else {
                    mLog.error("Request not recognized: " + connectRequest);
                }
            }
        });

        return true;
    }

    private Boolean saveMap(String mapName) throws ServiceNotFoundException {
        ServiceClient<SaveMapRequest, SaveMapResponse> saveMapService =
                mConnectedNode.newServiceClient(
                        NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(SAVE_MAP_SRV_NAME),
                        SaveMap._TYPE);

        SaveMapRequest saveMapRequest = mConnectedNode.getServiceRequestMessageFactory().newFromType(SaveMap._TYPE);
        saveMapRequest.setMapName(mapName);
        saveMapService.call(saveMapRequest, new ServiceResponseListener<SaveMapResponse>() {
            @Override
            public void onSuccess(SaveMapResponse saveMapResponse) {
                if (mCallbackListener == null) {
                    mLog.warn("No callback listener was set.");
                    return;
                }
                mCallbackListener.onSaveMapServiceCallFinish(saveMapResponse.getSuccess(),
                        saveMapResponse.getMessage(), saveMapResponse.getMapName(), saveMapResponse.getMapUuid());
            }
            @Override
            public void onFailure(RemoteException e) {
                if (mCallbackListener == null) {
                    mLog.warn("No callback listener was set.");
                    return;
                }
                mCallbackListener.onSaveMapServiceCallFinish(false, e.getMessage(), null, null);
            }
        });

        return true;
    }

    private Boolean getMapUuids() throws ServiceNotFoundException {
        ServiceClient<GetMapUuidsRequest, GetMapUuidsResponse> getMapUuidsService =
                mConnectedNode.newServiceClient(
                        NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(GET_MAP_UUIDS_SRV_NAME),
                        GetMapUuids._TYPE);

        GetMapUuidsRequest request = mConnectedNode.getServiceRequestMessageFactory().newFromType(GetMapUuids._TYPE);
        getMapUuidsService.call(request, new ServiceResponseListener<GetMapUuidsResponse>() {
            @Override
            public void onSuccess(GetMapUuidsResponse getMapUuidsResponse) {
                if (mCallbackListener == null) {
                    mLog.warn("No callback listener was set.");
                    return;
                }
                mCallbackListener.onGetMapUuidsFinish(getMapUuidsResponse.getMapUuids(),
                        getMapUuidsResponse.getMapNames());
            }
            @Override
            public void onFailure(RemoteException e) {
                if (mCallbackListener == null) {
                    mLog.warn("No callback listener was set.");
                    return;
                }
                mCallbackListener.onGetMapUuidsFinish(null, null);
            }
        });

        return true;
    }

    public Boolean callTangoConnectService(final byte connectRequest) {
        final String serviceName = NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(TANGO_CONNECT_SRV_NAME);
        return wrapCallROSService(serviceName, new Callable<Boolean>() {
            public Boolean call() throws ServiceNotFoundException {
                return tangoConnect(connectRequest);
            }
        });
    }

    public Boolean callSaveMapService(final String mapName) {
        final String serviceName = NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(SAVE_MAP_SRV_NAME);
        return wrapCallROSService(serviceName, new Callable<Boolean>() {
            public Boolean call() throws ServiceNotFoundException {
                return saveMap(mapName);
            }
        });
    }

    public Boolean callGetMapUuidsService() {
        final String serviceName = NodeNamespaceHelper.BuildTangoRosNodeNamespaceName(GET_MAP_UUIDS_SRV_NAME);
        return wrapCallROSService(serviceName, new Callable<Boolean>() {
            public Boolean call() throws ServiceNotFoundException {
                return getMapUuids();
            }
        });
    }
}