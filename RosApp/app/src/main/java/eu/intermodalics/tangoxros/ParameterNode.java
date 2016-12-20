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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.parameter.ParameterTree;


public class ParameterNode extends AbstractNodeMain implements NodeMain {

    private static final String NODE_NAME = "parameter_node";
    private static final String POSE_PARAM_NAME = "/publish_device_pose";
    private static final String POINT_CLOUD_PARAM_NAME = "/publish_point_cloud";
    private static final String CAMERA_COLOR_PARAM_NAME = "/publish_camera_color";
    private static final String CAMERA_FISHEYE_PARAM_NAME = "/publish_camera_fisheye";

    private PublisherConfiguration publishConfig = null;
    private ParameterTree parameterTree = null;

    @Override
    public GraphName getDefaultNodeName() { return GraphName.of(NODE_NAME); }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        parameterTree = connectedNode.getParameterTree();

        if (publishConfig != null) {
            uploadPreferencesToParameterServer();
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

    private void uploadPreferencesToParameterServer() {
        parameterTree.set(TangoRosNode.NODE_NAME + "/" + POSE_PARAM_NAME, publishConfig.publishDevicePose);
        parameterTree.set(TangoRosNode.NODE_NAME + "/" + POINT_CLOUD_PARAM_NAME, publishConfig.publishPointCloud);
        parameterTree.set(TangoRosNode.NODE_NAME + "/" + CAMERA_COLOR_PARAM_NAME, publishConfig.publishDevicePose);
        parameterTree.set(TangoRosNode.NODE_NAME + "/" + CAMERA_FISHEYE_PARAM_NAME, publishConfig.publishPointCloud);
    }
}
