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

import android.os.IBinder;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMain;

public class TangoRosNode extends NativeNodeMain {
    public static final String NODE_NAME = "tango";
    public static final String DEFAULT_LIB_NAME = "tango_ros_android_lib";

    public TangoRosNode() {
        super(DEFAULT_LIB_NAME);
    }

    public TangoRosNode(String libName) {
        super(libName);
    }

    @Override
    // Note: rosNodeName is actually the name of the loaded library.
    public native void execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);

    @Override
    public native void shutdown();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    /**
     * Binds to the tango service.
     *
     * @param nativeTangoServiceBinder The native binder object.
     * @return true if binding to the Tango service ended successfully.
     */
    public native boolean setBinderTangoService(IBinder nativeTangoServiceBinder);

    /**
     * Update the publisher configuration of the tango-ros node.
     *
     * @param publisherConfiguration the new publisher configuration.
     */
    public native void updatePublisherConfiguration(PublisherConfiguration publisherConfiguration);
}
