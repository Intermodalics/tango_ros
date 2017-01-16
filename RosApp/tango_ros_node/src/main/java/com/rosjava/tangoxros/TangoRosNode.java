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

package com.rosjava.tangoxros;

import android.content.ServiceConnection;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMainBeta;

/**
 *  * To run the TangoRosNode correctly:
 * - Load the Tango Shared library using {@link TangoInitializationHelper#loadTangoSharedLibrary}. Return code shall be
 * different from {@link TangoInitializationHelper#ARCH_ERROR}.
 * - Bind to the Tango Service using {@link TangoInitializationHelper#bindTangoService} providing a {@link ServiceConnection}.
 * See {@link TangoInitializationHelper.DefaultTangoServiceConnection} for a default handle.
 * - Create and execute node in the standard RosJava way.
 */
public class TangoRosNode extends NativeNodeMainBeta {
    public static final String NODE_NAME = "tango";
    public static final String DEFAULT_LIB_NAME = "tango_ros_android_lib";

    public TangoRosNode() {
        super(DEFAULT_LIB_NAME);
    }

    public TangoRosNode(String libName) {
        super(libName);
    }

    @Override
    public native int execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);

    @Override
    public native void shutdown();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }
}
