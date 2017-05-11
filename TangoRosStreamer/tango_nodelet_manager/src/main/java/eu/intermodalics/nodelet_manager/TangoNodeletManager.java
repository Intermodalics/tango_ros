// Copyright 2017 Intermodalics All Rights Reserved.
package eu.intermodalics.nodelet_manager;

import android.util.Log;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMain;
import org.ros.node.Node;

import java.util.Arrays;
import java.util.List;

public class TangoNodeletManager extends NativeNodeMain {
    // Node specific.
    public static final String NODE_NAME = "tango";
    public static final String DEFAULT_LIB_NAME = "nodelet_manager";

    public TangoNodeletManager() {
        super(DEFAULT_LIB_NAME);
    }

    public TangoNodeletManager(String libName) {
        super(libName);
    }

    /**
     * Static helper function which loads shared library to link native functions.
     */
    public static final boolean loadNodeletManagerSharedLibrary() {
        try {
            System.loadLibrary(DEFAULT_LIB_NAME);
        } catch (UnsatisfiedLinkError e) {
            Log.e(TangoNodeletManager.class.getName(),
                    "Error loading shared library: " + DEFAULT_LIB_NAME, e);
            return false;
        }
        return true;
    }

    @Override
    public native int execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);

    @Override
    public native int shutdown();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }
}
