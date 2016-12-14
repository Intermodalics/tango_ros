package eu.intermodalics.tangoxros;

import org.ros.namespace.GraphName;

/**
 * Created by juan on 14/12/16.
 */

public class TangoRosNode extends NativeNodeMain {

    private static final String nodeName = "TangoRosNode";

    public TangoRosNode() {
        super("tango_ros_node_native");
    }

    public TangoRosNode(String libName) {
        super(libName);
    }

    @Override
    public native void execute(String rosMasterUri, String rosHostName, String rosNodeName, String[] remappingArguments);

    @Override
    public native void shutdown();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }
}
