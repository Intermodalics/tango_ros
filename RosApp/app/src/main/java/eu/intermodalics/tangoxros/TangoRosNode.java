package eu.intermodalics.tangoxros;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMain;

/**
 * Created by juan on 14/12/16.
 */

public class TangoRosNode extends NativeNodeMain {

    public static final String nodeName = "tango_x_ros";

    public TangoRosNode() {
        super("tango_ros_android_lib");
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
