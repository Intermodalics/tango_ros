package eu.intermodalics.tangoxros;

public class PublisherConfiguration {
    public class CameraFlag {
        public static final int NONE = 0;
        public static final int FISHEYE = 1 << 1;
        public static final int COLOR = 1 << 2;
    }

    public boolean publishDevicePose = false;
    public boolean publishPointCloud = false;
    public int publishCamera = CameraFlag.NONE;
}