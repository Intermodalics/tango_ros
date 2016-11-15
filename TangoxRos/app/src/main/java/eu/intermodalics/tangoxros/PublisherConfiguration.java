package eu.intermodalics.tangoxros;

public class PublisherConfiguration {
    public static final int CAMERA_NONE = 0;
    public static final int CAMERA_FISHEYE = 1 << 1;
    public static final int CAMERA_COLOR = 1 << 2;

    public boolean publishDevicePose = false;
    public boolean publishPointCloud = false;
    public int publishCamera = CAMERA_NONE;
}