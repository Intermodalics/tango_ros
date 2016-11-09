package eu.intermodalics.tangoxros;

public class PublisherConfiguration {
    public enum CameraType {
        None,
        Fisheye,
        Color
    };

    public boolean publishDevicePose = false;
    public boolean publishPointCloud = false;
    public CameraType publishCamera = CameraType.None;
}