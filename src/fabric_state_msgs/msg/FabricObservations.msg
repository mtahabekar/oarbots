# header is the RGB capture time and optical frame; never publication time.
std_msgs/Header header
string camera_id
sensor_msgs/CameraInfo camera_info
builtin_interfaces/Time depth_stamp
string segmentation_source
string depth_source
string uv_source
float32 valid_depth_fraction
fabric_state_msgs/Boundary2D[] boundaries
fabric_state_msgs/DepthObservation[] depth
fabric_state_msgs/PointTrack[] tracks
fabric_state_msgs/UVPrior[] uv_priors
