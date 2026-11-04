# Identity is local to camera_id and process lifetime, NOT material UV.
uint64 id
builtin_interfaces/Time first_stamp
builtin_interfaces/Time previous_stamp
float32 u
float32 v
float32 previous_u
float32 previous_v
float32 forward_backward_error
bool has_depth
geometry_msgs/Point32 position
float32 axial_variance
