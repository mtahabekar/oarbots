uint8 NOT_HOLDING=0
uint8 HOLDING=1
string hand_id
int32 tag_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
# Joint position/velocity covariance; ordering x,y,z,vx,vy,vz.
float64[36] covariance
string[] source_cameras
builtin_interfaces/Time[] source_stamps
uint8 holding_state
bool holding_is_assumed
# Explicit configured corner identity, not an inferred correspondence.
float32[2] grip_uv
# Full constant-acceleration filter state at the header stamp, so a consumer can carry the
# grip to its own time with the same model the filter uses: acceleration, the 9x9 covariance
# of (position, velocity, acceleration) row-major, and the white-jerk density (m/s^3) that
# drives it. filter_covariance is empty when the publisher does not provide it.
geometry_msgs/Vector3 acceleration
float64[] filter_covariance
float64 jerk_stddev
