uint8 INFERRED=0
uint8 OBSERVED=1
uint8 STALE=2
# State time may be predicted; measurement_stamp never advances on prediction.
std_msgs/Header header
builtin_interfaces/Time measurement_stamp
string model_id
string physics_backend
bool predicted
bool degraded
geometry_msgs/Point[] positions
geometry_msgs/Vector3[] velocities
# Flattened row-major 6x6 covariance per vertex: x,y,z,vx,vy,vz.
float64[] covariance
uint8[] visibility
float64[] observation_age_sec
# Uniform total external force on the sheet (N); estimator is its sole owner.
geometry_msgs/Vector3 residual_force
float64[9] residual_force_covariance
