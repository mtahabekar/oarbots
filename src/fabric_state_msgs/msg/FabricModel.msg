std_msgs/Header header
string model_id
uint32 rows
uint32 columns
float64 width_m
float64 height_m
float64 areal_density_kg_m2
# Persistent row-major vertices: u increases along columns, v along rows.
geometry_msgs/Point[] rest_positions
geometry_msgs/Point32[] material_uv
uint32[] triangles
uint32[4] corner_indices
