# Raw RGB pixel coordinates. Holes and occlusion edges are not material boundaries.
bool is_hole
geometry_msgs/Point32[] pixels
# One value per pixel; zero near hands, robots, ROI limits and image edges.
float32[] confidence
