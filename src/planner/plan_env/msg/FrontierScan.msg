# Geometry actually acquired at this RGB/depth timestamp, in world coordinates.
std_msgs/Header header
geometry_msgs/Pose camera_pose
float64 resolution
float64 origin_x
float64 origin_y
float64 max_distance
# Original ray-observed free cells (temporary support, no grid semantic map).
int32[] free_x
int32[] free_y
# Current frontier cell lifetime tokens, shared across cluster re-grouping.
uint64[] cell_ids
int32[] cell_x
int32[] cell_y
