# Reference to an existing FrontierMap2D node, not a duplicate semantic node.
int32 frontier_id
bool active
bool connected
uint64 junction_node_id
geometry_msgs/Point frontier_centroid
# Safe endpoint lies on known FREE/noninflated side of the Frontier boundary.
geometry_msgs/Point approach
int32[] pixel_x
int32[] pixel_y
geometry_msgs/Point[] path
# Checked, shortened route from CURRENT Junction centre to the known-free approach.
# Every point/segment is FREE. path/pixels store its FREE raster witness.
# approach -> frontier_centroid is a separate drawing/association, NOT a safe path.
geometry_msgs/Point[] display_path
# Length of safe display_path only; excludes the drawing/association segment.
float64 length
string reason
