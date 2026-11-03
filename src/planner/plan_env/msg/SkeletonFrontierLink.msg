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
# Checked, shortened route from CURRENT Junction centre to Frontier centroid.
# Only its final Frontier cell may be UNKNOWN. path/pixels store the FREE raster witness.
geometry_msgs/Point[] display_path
# Full centre-to-centre display_path length, including terminal Frontier segment.
float64 length
string reason
