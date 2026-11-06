uint64 edge_id
uint64 start_node_id
uint64 end_node_id
int32[] pixel_x
int32[] pixel_y
geometry_msgs/Point[] path
# Same actual skeleton polyline as path; no artificial centre attachment.
geometry_msgs/Point[] display_path
float64 length
float64 min_clearance
