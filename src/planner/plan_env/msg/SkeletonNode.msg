uint8 ENDPOINT=0
uint8 JUNCTION=1
uint8 LOOP_ANCHOR=2
uint64 node_id
uint8 node_type
int32 pixel_x
int32 pixel_y
geometry_msgs/Point position
# Wire compatibility: contains the one actual representative skeleton pixel.
int32[] anchor_pixel_x
int32[] anchor_pixel_y
uint64[] neighbor_node_ids
uint32 neighbor_count
float64 clearance
bool near_unknown_boundary
bool active
