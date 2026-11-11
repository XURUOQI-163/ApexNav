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
# Visit evidence applies only to JUNCTION. ENDPOINT / LOOP_ANCHOR use NOT_APPLICABLE.
uint8 CANDIDATE=0
uint8 CONFIRMED=1
uint8 NOT_APPLICABLE=255
uint8 visit_state
float64 visit_radius
int64 confirmed_at_ns
geometry_msgs/Point visit_position
