# Original inference image, not the latest/annotated camera frame.
std_msgs/Header header
sensor_msgs/Image image
geometry_msgs/Pose camera_pose
geometry_msgs/Pose robot_pose
float64 camera_yaw
string goal_text
string model_name
string model_version
float64 raw_similarity
int64 computed_at_ns
uint64[] cell_ids
float64[] weights
