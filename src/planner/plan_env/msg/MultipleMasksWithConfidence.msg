# Original RGB acquisition stamp, shared with SemanticScore and depth/pose.
std_msgs/Header header


# Target instance point clouds in world coordinates
sensor_msgs/PointCloud2[] point_clouds

# Corresponding confidence scores
float32[] confidence_scores
