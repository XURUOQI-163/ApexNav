#pragma once
#include <plan_env/topology_graph.h>
#include <plan_env/sdf_map2d.h>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/core.hpp>
#include <deque>

namespace apexnav_planner {
// Optional in-process observer. It owns no motion, goal or stop publisher.
class SparseTopologyObserver {
public:
SparseTopologyObserver(rclcpp::Node::SharedPtr node, SDFMap2D *map);
void begin(const sensor_msgs::msg::Image::ConstSharedPtr &depth,
           const nav_msgs::msg::Odometry::ConstSharedPtr &pose,
           double fx, double fy, double cx, double cy);
void sample(int u, int v, const Eigen::Vector3d &projected);
void finish();
void publish();
private:
void rgb(const sensor_msgs::msg::Image::ConstSharedPtr &image);
void attach(int64_t stamp);
bool safe(sparse::Vec2 p)const;
rclcpp::Node::SharedPtr node_;
SDFMap2D *map_;
sparse::TopologyConfig cfg_;
std::unique_ptr<sparse::TopologyGraph> graph_;
std::unique_ptr<sparse::DepthDescriptor> frame_;
cv::Mat raw_;
Eigen::Vector3d position_;
Eigen::Matrix3d rotation_;
double fx_, fy_, cx_, cy_, yaw_ = 0, min_height_ = 0.28, max_height_ = 1.18;
double update_hz_ = 2;
bool max_depth_is_free_ = true;
int image_width_ = 320;
int64_t stamp_ = 0, last_frame_ = 0, last_publish_ = 0;
std::string frame_id_ = "world";
std::map<int64_t, sensor_msgs::msg::Image::ConstSharedPtr> rgb_cache_;
std::map<int64_t, sparse::DirectionalImage> views_;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};
}
