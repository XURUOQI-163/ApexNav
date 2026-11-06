#pragma once
#include <plan_env/skeleton_graph.h>
#include <plan_env/skeleton_frontier_links.h>
#include <plan_env/msg/skeleton_graph.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <future>

namespace apexnav_planner {
class SDFMap2D;
// Read-only observer of the existing map. The default mutually-exclusive ROS
// callback group serializes sampling against map updates. Worker owns its input
// and never accesses the live map or ROS node.
class SkeletonGraphManager {
public:
  SkeletonGraphManager(const rclcpp::Node::SharedPtr& node,SDFMap2D* map,const std::string& frame);
  ~SkeletonGraphManager();
private:
  struct Result {
    skeleton::Graph graph;skeleton::Fingerprint fingerprint;
    std::vector<skeleton::FrontierLink> links;
    uint64_t frontier_signature;
    bool geometry_rebuilt;
    double build_ms,link_ms;
  };
  void tick();
  skeleton::GridInput sample() const;
  std::vector<skeleton::Frontier> sampleFrontiers() const;
  bool validateLinks(const std::vector<skeleton::Frontier>& current);
  void publish(const rclcpp::Node::SharedPtr& node);
  rclcpp::Node::WeakPtr node_;
  SDFMap2D* map_;
  std::string frame_;
  skeleton::Options options_;
  int tile_size_;
  std::unique_ptr<skeleton::ChangeGate> gate_,link_gate_;
  skeleton::Graph graph_;
  std::vector<skeleton::FrontierLink> links_;
  uint64_t accepted_frontiers_=0;
  bool connect_frontiers_=true,geometry_rebuilt_=false;
  double max_association_distance_=.5,association_dash_length_=.04;
  double link_ms_=0;
  std::future<Result> job_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<plan_env::msg::SkeletonGraph>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  uint64_t revision_=0;
  uint32_t changed_tiles_=0;
  double build_ms_=0,sampling_ms_=0,z_,frontier_z_,node_scale_,edge_width_,label_scale_;
  bool labels_;
  builtin_interfaces::msg::Time sampled_at_;
};
}
