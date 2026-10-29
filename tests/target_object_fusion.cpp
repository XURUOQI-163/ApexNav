// Real mapping regression: target evidence and confidence/count gating survive
// removal of all competing classes. No simulator or model services required.
#include <plan_env/map_ros.h>
#include <plan_env/sdf_map2d.h>
#include <plan_env/object_map2d.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <iostream>

using namespace apexnav_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("target_fusion_check");
  node->declare_parameter("sdf_map.resolution", 0.05);
  node->declare_parameter("sdf_map.map_size_x", 4.0);
  node->declare_parameter("sdf_map.map_size_y", 4.0);
  node->declare_parameter("sdf_map.obstacles_inflation", 0.1);
  node->declare_parameter("sdf_map.max_ray_length", 3.0);
  node->declare_parameter("object.min_observation_num", 2);
  node->declare_parameter("object.fusion_type", 1);
  auto map = std::make_shared<SDFMap2D>();
  map->initMap(node);
  Eigen::Vector2d position(0.51, 0.51);
  map->setForceOccGrid(position);
  auto object_map = map->object_map2d_;
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->push_back(pcl::PointXYZ(0.51f, 0.51f, 0.8f));
  auto detect = [&](double score) {
    DetectedObject detection;
    detection.cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
    detection.score = score;
    if (object_map->searchSingleObjectCluster(detection) != 0)
      throw std::runtime_error("Repeated target did not merge into the same instance");
  };
  auto selected = [&](double threshold) {
    object_map->setConfidenceThreshold(threshold);
    std::vector<pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> targets;
    object_map->getTopConfidenceObjectCloud(targets);
    return targets.size() == 1 && !targets.front()->empty();
  };
  auto require = [](bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
  };
  detect(0.8);
  require(!selected(0.1), "One detection must not pass the observation-count gate");
  detect(0.4);
  require(selected(0.59) && !selected(0.61), "Two equal observations must fuse to 0.6");
  object_map->inputObservationObjectsCloud({cloud}, 0.0);
  require(selected(0.39) && !selected(0.41), "Undetected target evidence must update 0.6 to 0.4");
  detect(0.8);
  require(selected(0.49) && !selected(0.51), "Target fusion must retain prior negative evidence");
  std::vector<std::vector<Eigen::Vector2d>> clusters;
  std::vector<Eigen::Vector2d> centers;
  object_map->getObjects(clusters, centers);
  require(clusters.size() == 1 && centers.size() == 1, "Instance geometry must survive fusion");
  std::cout << "PASS: target multi-frame fusion, count/confidence gates, negative evidence and geometry\n";
  rclcpp::shutdown();
}
