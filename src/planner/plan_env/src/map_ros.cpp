/**
 * @file map_ros.cpp
 * @brief Implementation of ROS2 interface for 2D SDF mapping system
 *
 * This file implements the MapROS class which provides the ROS2 interface for
 * the 2D signed distance field mapping system. It handles sensor data processing,
 * object detection integration, and real-time map visualization.
 *
 * @author Zager-Zhang
 */

#include <plan_env/map_ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

#include <vector>
#include <chrono>
#include <functional>

using namespace std;
using namespace std::chrono_literals;

namespace apexnav_planner {

void MapROS::setMap(SDFMap2D* map)
{
  this->map_ = map;
}

void MapROS::setNode(rclcpp::Node::SharedPtr node)
{
  node_ = node;
}

void MapROS::init()
{
  // Load camera intrinsic parameters from ROS2 parameter server
  if (!node_->has_parameter("map_ros/fx")) {
    node_->declare_parameter("map_ros/fx", -1.0);
  }
  if (!node_->has_parameter("map_ros/fy")) {
    node_->declare_parameter("map_ros/fy", -1.0);
  }
  if (!node_->has_parameter("map_ros/cx")) {
    node_->declare_parameter("map_ros/cx", -1.0);
  }
  if (!node_->has_parameter("map_ros/cy")) {
    node_->declare_parameter("map_ros/cy", -1.0);
  }
  fx_ = node_->get_parameter("map_ros/fx").as_double();
  fy_ = node_->get_parameter("map_ros/fy").as_double();
  cx_ = node_->get_parameter("map_ros/cx").as_double();
  cy_ = node_->get_parameter("map_ros/cy").as_double();

  // Load depth filtering parameters
  if (!node_->has_parameter("map_ros/depth_filter_maxdist")) {
    node_->declare_parameter("map_ros/depth_filter_maxdist", -1.0);
  }
  if (!node_->has_parameter("map_ros/depth_filter_mindist")) {
    node_->declare_parameter("map_ros/depth_filter_mindist", -1.0);
  }
  if (!node_->has_parameter("map_ros/depth_filter_margin")) {
    node_->declare_parameter("map_ros/depth_filter_margin", -1);
  }
  if (!node_->has_parameter("map_ros/filter_min_height")) {
    node_->declare_parameter("map_ros/filter_min_height", 0.5);
  }
  if (!node_->has_parameter("map_ros/filter_max_height")) {
    node_->declare_parameter("map_ros/filter_max_height", 0.88);
  }
  if (!node_->has_parameter("map_ros/k_depth_scaling_factor")) {
    node_->declare_parameter("map_ros/k_depth_scaling_factor", -1.0);
  }
  if (!node_->has_parameter("map_ros/skip_pixel")) {
    node_->declare_parameter("map_ros/skip_pixel", -1);
  }
  if (!node_->has_parameter("map_ros/frame_id")) {
    node_->declare_parameter("map_ros/frame_id", "world");
  }
  if (!node_->has_parameter("map_ros/virtual_ground_height")) {
    node_->declare_parameter("map_ros/virtual_ground_height", -0.28);
  }

  depth_filter_maxdist_ = node_->get_parameter("map_ros/depth_filter_maxdist").as_double();
  depth_filter_mindist_ = node_->get_parameter("map_ros/depth_filter_mindist").as_double();
  depth_filter_margin_ = node_->get_parameter("map_ros/depth_filter_margin").as_int();
  filter_min_height_ = node_->get_parameter("map_ros/filter_min_height").as_double();
  filter_max_height_ = node_->get_parameter("map_ros/filter_max_height").as_double();
  k_depth_scaling_factor_ = node_->get_parameter("map_ros/k_depth_scaling_factor").as_double();
  skip_pixel_ = node_->get_parameter("map_ros/skip_pixel").as_int();
  frame_id_ = node_->get_parameter("map_ros/frame_id").as_string();
  virtual_ground_height_ = node_->get_parameter("map_ros/virtual_ground_height").as_double();

  // Handle Habitat simulator vs real-world configuration
  if (!node_->has_parameter("is_real_world")) {
    node_->declare_parameter("is_real_world", false);
  }
  bool is_real_world = node_->get_parameter("is_real_world").as_bool();

  if (!is_real_world) {
    // Override depth parameters with Habitat simulator settings
    if (!node_->has_parameter("habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.max_depth")) {
      node_->declare_parameter("habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.max_depth", -1.0);
    }
    if (!node_->has_parameter("habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.min_depth")) {
      node_->declare_parameter("habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.min_depth", -1.0);
    }
    double habitat_max_depth = node_->get_parameter("habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.max_depth").as_double();
    double habitat_min_depth = node_->get_parameter("habitat.simulator.agents.main_agent.sim_sensors.depth_sensor.min_depth").as_double();
    if (habitat_max_depth != -1.0 && habitat_min_depth != -1.0) {
      depth_filter_maxdist_ = habitat_max_depth;
      depth_filter_mindist_ = habitat_min_depth;
      RCLCPP_WARN(node_->get_logger(),
          "Using habitat simulator params, set depth_filter_range = [%.2f, %.2f] m",
          habitat_min_depth, habitat_max_depth);
    }
  }

  // Initialize point cloud data structures
  depth_cloud_.reset(new PointCloud3D());
  filtered_depth_cloud2d_.reset(new PointCloud2D());

  // Pre-allocate point cloud vectors for efficiency
  proj_points_.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  depth_cloud_->points.resize(640 * 480 / (skip_pixel_ * skip_pixel_));
  proj_points_cnt_ = 0;
  depth_image_.reset(new cv::Mat);

  // Initialize state flags
  local_updated_ = false;
  esdf_need_update_ = false;

  // Setup periodic timers for map updates and visualization
  esdf_timer_ = node_->create_wall_timer(100ms, std::bind(&MapROS::updateESDFCallback, this));
  vis_timer_ = node_->create_wall_timer(250ms, std::bind(&MapROS::visCallback, this));

  // Setup publishers for map visualization
  occupied_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/occupied", 10);
  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/unknown", 10);
  free_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/free", 10);
  occupied_inflate_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/occupied_inflate", 10);

  object_grid_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/occupancy_object", 10);
  esdf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/esdf", 10);
  update_range_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "/grid_map/update_range", 10);
  depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/depth_cloud", 10);
  filtered_depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/filtered_depth_cloud", 10);
  filtered_object_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/filtered_object_cloud", 10);
  all_object_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/all_object_cloud", 10);
  over_depth_object_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/over_depth_object_cloud", 10);
  value_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/grid_map/value_map", 10);
  confidence_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/grid_map/confidence_map", 10);

  // Setup subscribers for object detection and CLIP scores
  detected_object_cloud_sub_ = node_->create_subscription<plan_env::msg::MultipleMasksWithConfidence>(
      "/yoloe/clouds_with_scores", 10,
      std::bind(&MapROS::detectedObjectCloudCallback, this, std::placeholders::_1));
  semantic_score_sub_ = node_->create_subscription<plan_env::msg::SemanticScore>(
      "/semantic/score", 10,
      std::bind(&MapROS::semanticScoreCallback, this, std::placeholders::_1));

  // Setup synchronized subscribers for depth image and pose data
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      node_, "/map_ros/depth", qos_profile);
  pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
      node_, "/map_ros/pose", qos_profile);

  sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
      SyncPolicyImagePose(20), *depth_sub_, *pose_sub_);
  sync_image_pose_->registerCallback(
      std::bind(&MapROS::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize object tracking variables
  continue_over_depth_count_ = -1;
  map_start_time_ = node_->get_clock()->now();
  if (!node_->has_parameter("semantic/max_observation_age"))
    node_->declare_parameter("semantic/max_observation_age", 15.0);
  if (!node_->has_parameter("semantic/cache_size"))
    node_->declare_parameter("semantic/cache_size", 600);
  const double max_age = node_->get_parameter("semantic/max_observation_age").as_double();
  const int capacity = node_->get_parameter("semantic/cache_size").as_int();
  if (!std::isfinite(max_age) || max_age <= 0 || max_age > 3600 || capacity <= 0)
    throw std::invalid_argument("Invalid semantic observation cache parameters");
  semantic_cache_ = SemanticCache(capacity, static_cast<int64_t>(max_age * 1e9),
                                  map_start_time_.nanoseconds());
  semantic_backend_.clear();
}

void MapROS::visCallback()
{
  // Publish all visualization topics
  publishOccupied();
  publishInfOccupied();
  publishObjectMap();
  publishUnknown();
  publishFree();
  publishValueMap();
  // publishConfidenceMap();
  publishESDFMap();
  // publishUpdateRange();
}

void MapROS::semanticScoreCallback(const plan_env::msg::SemanticScore::SharedPtr msg)
{
  const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  const int64_t now = node_->get_clock()->now().nanoseconds();
  if (stamp <= map_start_time_.nanoseconds() || msg->header.frame_id != frame_id_ ||
      (msg->backend != "clip" && msg->backend != "blip2") ||
      !std::isfinite(msg->score) || msg->score < -1.0 || msg->score > 1.0)
    return;
  if (semantic_backend_.empty()) semantic_backend_ = msg->backend;
  if (semantic_backend_ != msg->backend) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "[Semantic] Mixed backends rejected; restart planner when switching models");
    return;
  }
  applySemanticMatch(semantic_cache_.addScore(stamp, now, msg->score));
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "[Semantic] backend=%s age=%.3fs cached=%zu discarded=%zu",
      msg->backend.c_str(), (now - stamp) * 1e-9,
      semantic_cache_.size(), semantic_cache_.dropped());
}

void MapROS::applySemanticMatch(const std::optional<SemanticCache::Match>& match)
{
  if (!match) return;
  const auto& observation = *match->observation;
  map_->value_map_->updateValueMap(observation.camera_pos, observation.camera_yaw,
                                  observation.free_grids, match->score);
  match->observation->score = match->score;
  applyObjectObservation(match->observation);
  RCLCPP_DEBUG(node_->get_logger(), "[Semantic] fused stamp=%lld pose=(%.3f,%.3f) score=%.4f cells=%zu",
      static_cast<long long>(match->stamp), observation.camera_pos.x(), observation.camera_pos.y(),
      match->score, observation.free_grids.size());
}

void MapROS::applyObjectObservation(const std::shared_ptr<SemanticObservation>& observation)
{
  if (!observation->has_detection || !observation->score || observation->object_observation_applied)
    return;
  observation->object_observation_applied = true;
  getObservationObjectsCloud(observation->detected_ids, observation->depth_cloud, *observation->score);
}

void MapROS::detectedObjectCloudCallback(const plan_env::msg::MultipleMasksWithConfidence::SharedPtr msg)
{
  const auto original = semantic_cache_.observation(
      rclcpp::Time(msg->header.stamp).nanoseconds(), node_->get_clock()->now().nanoseconds());
  if (!original || msg->header.frame_id != frame_id_ || (*original)->has_detection) return;
  const auto observation = *original;
  // Camera-dependent filtering must use the pose that produced these detections.
  const auto& camera_pos_ = observation->camera_pos3;
  const auto& camera_q_ = observation->camera_q;
  // Validate message structure consistency
  if (!(msg->confidence_scores.size() == msg->point_clouds.size() &&
          msg->confidence_scores.size() == msg->label_indices.size())) {
    RCLCPP_ERROR(node_->get_logger(), "[Bug] The MultipleMasksWithConfidence msg is wrong!!!");
    return;
  }

  auto t1 = node_->get_clock()->now();

  // Check camera orientation - only process when looking down (for better object detection)
  Eigen::Vector3d euler =
      camera_q_.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order: yaw, roll, pitch
  if (euler[2] < 0)
    euler[2] += M_PI;
  double camera_pitch = euler[2];
  if (camera_pitch < 1.5)  // Skip if camera not tilted down enough
    return;

  // Backup previous over-depth object cloud for consistency tracking
  auto last_over_depth_cloud =
      std::make_shared<PointCloud3D>(*map_->object_map2d_->over_depth_object_cloud_);
  map_->object_map2d_->over_depth_object_cloud_.reset(new PointCloud3D());

  // Initialize point cloud processing tools and containers
  pcl::VoxelGrid<Point3D> voxel_filter;
  PointCloud3D::Ptr all_object_cloud(new PointCloud3D());
  PointCloud3D::Ptr filtered_all_object_cloud(new PointCloud3D());
  vector<DetectedObject> detected_objects;

  // Process each detected object in the message
  for (size_t i = 0; i < msg->confidence_scores.size(); i++) {
    auto cloud = msg->point_clouds[i];
    auto confidence_score = msg->confidence_scores[i];
    auto label = msg->label_indices[i];

    // Convert ROS2 message to PCL point cloud
    PointCloud3D::Ptr single_object_cloud(new PointCloud3D());
    pcl::fromROSMsg(cloud, *single_object_cloud);
    *all_object_cloud += *single_object_cloud;

    // Apply voxel grid downsampling to reduce computational load
    voxel_filter.setInputCloud(single_object_cloud);
    voxel_filter.setLeafSize(0.04f, 0.04f, 0.06f);
    voxel_filter.filter(*single_object_cloud);

    // Filter out points beyond sensor accuracy range (>5m depth is unreliable)
    PointCloud3D::Ptr tmp_object_cloud(new PointCloud3D());
    PointCloud3D::Ptr over_depth_object_cloud(new PointCloud3D());
    for (auto object_pt : single_object_cloud->points) {
      Eigen::Vector3d object_pt3d = Eigen::Vector3d(object_pt.x, object_pt.y, object_pt.z);
      if ((object_pt3d - camera_pos_).norm() > depth_filter_maxdist_ - 0.10) {
        // Store over-depth points for target objects (label == 0) for tracking consistency
        if (label == 0)
          over_depth_object_cloud->points.push_back(object_pt);
        continue;
      }
      tmp_object_cloud->points.push_back(object_pt);
    }
    single_object_cloud = tmp_object_cloud;

    // Skip objects that are entirely beyond valid depth range
    if (single_object_cloud->points.empty()) {
      if (!over_depth_object_cloud->points.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Have all over depth object cloud!!!!");
        *map_->object_map2d_->over_depth_object_cloud_ += *over_depth_object_cloud;
      }
      continue;
    }

    // Apply DBSCAN clustering to remove noise and outliers
    single_object_cloud = dbscan(single_object_cloud, 0.12f, 10);
    if (single_object_cloud == nullptr) {
      RCLCPP_ERROR(node_->get_logger(), "After DBSCAN, no point cloud cluster!!");
      continue;
    }

    if (single_object_cloud->points.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Single object point cloud is empty!!!");
      continue;
    }

    // Accumulate filtered object data
    *filtered_all_object_cloud += *single_object_cloud;
    DetectedObject detected_object;
    detected_object.cloud = single_object_cloud;
    detected_object.score = confidence_score;
    detected_object.label = label;
    detected_objects.push_back(detected_object);
  }

  // Maintain consistency in over-depth object tracking
  if (continue_over_depth_count_ == -1 &&
      !map_->object_map2d_->over_depth_object_cloud_->points.empty())
    continue_over_depth_count_ = 0;
  else if (continue_over_depth_count_ <= 4 && continue_over_depth_count_ >= 0) {
    continue_over_depth_count_++;
    *map_->object_map2d_->over_depth_object_cloud_ = *last_over_depth_cloud;
  }
  else {
    continue_over_depth_count_ = -1;
  }

  // Publish visualization point clouds for debugging and monitoring
  publishPointCloud(filtered_object_cloud_pub_, filtered_all_object_cloud);
  publishPointCloud(all_object_cloud_pub_, all_object_cloud);
  publishPointCloud(over_depth_object_cloud_pub_, map_->object_map2d_->over_depth_object_cloud_);

  // Update object map with processed detection results
  *map_->object_map2d_->all_object_clouds_ = *filtered_all_object_cloud;
  vector<int> detected_object_cluster_ids;
  map_->inputObjectCloud2D(detected_objects, detected_object_cluster_ids);

  // Extract observation data from depth sensor for objects not detected by vision
  observation->detected_ids = std::move(detected_object_cluster_ids);
  observation->has_detection = true;
  applyObjectObservation(observation);

  double object_map_process_time = (node_->get_clock()->now() - t1).seconds();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
      "[Calculating Time] Object Map process time = %.3f s", object_map_process_time);
}

void MapROS::updateESDFCallback()
{
  if (!esdf_need_update_)
    return;

  auto t1 = node_->get_clock()->now();
  map_->updateESDFMap();
  esdf_need_update_ = false;
  double esdf_time = (node_->get_clock()->now() - t1).seconds();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 50000,
      "[Calculating Time] ESDF Map process time = %.3f s", esdf_time);
}

void MapROS::depthPoseCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    const nav_msgs::msg::Odometry::ConstSharedPtr& pose)
{
  // Extract camera pose from odometry message
  camera_pos_(0) = pose->pose.pose.position.x;
  camera_pos_(1) = pose->pose.pose.position.y;
  camera_pos_(2) = pose->pose.pose.position.z;
  camera_q_ = Eigen::Quaterniond(pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
      pose->pose.pose.orientation.y, pose->pose.pose.orientation.z);

  // Calculate camera yaw angle for value map updates
  Eigen::Vector3d euler =
      camera_q_.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order: yaw, roll, pitch
  double camera_yaw = euler[0];
  Eigen::Vector2d camera_pos = Eigen::Vector2d(camera_pos_(0), camera_pos_(1));

  // Skip processing if camera is outside map bounds
  if (!map_->isInMap(camera_pos))
    return;

  // Convert depth image format (Habitat publishes Float32, some sensors use 8UC1)
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor_);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_8UC1)
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, 255.0);
  cv_ptr->image.copyTo(*depth_image_);

  auto t1 = node_->get_clock()->now();

  // Process depth image into 3D point cloud and filter to 2D representation
  processDepthImage();
  filterPointCloudToXY();

  // Update occupancy grid with filtered depth data
  vector<Eigen::Vector2i> free_grids;
  // Dilate free_grids to ensure more complete coverage
  dilateGrids(free_grids, 1);
  map_->inputDepthCloud2D(filtered_depth_cloud2d_, camera_pos_, free_grids);
  double process_time = (node_->get_clock()->now() - t1).seconds();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 50000,
      "[Calculating Time] Grid Map process time = %.3f s", process_time);

  // Save geometry from this acquisition, not the camera pose at inference completion.
  // The score may arrive before or after this depth/pose pair; each stamp fuses once.
  auto observation = std::make_shared<SemanticObservation>();
  observation->camera_pos = camera_pos;
  observation->camera_yaw = camera_yaw;
  observation->free_grids = std::move(free_grids);
  observation->camera_pos3 = camera_pos_;
  observation->camera_q = camera_q_;
  // Retain a compact depth snapshot for time-aligned negative object evidence.
  PointCloud3D::Ptr valid_cloud(new PointCloud3D());
  valid_cloud->points.assign(depth_cloud_->points.begin(), depth_cloud_->points.begin() + proj_points_cnt_);
  observation->depth_cloud.reset(new PointCloud3D());
  pcl::VoxelGrid<Point3D> snapshot_filter;
  snapshot_filter.setInputCloud(valid_cloud);
  snapshot_filter.setLeafSize(0.1f, 0.1f, 0.1f);
  snapshot_filter.filter(*observation->depth_cloud);
  applySemanticMatch(semantic_cache_.addObservation(
      rclcpp::Time(img->header.stamp).nanoseconds(), node_->get_clock()->now().nanoseconds(), observation));

  // Trigger ESDF update if local map has been updated
  if (local_updated_) {
    map_->clearAndInflateLocalMap();
    esdf_need_update_ = true;
    local_updated_ = false;
  }
}

void MapROS::processDepthImage()
{
  proj_points_cnt_ = 0;

  uint16_t* row_ptr;
  int cols = depth_image_->cols;
  int rows = depth_image_->rows;
  double depth;
  Eigen::Matrix3d camera_r = camera_q_.toRotationMatrix();
  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / k_depth_scaling_factor_;

  // Iterate through depth image pixels with margin and skipping for efficiency
  for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_) {
    row_ptr = depth_image_->ptr<uint16_t>(v) + depth_filter_margin_;
    for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_) {
      // Convert pixel depth value to metric distance
      depth = (*row_ptr) * inv_factor * (depth_filter_maxdist_ - depth_filter_mindist_) +
              depth_filter_mindist_;
      row_ptr = row_ptr + skip_pixel_;

      // Apply depth range filtering
      if (depth > depth_filter_maxdist_)
        depth = depth_filter_maxdist_;
      else if (depth < depth_filter_mindist_)
        continue;

      // Project pixel to 3D camera coordinates
      pt_cur(0) = (u - cx_) * depth / fx_;
      pt_cur(1) = (v - cy_) * depth / fy_;
      pt_cur(2) = depth;

      // Transform to world coordinates
      pt_world = camera_r * pt_cur + camera_pos_;
      auto& pt = depth_cloud_->points[proj_points_cnt_++];
      pt.x = pt_world[0];
      pt.y = pt_world[1];
      pt.z = pt_world[2];
    }
  }
  publishPointCloud(depth_cloud_pub_, depth_cloud_);
}

void MapROS::getObservationObjectsCloud(const std::vector<int>& filter_object_ids,
    const PointCloud3D::Ptr& filtered_depth_cloud, double score)
{

  // Get object bounding boxes and create filter flags
  vector<Eigen::Vector3d> bmins, bmaxs;
  map_->object_map2d_->getObjectBoxes(bmins, bmaxs);
  vector<char> filter_object_flag(bmins.size(), 0);
  for (auto filter_object_id : filter_object_ids)
    if (filter_object_id >= 0 && static_cast<size_t>(filter_object_id) < filter_object_flag.size())
      filter_object_flag[filter_object_id] = 1;

  // Use CropBox filter to extract points within object bounding boxes
  pcl::CropBox<Point3D> crop_box_filter;
  crop_box_filter.setInputCloud(filtered_depth_cloud);
  vector<pcl::shared_ptr<PointCloud3D>> observation_clouds;

  for (size_t i = 0; i < bmins.size(); i++) {
    PointCloud3D::Ptr cloud_filtered(new PointCloud3D);
    if (filter_object_flag[i])
      observation_clouds.push_back(cloud_filtered);  // Empty cloud for detected objects
    else {
      // Extract points within bounding box for undetected objects
      double inf = 0.2f;  // Inflation factor for bounding box
      Eigen::Vector4f min_point(bmins[i][0] - inf, bmins[i][1] - inf, bmins[i][2] - inf, 1.0);
      Eigen::Vector4f max_point(bmaxs[i][0] + inf, bmaxs[i][1] + inf, bmaxs[i][2] + inf, 1.0);
      crop_box_filter.setMin(min_point);
      crop_box_filter.setMax(max_point);
      crop_box_filter.filter(*cloud_filtered);
      observation_clouds.push_back(cloud_filtered);
    }
  }

  // Original fusion rule, using only the score from this depth observation.
  map_->object_map2d_->inputObservationObjectsCloud(observation_clouds, max(0.0, score));
}

void MapROS::filterPointCloudToXY()
{
  // Default ground height assumption (currently set to 0)
  double cur_floor_height = 0.0;
  double virtual_ground = virtual_ground_height_;

  auto t1 = node_->get_clock()->now();
  PointCloud3D::Ptr filtered_cloud_3d(new PointCloud3D());
  PointCloud3D::Ptr down_depth_cloud_3d(new PointCloud3D());
  PointCloud3D::Ptr under_ground_cloud_3d(new PointCloud3D());
  PointCloud2D::Ptr under_ground_cloud_2d(new PointCloud2D());

  // Downsample point cloud for efficient processing
  pcl::VoxelGrid<Point3D> voxel_filter;
  voxel_filter.setInputCloud(depth_cloud_);
  voxel_filter.setLeafSize(0.04f, 0.04f, 0.1f);  // Different resolution for XY vs Z
  voxel_filter.filter(*down_depth_cloud_3d);

  filtered_depth_cloud2d_->clear();

  // Separate points by height categories
  for (size_t i = 0; i < down_depth_cloud_3d->points.size(); i++) {
    Point3D pt;
    pt.x = down_depth_cloud_3d->points[i].x;
    pt.y = down_depth_cloud_3d->points[i].y;
    pt.z = down_depth_cloud_3d->points[i].z;

    // Points below virtual ground (for virtual ground generation)
    if (down_depth_cloud_3d->points[i].z < cur_floor_height + virtual_ground)
      under_ground_cloud_3d->points.push_back(pt);
    // Points in obstacle height range
    else if (down_depth_cloud_3d->points[i].z > cur_floor_height + filter_min_height_ &&
             down_depth_cloud_3d->points[i].z < cur_floor_height + filter_max_height_)
      filtered_cloud_3d->points.push_back(pt);
  }

  pcl::RadiusOutlierRemoval<Point3D> outrem;

  // Remove outliers from obstacle points (handles noisy depth data from datasets)
  if (!filtered_cloud_3d->points.empty()) {
    outrem.setInputCloud(filtered_cloud_3d);
    outrem.setRadiusSearch(0.3);         // Search radius for neighbors
    outrem.setMinNeighborsInRadius(35);  // Minimum neighbor threshold
    outrem.filter(*filtered_cloud_3d);
  }

  publishPointCloud(filtered_depth_cloud_pub_, filtered_cloud_3d);

  // Project 3D obstacle points to 2D for occupancy mapping
  for (auto pt : filtered_cloud_3d->points) {
    Point2D pt_xy;
    pt_xy.x = pt.x;
    pt_xy.y = pt.y;
    filtered_depth_cloud2d_->points.push_back(pt_xy);
  }

  // Remove outliers from under-ground points (handles noisy depth data)
  if (!under_ground_cloud_3d->points.empty()) {
    outrem.setInputCloud(under_ground_cloud_3d);
    outrem.setRadiusSearch(0.21);        // Smaller search radius for ground points
    outrem.setMinNeighborsInRadius(40);  // Higher neighbor threshold
    outrem.filter(*under_ground_cloud_3d);
  }

  // Add virtual ground points to prevent getting stuck when going downstairs
  Eigen::Vector3d euler =
      camera_q_.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order: yaw roll pitch
  if (euler[2] < 0)
    euler[2] += M_PI;
  double camera_pitch = euler[2];

  // When camera is pointing down (pitch > 1.5 rad) and under-ground points exist
  if (camera_pitch > 1.5 && !under_ground_cloud_3d->points.empty()) {
    for (auto pt : under_ground_cloud_3d->points) {
      Eigen::Vector3d pt_pos = Eigen::Vector3d(pt.x, pt.y, pt.z);
      Eigen::Vector2d ground_pos;

      // Interpolate ray from camera to point, finding intersection with virtual ground
      if (interpolateLineAtZ(pt_pos, camera_pos_, cur_floor_height + virtual_ground, ground_pos)) {
        Point2D pt_xy;
        pt_xy.x = ground_pos(0);
        pt_xy.y = ground_pos(1);
        filtered_depth_cloud2d_->points.push_back(pt_xy);
        under_ground_cloud_2d->points.push_back(pt_xy);
      }
    }
    map_->inputVirtualGround(under_ground_cloud_2d);
  }

  double filter_time = (node_->get_clock()->now() - t1).seconds();
  if (filter_time > 0.1) {
    RCLCPP_WARN(node_->get_logger(),
        "Filter point cloud time maybe a little long = %.3f ms", filter_time * 1000);
  }
}

bool MapROS::interpolateLineAtZ(
    const Eigen::Vector3d& A, const Eigen::Vector3d& B, double target_z, Eigen::Vector2d& P)
{
  // Check if target_z is between A.z and B.z (intersection possible)
  if ((A.z() - target_z) * (B.z() - target_z) > 0)
    return false;  // target_z not within segment bounds

  // Calculate interpolation parameter t (0 = point A, 1 = point B)
  double t = (target_z - A.z()) / (B.z() - A.z());

  // Linear interpolation for X and Y coordinates
  double x = A.x() + t * (B.x() - A.x());
  double y = A.y() + t * (B.y() - A.y());
  P = Eigen::Vector2d(x, y);
  return true;
}

PointCloud3D::Ptr MapROS::dbscan(const PointCloud3D::Ptr& cloud, double eps, int minPts)
{
  if (cloud->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[DBSCAN] Input cloud is empty!");
    return nullptr;
  }

  // Build KD-tree for efficient neighbor search
  pcl::search::KdTree<Point3D>::Ptr tree(new pcl::search::KdTree<Point3D>);
  tree->setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;

  // Use PCL's EuclideanClusterExtraction to implement DBSCAN-like clustering
  pcl::EuclideanClusterExtraction<Point3D> ec;
  ec.setClusterTolerance(eps);                 // Neighborhood radius
  ec.setMinClusterSize(minPts);                // Minimum points per cluster
  ec.setMaxClusterSize(cloud->points.size());  // Maximum cluster size (full cloud)
  ec.setSearchMethod(tree);                    // Set KD-Tree for neighbor search
  ec.setInputCloud(cloud);                     // Input point cloud
  ec.extract(cluster_indices);                 // Extract clustering results

  // Return null if no clusters found
  if (cluster_indices.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[DBSCAN] No clusters found!");
    return nullptr;
  }

  // Find the largest cluster by counting points
  int largest_cluster_index = -1;
  size_t max_size = 0;
  for (size_t i = 0; i < cluster_indices.size(); ++i) {
    if (cluster_indices[i].indices.size() > max_size) {
      max_size = cluster_indices[i].indices.size();
      largest_cluster_index = i;
    }
  }

  // Create new point cloud containing only the largest cluster
  PointCloud3D::Ptr largest_cluster(new PointCloud3D);
  for (int idx : cluster_indices[largest_cluster_index].indices)
    largest_cluster->points.push_back(cloud->points[idx]);
  return largest_cluster;
}

}  // namespace apexnav_planner
