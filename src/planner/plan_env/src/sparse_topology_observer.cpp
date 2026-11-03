#include <plan_env/sparse_topology_observer.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sstream>

namespace apexnav_planner {
using namespace sparse;
SparseTopologyObserver::SparseTopologyObserver(rclcpp::Node::SharedPtr node, SDFMap2D *map) : node_(node), map_(map) {
  auto number = [&](const std::string &key, double value) {
                  std::string name = "sparse_topology." + key;
                  if (!node_->has_parameter(name)) {
                    node_->declare_parameter(name, value);
                  }
                  return node_->get_parameter(name).as_double();
                };
  cfg_.sector_degrees = number("sector_degrees", 30);
  cfg_.max_distance = number("max_distance", 5);
  cfg_.open_distance = number("open_distance", cfg_.max_distance * .9);
  cfg_.candidate_step = number("candidate_step", std::min(3.5, cfg_.max_distance * .7));
  cfg_.jump_distance = number("jump_distance", 1);
  cfg_.reach_radius = number("reach_radius", .4);
  cfg_.duplicate_radius = number("duplicate_radius", .7);
  cfg_.waypoint_spacing = number("waypoint_spacing", 1.5);
  cfg_.clearance = number("clearance", .18);
  cfg_.ray_step = map_->getResolution() * .5;
  update_hz_ = number("update_hz", 2);
  min_height_ = node_->get_parameter("map_ros/filter_min_height").as_double();
  max_height_ = node_->get_parameter("map_ros/filter_max_height").as_double();
  frame_id_ = node_->get_parameter("map_ros/frame_id").as_string();
  if (!node_->has_parameter("sparse_topology.max_depth_is_free")) {
    node_->declare_parameter("sparse_topology.max_depth_is_free", true);
  }
  max_depth_is_free_ = node_->get_parameter("sparse_topology.max_depth_is_free").as_bool();
  if (!std::isfinite(update_hz_) || update_hz_ <= 0 || update_hz_ > 30) {
    throw std::invalid_argument("topology update_hz must be 0..30");
  }
  graph_ = std::make_unique<TopologyGraph>(cfg_);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/sparse_topology/markers", rclcpp::QoS(1).transient_local());
  state_pub_ = node_->create_publisher<std_msgs::msg::String>("/sparse_topology/state", rclcpp::QoS(1).transient_local());
  rgb_sub_ = node_->create_subscription<sensor_msgs::msg::Image>("/habitat/camera_rgb", rclcpp::SensorDataQoS(),
                                                                 [this](sensor_msgs::msg::Image::ConstSharedPtr img) {
      rgb(img);
    });
  publish();
}
void SparseTopologyObserver::begin(const sensor_msgs::msg::Image::ConstSharedPtr &depth,
                                   const nav_msgs::msg::Odometry::ConstSharedPtr &pose, double fx, double fy, double cx, double cy) {
  frame_.reset();
  stamp_ = rclcpp::Time(depth->header.stamp).nanoseconds();
  if (stamp_ <= last_frame_ || stamp_ - last_frame_ < int64_t(1e9 / update_hz_)) {
    return;
  }
  last_frame_ = stamp_;
  fx_ = fx;
  fy_ = fy;
  cx_ = cx;
  cy_ = cy;
  if (fx <= 0 || fy <= 0) {
    return;
  }
  raw_ = cv_bridge::toCvShare(depth)->image;
  // This mapper's Habitat depth contract is normalized 32FC1 or normalized 16UC1.
  if (raw_.type() != CV_32FC1 && raw_.type() != CV_16UC1) {
    return;
  }
  position_ = {pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z};
  const auto &q = pose->pose.pose.orientation;
  rotation_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  auto forward = rotation_.col(2);
  yaw_ = std::atan2(forward.y(), forward.x());
  frame_ = std::make_unique<DepthDescriptor>(cfg_.sector_degrees, cfg_.max_distance, Vec2{position_.x(), position_.y()});
  DirectionalImage image;
  image.stamp = stamp_;
  image.camera_position = frame_->origin();
  image.camera_z = position_.z();
  image.yaw = yaw_;
  image.orientation = {q.x, q.y, q.z, q.w};
  views_[stamp_] = image;
  while (views_.size() > 20) {
    views_.erase(views_.begin());
  }
}
void SparseTopologyObserver::sample(int u, int v, const Eigen::Vector3d &projected) {
  if (!frame_) {
    return;
  }
  const double value = raw_.type() == CV_32FC1 ? raw_.at<float>(v, u) : raw_.at<uint16_t>(v, u) / 65535.;
  const auto delta = projected - position_;
  const bool valid = std::isfinite(value) && value > 0 && value <= 1.;
  if (!valid) {
    const Eigen::Vector3d ray = rotation_ * Eigen::Vector3d((u - cx_) / fx_, (v - cy_) / fy_, 1.);
    if (std::abs(ray.z()) < .15 * std::hypot(ray.x(), ray.y())) {
      frame_->addRay(std::atan2(ray.y(), ray.x()), 0, false, false, stamp_);
    }
    return;
  }
  // Same obstacle-height band as navigation; floor/ceiling rays cannot turn into
  // phantom walls or clear whole unseen sectors. Valid central rays provide support.
  if (projected.z() < min_height_ || projected.z() > max_height_) {
    return;
  }
  const bool saturated = value >= .999;
  frame_->addRay(std::atan2(delta.y(), delta.x()), delta.head<2>().norm(),
                 !saturated || max_depth_is_free_, !saturated, stamp_);
}
bool SparseTopologyObserver::safe(Vec2 p)const {
  // Direct grid queries, no PointCloud2 round trip. Check a footprint, not only a center.
  Eigen::Vector2d center(p.x, p.y);
  if (!map_->isInMap(center) || map_->getInflateOccupancy(center)) {
    return false;
  }
  Eigen::Vector2d origin, size;
  map_->getRegion(origin, size);
  return footprintClear(p, cfg_.clearance, map_->getResolution(), {origin.x(), origin.y()},
                        [this](int x, int y) {
      const Eigen::Vector2i cell(x, y);
      return map_->isInMap(cell) && map_->getOccupancy(cell) == SDFMap2D::FREE;
    });
}
void SparseTopologyObserver::finish() {
  if (!frame_) {
    return;
  }
  frame_->finish(stamp_);
  graph_->update(frame_->origin(), yaw_, stamp_, *frame_, [this](Vec2 p) {
      return safe(p);
    });
  attach(stamp_);
  frame_.reset();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                       "[SparseTopology] WAYPOINT=%zu FRONTIER=%zu edges=%zu created=%lu converted=%lu duplicate_rejections=%lu covered=%lu removed=%lu revisits=%lu",
                       graph_->waypointCount(), graph_->frontierCount(), graph_->edges().size(), graph_->stats().created, graph_->stats().converted,
                       graph_->stats().duplicates, graph_->stats().covered, graph_->stats().removed, graph_->stats().revisits);
}
void SparseTopologyObserver::rgb(const sensor_msgs::msg::Image::ConstSharedPtr &image) {
  if (image->header.frame_id != frame_id_) {
    return;
  }
  auto stamp = rclcpp::Time(image->header.stamp).nanoseconds();
  rgb_cache_[stamp] = image;
  while (rgb_cache_.size() > 20) {
    rgb_cache_.erase(rgb_cache_.begin());
  }
  attach(stamp);
}
void SparseTopologyObserver::attach(int64_t stamp) {
  if (!graph_->wantsImage(stamp) || !rgb_cache_.count(stamp) || !views_.count(stamp)) {
    return;
  }
  try{
    cv::Mat image = cv_bridge::toCvCopy(rgb_cache_.at(stamp), "bgr8")->image;
    if (image.cols > image_width_) {
      cv::resize(image, image, cv::Size(image_width_, image.rows * image_width_ / image.cols));
    }
    auto bytes = std::make_shared<std::vector<unsigned char> >();
    cv::imencode(".jpg", image, *bytes, {cv::IMWRITE_JPEG_QUALITY, 75});
    auto view = views_.at(stamp);
    view.jpeg = bytes;
    graph_->attachImage(view);
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "[SparseTopology] RGB encode: %s", e.what());
  }
}
void SparseTopologyObserver::publish() {
  using M = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray output;
  M clear;
  clear.header.frame_id = frame_id_;
  clear.header.stamp = node_->now();
  clear.action = M::DELETEALL;
  output.markers.push_back(clear);
  for (const auto& [id, n]:graph_->nodes()) {
    M marker;
    marker.header = clear.header;
    marker.ns = "topology_nodes";
    marker.id = int(id);
    marker.type = M::SPHERE;
    marker.action = M::ADD;
    marker.lifetime.sec = 2;
    marker.pose.orientation.w = 1;
    marker.pose.position.x = n.position.x;
    marker.pose.position.y = n.position.y;
    marker.pose.position.z = .25;
    marker.scale.x = marker.scale.y = marker.scale.z = .22;
    marker.color.a = 1;
    if (n.type == NodeType::WAYPOINT) {
      marker.color.g = 1;
    }
    else{
      marker.color.r = 1;
    }
    output.markers.push_back(marker);
    marker.ns = "topology_ids";
    marker.type = M::TEXT_VIEW_FACING;
    marker.text = std::to_string(id);
    marker.pose.position.z = .52;
    marker.scale.z = .20;
    output.markers.push_back(marker);
  }
  M lines;
  lines.header = clear.header;
  lines.ns = "topology_edges";
  lines.id = 0;
  lines.type = M::LINE_LIST;
  lines.action = M::ADD;
  lines.lifetime.sec = 2;
  lines.pose.orientation.w = 1;
  lines.scale.x = .035;
  lines.color.b = 1;
  lines.color.a = 1;
  for (const auto& [key, e]:graph_->edges()) {
    for (auto id:{e.a, e.b}) {
      const auto &p = graph_->nodes().at(id).position;
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = .2;
      lines.points.push_back(point);
    }
  }
  output.markers.push_back(lines);
  marker_pub_->publish(output);
  auto now = node_->now().nanoseconds();
  if (!graph_->nodes().empty() && now - last_publish_ < 1000000000) {
    return;
  }
  last_publish_ = now;
  std::ostringstream json;
  json << "{\"waypoints\":" << graph_->waypointCount() << ",\"frontiers\":" << graph_->frontierCount() << ",\"duplicates\":" << graph_->stats().duplicates << ",\"revisits\":" << graph_->stats().revisits << ",\"nodes\":[";
  bool comma = false;
  for (const auto& [id, n]:graph_->nodes()) {
    if (comma) {
      json << ',';
    }
    comma = true;
    json << "{\"id\":" << id << ",\"type\":\"" << (n.type == NodeType::WAYPOINT ? "WAYPOINT" : "FRONTIER") << "\",\"x\":" << n.position.x << ",\"y\":" << n.position.y << ",\"measured_at_node\":" << (n.measured_at_node ? "true" : "false") << ",\"descriptor_origin\":[" << n.descriptor.origin().x << ',' << n.descriptor.origin().y << "],\"sectors\":[";
    bool sep = false;
    for (const auto &s:n.descriptor.sectors()) {
      if (sep) {
        json << ',';
      }
      sep = true;
      json << "{\"state\":" << int(s.state) << ",\"range\":" << (std::isfinite(s.free_range) ? s.free_range : -1) << ",\"obstacle\":" << (std::isfinite(s.obstacle_distance) ? s.obstacle_distance : -1) << ",\"stamp\":" << s.stamp << ",\"mask\":" << s.valid_mask << '}';
    }
    json << "],\"images\":[";
    sep = false;
    for (const auto& [slot, img]:n.images) {
      if (sep) {
        json << ',';
      }
      sep = true;
      json << "{\"stamp\":" << img.stamp << ",\"yaw\":" << img.yaw << ",\"camera_position\":[" << img.camera_position.x << ',' << img.camera_position.y << ',' << img.camera_z << "],\"bytes\":" << (img.jpeg ? img.jpeg->size() : 0) << '}';
    }
    json << "]}";
  }
  json << "],\"edges\":[";
  comma = false;
  for (const auto& [key, e]:graph_->edges()) {
    if (comma) {
      json << ',';
    }
    comma = true;
    json << "{\"a\":" << e.a << ",\"b\":" << e.b << ",\"weight\":" << e.weight << '}';
  }
  json << "]}";
  std_msgs::msg::String state;
  state.data = json.str();
  state_pub_->publish(state);
}
}
