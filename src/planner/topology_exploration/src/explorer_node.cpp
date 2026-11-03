// OpenCV must precede OSQP: OSQP 0.6 exports a MAX_ITER macro.
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <topology_exploration/explorer_node.h>
#include <plan_env/depth_projection.h>
#include <chrono>
#include <cstdlib>
#include <sstream>
namespace apexnav_planner::topology {
ExplorerNode::ExplorerNode() : Node("topology_explorer"){
}
ExplorerNode::~ExplorerNode() {
  finalize();
}
void ExplorerNode::finalize() {
  saveSnapshot(true);
  if (snapshot_) snapshot_->end();
}
void ExplorerNode::saveSnapshot(bool wait) {
  if (!snapshot_ || !snapshot_->active()) return;
  if (snapshot_job_.valid()) {
    if (!wait && snapshot_job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    try {
      snapshot_job_.get();
    } catch (const std::exception &e) {
      // A previous failed job must not swallow the current/final write attempt.
      RCLCPP_ERROR(get_logger(), "previous topology snapshot failed: %s", e.what());
    }
  }
  try {
    // Copy on the ROS thread: descriptor values are copied, immutable JPEG
    // buffers are shared. At most one disk writer runs; no graph data races.
    snapshot_job_ = std::async(std::launch::async,
        [this, graph = *graph_, stamp = now().nanoseconds(), state = state_]() {
          snapshot_->write(graph, stamp, state);
        });
    if (wait) snapshot_job_.get();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "topology snapshot failed: %s", e.what());
  }
}
void ExplorerNode::initialize(){
  map_cfg_.size = parameter("local_map.size", 6.);
  map_cfg_.resolution = parameter("local_map.resolution", .05);
  map_cfg_.length = parameter("robot.length", .322);
  map_cfg_.width = parameter("robot.width", .220);
  map_cfg_.margin = parameter("robot.margin", .05);
  control_cfg_.max_v = parameter("max_correction_vel", .4);
  control_cfg_.max_w = parameter("max_correction_omega", 1.);
  control_cfg_.accel = parameter("mpc.max_accel", .5);
  control_cfg_.alpha = parameter("mpc.max_angular_accel", 1.);
  if (!parameter("mpc.strict_command_limits", true)) {
    throw std::invalid_argument("TOPOLOGY requires strict MPC command limits");
  }
  dt_ = parameter("mpc.dt", .05);
  horizon_ = parameter("mpc.predict_steps", 20);
  if (dt_ != .05) {
    throw std::invalid_argument("topology control currently requires mpc.dt=0.05 (20Hz)");
  }
  if (parameter("mpc.delay_num", 0) != 0) {
    throw std::invalid_argument("TOPOLOGY requires mpc.delay_num=0");
  }
  parameter("mpc.max_iter", 4);
  parameter("mpc.du_threshold", .001);
  parameter("mpc.matrix_q", std::vector<double>{30., 30., 2.5, 2.5});
  parameter("mpc.matrix_r", std::vector<double>{.1, .01});
  parameter("mpc.matrix_rd", std::vector<double>{.01, .01});
  for (double v:{control_cfg_.max_v, control_cfg_.max_w, control_cfg_.accel, control_cfg_.alpha}) {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("invalid chassis limit");
    }
  }
  guard_ = ControllerGuard(control_cfg_);
  mpc_.init(shared_from_this());
  map_ = std::make_unique<RollingLocalMap>(shared_from_this(), map_cfg_);
  sparse::TopologyConfig graph_cfg;
  graph_cfg.sector_degrees = parameter("sparse_topology.sector_degrees", 30.);
  graph_cfg.max_distance = parameter("sparse_topology.max_distance", 5.);
  graph_cfg.open_distance = parameter("sparse_topology.open_distance", 1.2);
  graph_cfg.jump_distance = parameter("sparse_topology.jump_distance", 1.);
  graph_cfg.fine_candidates = parameter("sparse_topology.fine_candidates", true);
  graph_cfg.min_open_angle_degrees = parameter("sparse_topology.min_open_angle_degrees", 10.);
  graph_cfg.candidate_step = parameter("sparse_topology.candidate_step", std::min(2.2, map_cfg_.size * .4));
  graph_cfg.min_candidate_distance = parameter("sparse_topology.min_candidate_distance", .8);
  if (graph_cfg.candidate_step > map_cfg_.size * .5 - map_->radius()) {
    throw std::invalid_argument("candidate_step must fit inside the local window with footprint clearance");
  }
  graph_cfg.clearance = map_->radius();
  graph_cfg.reach_radius = .35;
  // A controlled step is shorter than the local window. Let its FRONTIER become
  // the next anchor instead of inserting an extra waypoint halfway through it.
  graph_cfg.waypoint_spacing = graph_cfg.candidate_step + 2 * graph_cfg.reach_radius;
  graph_cfg.coverage_radius = graph_cfg.candidate_step + graph_cfg.clearance;
  graph_ = std::make_unique<sparse::TopologyGraph>(graph_cfg);
  const char *snapshot_env = std::getenv("APEXNAV_TOPOLOGY_SNAPSHOT_DIR");
  snapshot_ = std::make_unique<SnapshotWriter>(parameter<std::string>("snapshot.directory",
      snapshot_env ? snapshot_env : std::string(APEXNAV_PROJECT_ROOT) + "/topology_snapshot"));
  double snapshot_interval = parameter("snapshot.interval", 1.);
  if (!std::isfinite(snapshot_interval) || snapshot_interval < .1)
    throw std::invalid_argument("snapshot.interval must be finite and at least 0.1s");
  RCLCPP_INFO(get_logger(), "frontier generation: open=%.2fm max=%.2fm step=%.2fm fine=%s angle=%.1fdeg",
              graph_cfg.open_distance, graph_cfg.max_distance, graph_cfg.candidate_step,
              graph_cfg.fine_candidates ? "true" : "false", graph_cfg.min_open_angle_degrees);
  fx_ = parameter("camera.fx", fx_);
  fy_ = parameter("camera.fy", fy_);
  cx_ = parameter("camera.cx", cx_);
  cy_ = parameter("camera.cy", cy_);
  sensor_max_ = parameter("camera.max_depth", sensor_max_);
  min_height_ = parameter("collision.min_height", min_height_);
  max_height_ = parameter("collision.max_height", max_height_);
  depth_timeout_ = parameter("safety.depth_timeout", depth_timeout_);
  odom_timeout_ = parameter("safety.odom_timeout", odom_timeout_);
  for (double v:{fx_, fy_, cx_, cy_, sensor_max_, min_height_, max_height_, depth_timeout_, odom_timeout_}) {
    if (!std::isfinite(v)) {
      throw std::invalid_argument("nonfinite sensor/safety parameter");
    }
  }
  if (fx_ <= 0 || fy_ <= 0 || sensor_max_ <= 0 || min_height_ < 0 || max_height_ <= min_height_ ||
      depth_timeout_ <= 0 || odom_timeout_ <= 0) {
    throw std::invalid_argument("invalid sensor/safety parameters");
  }
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/topology/local_map", rclcpp::QoS(
                                                              1).transient_local());
  graph_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/sparse_topology/markers", rclcpp::QoS(
                                                             1).transient_local());
  state_pub_ = create_publisher<std_msgs::msg::String>("/topology/state", rclcpp::QoS(1).transient_local());
  global_path_pub_ = create_publisher<nav_msgs::msg::Path>("/topology/global_path", 1);
  local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/topology/local_path", 1);
  odom_sub_ = create_subscription<Odom>("/habitat/odom", rclcpp::SensorDataQoS(), [this](Odom::SharedPtr m){
      odom(m);
    });
  rgb_sub_ =
    create_subscription<Image>("/habitat/camera_rgb", rclcpp::SensorDataQoS(), [this](
                                 Image::ConstSharedPtr m){
      rgb(m);
    });
  depth_sub_ = std::make_shared<message_filters::Subscriber<Image> >(this, "/habitat/camera_depth",
                                                                     rmw_qos_profile_sensor_data);
  pose_sub_ = std::make_shared<message_filters::Subscriber<Odom> >(this, "/habitat/sensor_pose",
                                                                   rmw_qos_profile_sensor_data);
  sync_ = std::make_shared<message_filters::Synchronizer<Sync> >(Sync(20), *depth_sub_, *pose_sub_);
  sync_->registerCallback(std::bind(&ExplorerNode::depth, this, std::placeholders::_1,
                                    std::placeholders::_2));
  stop_sub_ =
    create_subscription<std_msgs::msg::Empty>("/traj_server/stop", 10, [this](
                                                std_msgs::msg::Empty::SharedPtr){
      external_stop_ = true;
      state_ = "STOPPED";
      stop("external stop");
      saveSnapshot(true);
    });
  reset_service_ =
    create_service<std_srvs::srv::Trigger>("/exploration/reset",
                                           [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                                  std_srvs::srv::Trigger::Response::SharedPtr r){
      try {
        reset();
        r->success = true;
        r->message = "topology, local map, controller and snapshot reset: " + snapshot_->directory().string();
      } catch (const std::exception &e) {
        r->success = false;
        r->message = e.what();
        RCLCPP_ERROR(get_logger(), "episode reset failed: %s", e.what());
      }
    });
  controller_reset_service_ =
    create_service<std_srvs::srv::Trigger>("/traj_server/reset",
                                           [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                                                  std_srvs::srv::Trigger::Response::SharedPtr r){
      guard_.stop();
      mpc_.reset();
      publishCommand({});
      r->success = true;
    });
  // Constructing/restarting the planner is not a new simulation.
  reset(false);
  control_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      tick();
    });
  vis_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {
      visualize();
    });
  snapshot_timer_ = create_wall_timer(std::chrono::duration<double>(snapshot_interval), [this] {
      // Late RGB/depth may arrive after the stop message on another topic.
      // Saving depends on the episode lifetime, not controller data freshness.
      saveSnapshot();
    });
}
void ExplorerNode::reset(bool new_simulation){
  // Reset is an entry boundary only. Stop and join the old writer before
  // clearing both the in-memory graph and its on-disk snapshot.
  guard_.stop();
  mpc_.reset();
  publishCommand({});
  saveSnapshot(true);
  if (snapshot_) snapshot_->end();
  reset_stamp_ = now().nanoseconds();
  odom_stamp_ = depth_stamp_ = graph_stamp_ = 0;
  have_odom_ = seeded_ = external_stop_ = localization_fault_ = false;
  observing_ = true;
  observe_turn_ = 0;
  trace_anchor_ = 0;
  trace_.clear();
  rgb_cache_.clear();
  views_.clear();
  route_.reset();
  local_path_ = {};
  global_.reset();
  graph_->reset();
  map_->reset({0, 0});
  guard_.stop();
  mpc_.reset();
  state_ = "WAIT_DATA";
  reason_ = "episode reset";
  publishCommand({});
  visualize();
  if (new_simulation) {
    snapshot_->begin(now().nanoseconds());
    RCLCPP_INFO(get_logger(), "new simulation snapshot: %s", snapshot_->directory().c_str());
  }
}
void ExplorerNode::odom(const Odom::SharedPtr msg){
  int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
  double age = (now().nanoseconds() - stamp) * 1e-9;
  if (stamp <= reset_stamp_ || stamp <= odom_stamp_ || age < -.1 || age > odom_timeout_ ||
      msg->header.frame_id != "world") {
    return;
  }
  const auto &p = msg->pose.pose.position;
  const auto &q = msg->pose.pose.orientation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(msg->twist.twist.linear.x) ||
      !std::isfinite(
        msg->twist.twist.angular.z) || !quat.coeffs().allFinite() || quat.norm() < .9 || quat.norm() > 1.1) {
    localization_fault_ = true;
    stop("invalid localization");
    return;
  }
  double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  Vec2 next{p.x, p.y};
  if (have_odom_) {
    double dt = (stamp - odom_stamp_) * 1e-9;
    if (next.distance(position_) > .15 + 2 * control_cfg_.max_v * dt ||
        std::abs(std::remainder(yaw - yaw_, 2 * sparse::PI)) > .2 + 2 * control_cfg_.max_w * dt) {
      localization_fault_ = true;
      stop("localization jump; reset required");
      return;
    }
    if (observing_) {
      observe_turn_ += std::abs(std::remainder(yaw - yaw_, 2 * sparse::PI));
    }
  }
  position_ = next;
  yaw_ = yaw;
  speed_ = msg->twist.twist.linear.x;
  angular_ = msg->twist.twist.angular.z;
  have_odom_ = true;
  odom_stamp_ = stamp;
  if (!trace_.empty() && trace_.back().distance(position_) > .025) {
    trace_.push_back(position_);
  }
  if (trace_.size() > 2000) {
    trace_.clear();
    trace_anchor_ = 0;
  }
}
void ExplorerNode::depth(const Image::ConstSharedPtr &image, const Odom::ConstSharedPtr &pose){
  int64_t stamp = rclcpp::Time(image->header.stamp).nanoseconds();
  double age = (now().nanoseconds() - stamp) * 1e-9;
  if (!have_odom_ || localization_fault_ || stamp <= reset_stamp_ || stamp <= depth_stamp_ || age < -.1 ||
      age > depth_timeout_ || image->header.frame_id != "world" || pose->header.frame_id != "world") {
    return;
  }
  if (depth_stamp_ && stamp - depth_stamp_ < 100000000) {
    return;                                             // at most 10 Hz mapping
  }
  if (std::abs(stamp - odom_stamp_) > int64_t(odom_timeout_ * 1e9)) {
    return;
  }
  const auto &p = pose->pose.pose.position;
  const auto &q = pose->pose.pose.orientation;
  Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !quat.coeffs().allFinite() ||
      quat.norm() < .9 || quat.norm() > 1.1) {
    return;
  }
  Eigen::Vector3d camera(p.x, p.y, p.z);
  if (Vec2{p.x, p.y}.distance(position_) > .5) {
    return;
  }
  auto rotation = quat.normalized().toRotationMatrix();
  camera_yaw_ = std::atan2(rotation(1, 2), rotation(0, 2));
  cv::Mat raw;
  try{
    raw = cv_bridge::toCvShare(image)->image;
  }
  catch (const std::exception &) {
    return;
  }
  if (raw.type() != CV_32FC1 && raw.type() != CV_16UC1) {
    return;
  }
  sparse::DepthDescriptor descriptor(get_parameter("sparse_topology.sector_degrees").as_double(),
                                     get_parameter("sparse_topology.max_distance").as_double(), {p.x, p.y});
  std::map<int, Ray> rays;
  for (int v = 2; v < raw.rows - 2; v += 4) {
    for (int u = 2; u < raw.cols - 2; u += 4) {
      double value = raw.type() == CV_32FC1?raw.at<float>(v, u):raw.at<uint16_t>(v, u) / 65535.;
      Eigen::Vector3d ray = rotation * Eigen::Vector3d((u - cx_) / fx_, (v - cy_) / fy_, 1.);
      double bearing = std::atan2(ray.y(), ray.x());
      if (!std::isfinite(value) || value <= 0 || value > 1) {
        if (std::abs(ray.z()) < .15 * ray.head<2>().norm()) {
          descriptor.addRay(bearing, 0, false, false, stamp);
        }
        continue;
      }
      Eigen::Vector3d world = projectDepthPixel(u, v, value * sensor_max_, fx_, fy_, cx_, cy_, rotation,
                                                camera);
      if (world.z() < min_height_ || world.z() > max_height_) {
        continue;
      }
      double distance = (world - camera).head<2>().norm();
      bool hit = value < .999;
      descriptor.addRay(bearing, distance, true, hit, stamp);
      int slot = int(sparse::wrap(bearing) * 720 / sparse::PI);
      Ray sample{{world.x(), world.y()}, hit};
      auto old = rays.find(slot);
      if (old == rays.end() || Vec2{p.x, p.y}.distance(old->second.endpoint) > distance) {
        rays[slot] = sample;
      }
    }
  }
  // An invalid-only frame cannot refresh the watchdog or clear obstacles.
  if (rays.empty()) {
    return;
  }
  map_->moveTo(position_);
  if (!seeded_) {
    map_->seedFootprint(position_);
    seeded_ = true;
  }
  std::vector<Ray> samples;
  for (const auto & [slot, ray]:rays) {
    samples.push_back(ray);
  }
  map_->observe({p.x, p.y}, samples);
  depth_stamp_ = stamp;
  descriptor.finish(stamp);
  if (stamp - graph_stamp_ >= 250000000) {
    // Habitat's camera and base share x/y. Anchor this measurement at its own
    // exact-stamp pose, not at a newer odometry callback's arrival-time pose.
    graph_->update({p.x, p.y}, camera_yaw_, stamp, descriptor, [this](Vec2 v){
        return map_->safe(v);
      }, [this](Vec2 v){
        return map_->obstructed(v);
      }, [this](Vec2 a, Vec2 b){
        auto path = local_.plan(grid(), a, b);
        return path.complete ? path.points : std::vector<Vec2>{};
      });
    graph_stamp_ = stamp;
    sparse::DirectionalImage view;
    view.stamp = stamp;
    view.camera_position = {p.x, p.y};
    view.camera_z = p.z;
    view.yaw = camera_yaw_;
    view.orientation = {q.x, q.y, q.z, q.w};
    views_[stamp] = view;
    while (views_.size() > 20) {
      views_.erase(views_.begin());
    }
    attach(stamp);
    auto current = graph_->currentWaypoint();
    if (current && current != trace_anchor_) {
      if (trace_anchor_ && graph_->nodes().count(trace_anchor_) && trace_.size() > 1 &&
          graph_->nodes().at(current).position.distance(position_) < .4) {
        trace_.push_back(position_);
        graph_->recordTraversal(trace_anchor_, current, trace_, stamp);
      }
      trace_anchor_ = current;
      trace_ = {graph_->nodes().at(current).position, position_};
    }
  }
}
void ExplorerNode::rgb(const Image::ConstSharedPtr &image){
  int64_t stamp = rclcpp::Time(image->header.stamp).nanoseconds();
  if (stamp <= reset_stamp_ || image->header.frame_id != "world") {
    return;
  }
  rgb_cache_[stamp] = image;
  while (rgb_cache_.size() > 20) {
    rgb_cache_.erase(rgb_cache_.begin());
  }
  attach(stamp);
}
void ExplorerNode::attach(int64_t stamp){
  if (!views_.count(stamp) || !rgb_cache_.count(stamp) || !graph_->wantsImage(stamp)) {
    return;
  }
  try{
    auto rgb = cv_bridge::toCvCopy(rgb_cache_.at(stamp), "bgr8")->image;
    cv::Mat small;
    cv::resize(rgb, small, cv::Size(320, std::max(1, rgb.rows * 320 / rgb.cols)));
    auto bytes = std::make_shared<std::vector<unsigned char> >();
    cv::imencode(".jpg", small, *bytes, {cv::IMWRITE_JPEG_QUALITY, 75});
    auto view = views_.at(stamp);
    view.jpeg = bytes;
    graph_->attachImage(view);
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "topology image: %s", e.what());
  }
}
bool ExplorerNode::fresh()const {
  int64_t t = get_clock()->now().nanoseconds();
  return have_odom_ && depth_stamp_ > reset_stamp_ && !localization_fault_ && !external_stop_ &&
         t - odom_stamp_ <= int64_t(odom_timeout_ * 1e9) && t - depth_stamp_ <= int64_t(depth_timeout_ * 1e9);
}
GridView ExplorerNode::grid()const {
  return {map_->width(), map_->resolution(), map_->origin(), [this](Vec2 p){
            return map_->safe(p);
          }
  };
}
void ExplorerNode::publishCommand(Command c){
  geometry_msgs::msg::Twist out;
  out.linear.x = c.v;
  out.angular.z = c.w;
  cmd_pub_->publish(out);
}
void ExplorerNode::stop(const std::string &reason){
  reason_ = reason;
  guard_.stop();
  mpc_.reset();
  publishCommand({});
}
void ExplorerNode::startObserve(){
  observing_ = true;
  observe_turn_ = 0;
  route_.reset();
  local_path_ = {};
  state_ = "OBSERVE";
  stop("collect directional observations");
}
void ExplorerNode::failGoal(const std::string &reason){
  if (route_) {
    global_.fail(route_->target, now().seconds());
  }
  route_.reset();
  local_path_ = {};
  state_ = "PLAN";
  stop(reason);
}
std::vector<Connection> ExplorerNode::connections(){
  std::vector<std::pair<double, uint64_t> > nearby;
  for (const auto & [id, n]:graph_->nodes()) {
    if (n.type == sparse::NodeType::WAYPOINT && map_->safe(n.position)) {
      nearby.push_back({position_.distance(n.position), id});
    }
  }
  std::sort(nearby.begin(), nearby.end());
  std::vector<Connection> result;
  for (size_t i = 0; i < std::min(size_t(5), nearby.size()); ++i) {
    auto path = local_.plan(grid(), position_, graph_->nodes().at(nearby[i].second).position);
    if (path.complete) {
      result.push_back({nearby[i].second, path.cost, path.points});
    }
  }
  return result;
}
std::vector<Vec2> ExplorerNode::remainingRoute(){
  if (!route_ || route_->path.empty()) {
    return {};
  }
  return LocalPlanner::nextLeg(route_->path, position_, route_cursor_);
}
void ExplorerNode::plan(){
  last_plan_ = now().seconds();
  if (route_) {
    auto target = graph_->nodes().find(route_->target);
    if (target == graph_->nodes().end()) {
      failGoal("frontier removed by new observations");
    }
    else if (target->second.type == sparse::NodeType::WAYPOINT) {
      startObserve();
      return;
    }
    else{
      for (size_t i = 1; i < route_->nodes.size(); ++i) {
        auto it = graph_->edges().find(std::minmax(route_->nodes[i - 1], route_->nodes[i]));
        if (it == graph_->edges().end() || !it->second.verified) {
          failGoal("topological edge invalidated");
          break;
        }
      }
    }
  }
  if (!route_) {
    route_ = global_.choose(graph_->nodes(), graph_->edges(), connections(), now().seconds());
    route_cursor_ = 1;
    progress_time_ = now().seconds();
    progress_position_ = position_;
    if (!route_) {
      state_ = "NO_REACHABLE_FRONTIER";
      local_path_ = {};
      stop("waiting for reachable frontier");
      return;
    }
  }
  auto rest = remainingRoute();
  Vec2 goal = position_;
  // Choose along the ordered edge geometry until its first window exit. A local
  // A* may detour around a newly seen obstacle to that in-window subgoal.
  for (size_t i = 1; i < rest.size(); ++i) {
    Vec2 a = rest[i - 1], b = rest[i];
    int n = std::max(1, int(std::ceil(a.distance(b) / .05)));
    bool exit = false;
    for (int j = 1; j <= n; ++j) {
      auto p = a + (b - a) * (double(j) / n);
      if (!map_->contains(p)) {
        exit = true;
        break;
      }
      if (map_->safe(p)) {
        goal = p;
      }
    }
    if (exit) {
      break;
    }
  }
  local_path_ = local_.plan(grid(), position_, goal);
  if (!local_path_.complete || local_path_.cost < .1) {
    auto prefix = local_.orderedPrefix(grid(), position_, rest);
    local_path_ = {};
    if (prefix.size() > 1 && position_.distance(prefix.back()) > .15) {
      local_path_ = local_.plan(grid(), position_, prefix.back());
    }
  }
  if (local_path_.points.size() < 2 || local_path_.cost < .1) {
    state_ = "WAIT_SAFE_PATH";
    stop("no safe local prefix");
  }
  else{
    state_ = "TRACK";
  }
  if (position_.distance(progress_position_) > .1) {
    progress_time_ = now().seconds();
    progress_position_ = position_;
  }
  if (now().seconds() - progress_time_ > 8.) {
    failGoal("no progress; frontier in cooldown");
  }
}
std::vector<Eigen::Vector3d> ExplorerNode::reference(const std::vector<Vec2> &path)const {
  std::vector<Eigen::Vector3d> ref(horizon_, Eigen::Vector3d(position_.x, position_.y, yaw_));
  if (path.size() < 2) {
    return ref;
  }
  Vec2 d = path[1] - position_;
  double target_yaw = std::atan2(d.y, d.x);
  if (std::abs(std::remainder(target_yaw - yaw_, 2 * sparse::PI)) > .6) {
    for (auto &r:ref) {
      r.z() = target_yaw;
    }
    return ref;
  }
  double total = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    total += path[i - 1].distance(path[i]);
  }
  double velocity =
    std::min(control_cfg_.max_v, std::sqrt(std::max(0., 2 * control_cfg_.accel * (total - .12))));
  for (int k = 0; k < horizon_; ++k) {
    double travel = std::min(total, (k + 1) * dt_ * velocity);
    Vec2 p = path.front();
    double angle = target_yaw;
    for (size_t i = 1; i < path.size(); ++i) {
      Vec2 v = path[i] - path[i - 1];
      double l = v.norm();
      if (l < 1e-6) {
        continue;
      }
      angle = std::atan2(v.y, v.x);
      if (travel <= l) {
        p = path[i - 1] + v * (travel / l);
        break;
      }
      travel -= l;
      p = path[i];
    }
    ref[k] = {p.x, p.y, angle};
  }
  return ref;
}
void ExplorerNode::tick(){
  if (count_publishers("/cmd_vel") > 1) {
    state_ = "STOPPED";
    stop("multiple cmd_vel publishers; stop the other planner");
    return;
  }
  if (!fresh()) {
    state_ = "WAIT_DATA";
    stop(localization_fault_?"localization invalid; reset required":"stale depth/odometry or external stop");
    return;
  }
  if (!map_->safe(position_)) {
    state_ = "STOPPED";
    stop("current footprint obstructed/unknown");
    return;
  }
  if (observing_ && observe_turn_ >= 2 * sparse::PI - .2) {
    observing_ = false;
    state_ = "PLAN";
    stop("observation complete");
  }
  if (!observing_ && now().seconds() - last_plan_ > .2) {
    plan();
  }
  std::vector<Eigen::Vector3d> ref;
  if (observing_) {
    state_ = "OBSERVE";
    ref.assign(horizon_, {position_.x, position_.y, yaw_ + .8});
  }
  else {
    if (state_ != "TRACK" || local_path_.points.size() < 2) {
      stop(reason_);
      return;
    }
    // Replan begins at current pose; between updates, do not follow stale starting points.
    auto path = local_path_.points;
    path.front() = position_;
    if (!LocalPlanner::segmentSafe(grid(), position_, path[1])) {
      local_path_ = {};
      stop("new obstacle on reference");
      last_plan_ = 0;
      return;
    }
    ref = reference(path);
  }
  mpc_.setOdom({position_.x, position_.y, yaw_, speed_});
  mpc_.setPreviousCommand({guard_.last().v, guard_.last().w});
  auto cmd = mpc_.calCmd(ref);
  if (!mpc_.healthy()) {
    stop("MPC solve failed");
    return;
  }
  Command requested{cmd.x(), cmd.y()};
  if (observing_) {
    requested.v = 0;
  }
  auto safe = guard_.limitAndCheck(requested, position_, yaw_, dt_, [this](Vec2 p){
      return map_->safe(p);
    }, fresh());
  if (safe.v == 0 && safe.w == 0 && (requested.v > .001 || std::abs(requested.w) > .001)) {
    reason_ = "control/braking sweep rejected";
  }
  else{
    reason_ = observing_?"observing": "following safe local reference";
  }
  publishCommand(safe);
}
}
int main(int argc, char * *argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<apexnav_planner::topology::ExplorerNode>();
  try{
    node->initialize();
    rclcpp::spin(node);
    // MPC retains a shared node reference: do not rely on the node destructor
    // to flush pending writes when the process receives SIGINT/SIGTERM.
    node->finalize();
  }
  catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "topology: %s", e.what());
    node->finalize();
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
