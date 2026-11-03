#include <topology_exploration/explorer_node.h>
#include <sstream>
namespace apexnav_planner::topology {
void ExplorerNode::visualize(){
  const auto &d = graph_->diagnostics();
  if (state_ != reported_state_) {
    RCLCPP_INFO(get_logger(), "state %s -> %s: %s; W=%zu F=%zu E=%zu target=%lu",
                reported_state_.empty() ? "START" : reported_state_.c_str(), state_.c_str(), reason_.c_str(),
                graph_->waypointCount(), graph_->frontierCount(), graph_->edges().size(),
                route_ ? route_->target : 0UL);
    reported_state_ = state_;
  }
  if (graph_stamp_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "frontiers: anchor=%d open_bins=%lu open/jump=%lu/%lu accepted=%lu detours=%lu reconnected=%lu "
        "filtered(close/unsafe/no_path/duplicate/covered)=%lu/%lu/%lu/%lu/%lu; W=%zu F=%zu E=%zu reason=%s",
        d.at_anchor, d.open_bins, d.open_candidates, d.jump_candidates, d.accepted, d.detours, d.reconnected,
        d.too_close, d.unsafe, d.no_path, d.duplicate, d.covered,
        graph_->waypointCount(), graph_->frontierCount(), graph_->edges().size(), reason_.c_str());
  }
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "world";
  map.header.stamp = now();
  map.info.resolution = map_->resolution();
  map.info.width = map.info.height = map_->width();
  auto origin = map_->origin();
  map.info.origin.position.x = origin.x;
  map.info.origin.position.y = origin.y;
  map.info.origin.orientation.w = 1.;
  map.data.resize(map_->capacity());
  // SDF storage is x-major, OccupancyGrid is y-major.
  for (int x = 0; x < map_->width(); ++x) {
    for (int y = 0; y < map_->width(); ++y) {
      map.data[y * map_->width() + x] = map_->snapshot()[x * map_->width() + y];
    }
  }
  map_pub_->publish(map);
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray markers;
  Marker clear;
  clear.header = map.header;
  clear.action = Marker::DELETEALL;
  markers.markers.push_back(clear);
  for (const auto & [id, n]:graph_->nodes()) {
    Marker m;
    m.header = map.header;
    m.ns = "topology_nodes";
    m.id = int(id);
    m.type = Marker::SPHERE;
    m.pose.orientation.w = 1;
    m.pose.position.x = n.position.x;
    m.pose.position.y = n.position.y;
    m.pose.position.z = .25;
    m.scale.x = m.scale.y = m.scale.z = .22;
    m.color.a = 1.;
    m.lifetime.sec = 2;
    if (n.type == sparse::NodeType::WAYPOINT) {
      m.color.g = 1;
    }
    else{
      m.color.r = 1;
    }
    auto failed = global_.status().find(id);
    if (failed != global_.status().end() &&
        (failed->second.blocked || now().seconds() < failed->second.retry_after)) {
      m.color.g = .45;
      m.color.r = .6;
    }
    markers.markers.push_back(m);
    m.ns = "topology_ids";
    m.type = Marker::TEXT_VIEW_FACING;
    m.scale.z = .18;
    m.pose.position.z = .5;
    m.text = std::to_string(id);
    markers.markers.push_back(m);
  }
  Marker lines;
  lines.header = map.header;
  lines.ns = "topology_edges";
  lines.type = Marker::LINE_LIST;
  lines.pose.orientation.w = 1;
  lines.scale.x = .035;
  lines.color.b = 1.;
  lines.color.a = 1.;
  lines.lifetime.sec = 2;
  for (const auto & [key, e]:graph_->edges()) {
    if (e.verified) {
      auto points = e.path;
      if (points.empty()) {
        points = {graph_->nodes().at(e.a).position, graph_->nodes().at(e.b).position};
      }
      for (size_t i = 1; i < points.size(); ++i) {
        for (auto p:{points[i - 1], points[i]}) {
          geometry_msgs::msg::Point v;
          v.x = p.x;
          v.y = p.y;
          v.z = .2;
          lines.points.push_back(v);
        }
      }
    }
  }
  markers.markers.push_back(lines);
  Marker robot;
  robot.header = map.header;
  robot.ns = "robot";
  robot.type = Marker::ARROW;
  robot.pose.position.x = position_.x;
  robot.pose.position.y = position_.y;
  robot.pose.position.z = .3;
  robot.pose.orientation.z = std::sin(yaw_ / 2);
  robot.pose.orientation.w = std::cos(yaw_ / 2);
  robot.scale.x = .35;
  robot.scale.y = .10;
  robot.scale.z = .10;
  robot.color.b = robot.color.a = 1;
  robot.lifetime.sec = 2;
  markers.markers.push_back(robot);
  graph_pub_->publish(markers);
  auto pathMessage = [&](const std::vector<Vec2> &points){
                       nav_msgs::msg::Path path;
                       path.header = map.header;
                       for (auto p:points) {
                         geometry_msgs::msg::PoseStamped v;
                         v.header = path.header;
                         v.pose.position.x = p.x;
                         v.pose.position.y = p.y;
                         v.pose.position.z = .12;
                         v.pose.orientation.w = 1;
                         path.poses.push_back(v);
                       }
                       return path;
                     };
  global_path_pub_->publish(pathMessage(route_?route_->path:std::vector<Vec2>{}));
  local_path_pub_->publish(pathMessage(local_path_.points));
  std::ostringstream out;
  out << "{\"state\":\"" << state_ << "\",\"reason\":\"" << reason_ << "\",\"waypoints\":" <<
    graph_->waypointCount() << ",\"frontiers\":" << graph_->frontierCount() << ",\"edges\":" <<
    graph_->edges().size() << ",\"converted\":" << graph_->stats().converted << ",\"target\":" <<
    (route_?route_->target:0) << ",\"route_cost\":" << (route_?route_->cost:0) << ",\"map_cells\":" <<
    map_->capacity() << ",\"map_origin\":[" << origin.x << ',' << origin.y << "],\"v\":" << guard_.last().v <<
    ",\"w\":" << guard_.last().w << ",\"created_total\":" << graph_->stats().created <<
    ",\"detours_total\":" << graph_->stats().detours << ",\"duplicate_rejections_total\":" << graph_->stats().duplicates <<
    ",\"candidate_diagnostics\":{\"at_anchor\":" << (d.at_anchor ? "true" : "false") <<
    ",\"open_bins\":" << d.open_bins << ",\"open_candidates\":" << d.open_candidates <<
    ",\"jump_candidates\":" << d.jump_candidates << ",\"too_close\":" << d.too_close <<
    ",\"unsafe\":" << d.unsafe << ",\"no_path\":" << d.no_path << ",\"duplicate\":" << d.duplicate <<
    ",\"covered\":" << d.covered << ",\"accepted\":" << d.accepted << ",\"detours\":" << d.detours <<
    ",\"reconnected\":" << d.reconnected << "},\"nodes\":[";
  bool comma = false;
  for (const auto & [id, n]:graph_->nodes()) {
    if (comma) {
      out << ',';
    }
    comma = true;
    out << "{\"id\":" << id << ",\"type\":\"" <<
      (n.type ==
       sparse::NodeType::WAYPOINT?"WAYPOINT":"FRONTIER") << "\",\"x\":" << n.position.x << ",\"y\":" <<
      n.position.y << "}";
  }
  out << "],\"failed\":[";
  comma = false;
  for (const auto & [id, s]:global_.status()) {
    if (comma) {
      out << ',';
    }
    comma = true;
    out << "{\"id\":" << id << ",\"failures\":" << s.failures << ",\"retry_after\":" << s.retry_after <<
      ",\"blocked\":" << (s.blocked?"true":"false") << '}';
  }
  out << "]}";
  std_msgs::msg::String state;
  state.data = out.str();
  state_pub_->publish(state);
}
}
