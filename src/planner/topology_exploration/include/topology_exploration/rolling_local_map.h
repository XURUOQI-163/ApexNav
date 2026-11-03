#pragma once
#include <plan_env/sdf_map2d.h>
#include <plan_env/topology_graph.h>
#include <vector>
namespace apexnav_planner::topology {
using sparse::Vec2;
enum class Cell {
  UNKNOWN, FREE, OCCUPIED
};
struct Ray {
  Vec2 endpoint;
  bool hit;
};
struct MapConfig {
  double size = 6., resolution = .05, length = .322, width = .220, margin = .05;
};
class RollingLocalMap {
public:
explicit RollingLocalMap(rclcpp::Node::SharedPtr node, MapConfig cfg = {});
void reset(Vec2 center);
void moveTo(Vec2 center);
void seedFootprint(Vec2 position);
void observe(Vec2 origin, const std::vector<Ray> &rays);
Cell occupancy(Vec2 p) const;
bool contains(Vec2 p) const;
bool safe(Vec2 p) const;
bool obstructed(Vec2 p) const;
bool segmentSafe(Vec2 a, Vec2 b) const;
Vec2 origin() const;
Vec2 cellCenter(int index) const;
int index(Vec2 p) const;
int width() const {
  return n_;
}
size_t capacity() const {
  return map_->mp_->buffer_size_;
}
double resolution() const {
  return cfg_.resolution;
}
double radius() const {
  return radius_;
}
const std::vector<int8_t> &snapshot() const {
  return snapshot_;
}
uint64_t revision() const {
  return revision_;
}
private:
void refresh();
MapConfig cfg_;
double radius_;
int n_;
std::shared_ptr<SDFMap2D> map_;
std::vector<int8_t> snapshot_;
uint64_t revision_ = 0;
};
}
