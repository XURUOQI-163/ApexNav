#include <topology_exploration/rolling_local_map.h>
#include <plan_env/raycast2d.h>
#include <algorithm>
#include <unordered_set>
namespace apexnav_planner::topology {
RollingLocalMap::RollingLocalMap(rclcpp::Node::SharedPtr node, MapConfig cfg) : cfg_(cfg){
  for (double v:{cfg.size, cfg.resolution, cfg.length, cfg.width}) {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("invalid rolling-map dimension");
    }
  }
  if (!std::isfinite(cfg.margin) || cfg.margin < 0 || cfg.size / cfg.resolution > 1000) {
    throw std::invalid_argument("invalid rolling-map clearance/capacity");
  }
  radius_ = std::hypot(cfg.length, cfg.width) * .5 + cfg.margin;
  if (cfg.size < 4 * radius_) {
    throw std::invalid_argument("window too small for footprint");
  }
  auto param = [&](const std::string &name, auto value){
                 if (!node->has_parameter(name)) {
                   node->declare_parameter(name, value);
                 }
               };
  param("sdf_map.resolution", cfg.resolution);
  param("sdf_map.map_size_x", cfg.size);
  param("sdf_map.map_size_y", cfg.size);
  param("sdf_map.obstacles_inflation", radius_);
  param("sdf_map.local_bound", cfg.size * .5);
  param("sdf_map.max_ray_length", 4.99);
  param("is_real_world", true);
  param("sdf_map.optimistic", false);
  map_ = std::make_shared<SDFMap2D>();
  map_->initMap(node, false);
  n_ = map_->mp_->map_voxel_num_.x();
  reset({0, 0});
}
Vec2 RollingLocalMap::origin()const {
  return {map_->mp_->map_origin_.x(), map_->mp_->map_origin_.y()};
}
bool RollingLocalMap::contains(Vec2 p)const {
  return std::isfinite(p.x) && std::isfinite(p.y) && map_->isInMap(Eigen::Vector2d(p.x, p.y));
}
int RollingLocalMap::index(Vec2 p)const {
  if (!contains(p)) {
    return -1;
  }
  Eigen::Vector2i i;
  map_->posToIndex({p.x, p.y}, i);
  return i.x() * n_ + i.y();
}
Vec2 RollingLocalMap::cellCenter(int i)const {
  Eigen::Vector2d p;
  map_->indexToPos({i / n_, i % n_}, p);
  return {p.x(), p.y()};
}
Cell RollingLocalMap::occupancy(Vec2 p)const {
  if (!contains(p)) {
    return Cell::UNKNOWN;
  }
  int s = map_->getOccupancy(Eigen::Vector2d(p.x, p.y));
  return s == SDFMap2D::FREE?Cell::FREE:s == SDFMap2D::OCCUPIED?Cell::OCCUPIED:Cell::UNKNOWN;
}
void RollingLocalMap::reset(Vec2 center){
  std::fill(map_->md_->occupancy_buffer_.begin(),
            map_->md_->occupancy_buffer_.end(), map_->mp_->clamp_min_log_ - map_->mp_->unknown_flag_);
  moveTo(center);
  refresh();
}
void RollingLocalMap::moveTo(Vec2 center){
  auto &p = *map_->mp_;
  auto &d = *map_->md_;
  Eigen::Vector2d next(std::floor(center.x / cfg_.resolution) * cfg_.resolution - p.map_size_.x() / 2,
                       std::floor(center.y / cfg_.resolution) * cfg_.resolution - p.map_size_.y() / 2);
  Eigen::Vector2i shift = ((next - p.map_origin_) / cfg_.resolution).array().round().cast<int>();
  if (shift.isZero()) {
    return;
  }
  std::vector<double> buffer(capacity(), p.clamp_min_log_ - p.unknown_flag_);
  for (int x = 0; x < n_; ++x) {
    for (int y = 0; y < n_; ++y) {
      int ox = x + shift.x(), oy = y + shift.y();
      if (ox >= 0 && oy >= 0 && ox < n_ && oy < n_) {
        buffer[x * n_ + y] = d.occupancy_buffer_[ox * n_ + oy];
      }
    }
  }
  d.occupancy_buffer_.swap(buffer);
  p.map_origin_ = next;
  p.map_min_boundary_ = next;
  p.map_max_boundary_ = next + p.map_size_;
  std::fill(d.count_hit_.begin(), d.count_hit_.end(), 0);
  std::fill(d.count_miss_.begin(), d.count_miss_.end(), 0);
  std::fill(d.flag_rayend_.begin(), d.flag_rayend_.end(), -1);
  std::fill(d.virtual_ground_buffer_.begin(), d.virtual_ground_buffer_.end(), 0);
  std::fill(d.distance_buffer_.begin(), d.distance_buffer_.end(), 0);
  std::fill(d.distance_buffer_neg_.begin(), d.distance_buffer_neg_.end(), 0);
  d.cache_voxel_ = {};
  d.occupancy_need_clear_.clear();
  map_->caster_->setParams(cfg_.resolution, next);
  refresh();
}
void RollingLocalMap::seedFootprint(Vec2 pos){
  // A collision-free initial pose supplies only the space already occupied by the
  // robot. Never erase a measured obstacle or seed a route in front of the robot.
  sparse::footprintClear(pos, radius_, cfg_.resolution, origin(), [&](int x, int y){
      Eigen::Vector2i i(x, y);
      if (map_->isInMap(i) && map_->getOccupancy(i) == SDFMap2D::UNKNOWN) {
        map_->md_->occupancy_buffer_[map_->toAddress(i)] = map_->mp_->clamp_min_log_;
      }
      return true;
    });
  refresh();
}
void RollingLocalMap::observe(Vec2 source, const std::vector<Ray> &rays){
  if (!contains(source)) {
    return;
  }
  auto &p = *map_->mp_;
  auto &d = *map_->md_;
  std::unordered_set<int> hits, free;
  for (const auto &r:rays) {
    if (r.hit && contains(r.endpoint)) {
      hits.insert(index(r.endpoint));
    }
  }
  RayCaster2D caster;
  caster.setParams(cfg_.resolution, p.map_origin_);
  for (const auto &ray:rays) {
    if (!std::isfinite(ray.endpoint.x) || !std::isfinite(ray.endpoint.y)) {
      continue;
    }
    Vec2 endpoint = ray.endpoint;
    if (!contains(endpoint)) {
      Eigen::Vector2d end = map_->closetPointInMap({endpoint.x, endpoint.y}, {source.x, source.y});
      endpoint = {end.x(), end.y()};
    }
    if (!contains(endpoint)) {
      continue;
    }
    Eigen::Vector2i i;
    caster.input({source.x, source.y}, {endpoint.x, endpoint.y});
    int limit = 2 * n_ + 2;
    while (limit-- > 0) {
      bool more = caster.nextId(i);
      if (!map_->isInMap(i)) {
        break;
      }
      int a = map_->toAddress(i);
      if (hits.count(a)) {
        break;
      }
      free.insert(a);
      if (!more) {
        break;
      }
    }
  }
  for (int a:free) {
    map_->setCacheOccupancy(a, 0);
  }
  for (int a:hits) {
    map_->setCacheOccupancy(a, 1);
  }
  while (!d.cache_voxel_.empty()) {
    int a = d.cache_voxel_.front();
    d.cache_voxel_.pop();
    if (d.occupancy_buffer_[a] < p.clamp_min_log_ - 1e-3) {
      d.occupancy_buffer_[a] = p.min_occupancy_log_;
    }
    // A fresh hit is immediate collision evidence, even after many free scans.
    // Keep probabilistic clearing, but do not delay execution safety by 4 frames.
    if (d.count_hit_[a] > 0) {
      d.occupancy_buffer_[a] = std::max(d.occupancy_buffer_[a], p.min_occupancy_log_);
    }
    double delta = d.count_hit_[a] >= d.count_miss_[a]?p.prob_hit_log_:p.prob_miss_log_;
    d.occupancy_buffer_[a] = std::clamp(d.occupancy_buffer_[a] + delta, p.clamp_min_log_, p.clamp_max_log_);
    d.count_hit_[a] = d.count_miss_[a] = 0;
  }
  refresh();
}
bool RollingLocalMap::safe(Vec2 p)const {
  if (!contains(p)) {
    return false;
  }
  return sparse::footprintClear(p, radius_, cfg_.resolution, origin(), [&](int x, int y){
      Eigen::Vector2i i(x, y);
      return map_->isInMap(i) && map_->getOccupancy(i) == SDFMap2D::FREE;
    });
}
bool RollingLocalMap::obstructed(Vec2 p)const {
  bool hit = false;
  sparse::footprintClear(p, radius_, cfg_.resolution, origin(), [&](int x, int y){
      Eigen::Vector2i i(x, y);
      if (map_->isInMap(i) && map_->getOccupancy(i) == SDFMap2D::OCCUPIED) {
        hit = true;
      }
      return !hit;
    });
  return hit;
}
bool RollingLocalMap::segmentSafe(Vec2 a, Vec2 b)const {
  int n = std::max(1, int(std::ceil(a.distance(b) / (cfg_.resolution * .5))));
  for (int i = 0; i <= n; ++i) {
    if (!safe(a + (b - a) * (double(i) / n))) {
      return false;
    }
  }
  return true;
}
void RollingLocalMap::refresh(){
  snapshot_.resize(capacity());
  std::fill(map_->md_->occupancy_buffer_inflate_.begin(), map_->md_->occupancy_buffer_inflate_.end(), 0);
  for (int a = 0; a < int(capacity()); ++a) {
    auto s = occupancy(cellCenter(a));
    snapshot_[a] = s == Cell::UNKNOWN?-1:s == Cell::OCCUPIED?100:0;
  }
  ++revision_;
}
}
