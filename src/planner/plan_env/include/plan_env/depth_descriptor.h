#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace apexnav_planner::sparse {
constexpr double PI = 3.14159265358979323846;
struct Vec2 {
  double x = 0, y = 0;
  Vec2 operator+(Vec2 b) const {
    return {x + b.x, y + b.y};
  }
  Vec2 operator-(Vec2 b) const {
    return {x - b.x, y - b.y};
  }
  Vec2 operator*(double k) const {
    return {x *k, y *k};
  }
  double norm() const {
    return std::hypot(x, y);
  }
  double distance(Vec2 b) const {
    return (*this - b).norm();
  }
};
inline double angle(Vec2 p) {
  return std::atan2(p.y, p.x);
}
inline Vec2 direction(double a) {
  return {std::cos(a), std::sin(a)};
}
inline double wrap(double a) {
  a = std::fmod(a + 2 * PI, 2 * PI);
  return a < 0 ? a + 2 * PI : a;
}
enum class ObservationState {
  UNOBSERVED, INVALID, FREE, OBSTACLE
};
struct DepthSector {
  ObservationState state = ObservationState::UNOBSERVED;
  double obstacle_distance = std::numeric_limits<double>::infinity();
  double free_range = std::numeric_limits<double>::infinity();
  int64_t stamp = 0;
  unsigned valid_mask = 0;
  unsigned valid_samples = 0, invalid_samples = 0;
  bool full() const {
    return valid_mask == 63;
  }
};
class DepthDescriptor {
public:
DepthDescriptor(double sector_degrees = 30, double max_distance = 5, Vec2 origin = {})
  : max_distance_(max_distance), origin_(origin) {
  if (!std::isfinite(sector_degrees) || sector_degrees < 5 || sector_degrees > 90 ||
      std::abs(360 / sector_degrees - std::round(360 / sector_degrees)) > 1e-6 ||
      !std::isfinite(max_distance) || max_distance <= 0) {
    throw std::invalid_argument("descriptor requires sector angle dividing 360 (5..90) and positive range");
  }
  sectors_.resize(std::lround(360 / sector_degrees));
  directions_.resize(sectors_.size() * 6);
  step_ = 2 * PI / sectors_.size();
}
size_t index(double yaw) const {
  return size_t(wrap(yaw + step_ / 2) / step_) % sectors_.size();
}
double center(size_t i) const {
  return i * step_;
}
void addRay(double world_yaw, double range, bool valid, bool obstacle, int64_t stamp) {
  if (!std::isfinite(world_yaw)) {
    return;
  }
  auto i = index(world_yaw);
  auto &s = sectors_[i];
  const double a = wrap(world_yaw + step_ / 2) - i * step_;
  const int sub = std::clamp(int(a / step_ * 6), 0, 5);
  auto &fine = directions_[i * 6 + sub];
  s.stamp = stamp;
  fine.stamp = stamp;
  if (!valid || !std::isfinite(range) || range <= 0) {
    ++s.invalid_samples;
    ++fine.invalid_samples;
    return;
  }
  s.valid_mask |= 1u << sub;
  ++s.valid_samples;
  ++fine.valid_samples;
  const double r = std::min(range, max_distance_);
  s.free_range = std::min(s.free_range, r);
  fine.free_range = std::min(fine.free_range, r);
  if (obstacle && range < max_distance_) {
    s.obstacle_distance = std::min(s.obstacle_distance, range);
    fine.obstacle_distance = std::min(fine.obstacle_distance, range);
  }
}
void finish(int64_t stamp) {
  stamp_ = stamp;
  for (auto *bins : {&sectors_, &directions_}) for (auto &s:*bins) {
    if (std::isfinite(s.obstacle_distance)) {
      s.state = ObservationState::OBSTACLE;
    }
    else if (s.valid_samples) {
      s.state = ObservationState::FREE;
    }
    else if (s.invalid_samples) {
      s.state = ObservationState::INVALID;
    }
  }
}
// Only combine rotations about the same position. Translation is a new view;
// pretending it was measured at the old node would invent free space.
bool merge(const DepthDescriptor &other) {
  if (origin_.distance(other.origin_) > 1e-3 || sectors_.size() != other.sectors_.size()) {
    return false;
  }
  // Fine ranges choose candidates only. Coarse minima and coverage below stay
  // conservative; map footprint checks remain the traversal authority.
  for (size_t i = 0; i < directions_.size(); ++i) {
    const auto &n = other.directions_[i];
    auto &old = directions_[i];
    if (n.state == ObservationState::UNOBSERVED || n.stamp < old.stamp) continue;
    // Each frame already stores its minimum valid range within this small bin.
    // Fresh evidence replaces that sample (including invalidation); otherwise
    // one past short return would close this direction forever. Candidate paths
    // still need independent known-free footprint verification in the map.
    old = n;
  }
  for (size_t i = 0; i < sectors_.size(); ++i) {
    const auto &n = other.sectors_[i];
    auto &old = sectors_[i];
    if (n.state == ObservationState::UNOBSERVED || n.stamp < old.stamp) {
      continue;
    }
    if (n.full() || !old.valid_samples || n.state == ObservationState::INVALID) {
      old = n;
      continue;
    }
    old.valid_mask |= n.valid_mask;
    old.valid_samples += n.valid_samples;
    old.invalid_samples += n.invalid_samples;
    old.stamp = n.stamp;
    old.free_range = std::min(old.free_range, n.free_range);
    old.obstacle_distance = std::min(old.obstacle_distance, n.obstacle_distance);
    old.state = std::isfinite(old.obstacle_distance) ? ObservationState::OBSTACLE : ObservationState::FREE;
  }
  stamp_ = std::max(stamp_, other.stamp_);
  return true;
}
bool covers(Vec2 point, double margin) const {
  const auto delta = point - origin_;
  const double r = delta.norm();
  if (r < 1e-6) {
    return false;
  }
  const double half = std::atan2(margin, r);
  const double a = angle(delta);
  // Require the entire angular window to be measured, including partial-FoV
  // sector boundaries; a forward camera must not require artificial 360 coverage.
  const int samples = std::max(1, int(std::ceil(2 * half / (step_ / 6))));
  for (int i = 0; i <= samples; ++i) {
    const auto &s = sectors_[index(a - half + 2 * half * i / samples)];
    if (!s.full() || !(s.state == ObservationState::FREE || s.state == ObservationState::OBSTACLE) ||
        s.free_range <= r + margin) {
      return false;
    }
  }
  return true;
}
const std::vector<DepthSector>&sectors()const {
  return sectors_;
}
const std::vector<DepthSector>&directions()const { return directions_; }
double directionCenter(size_t i)const { return -step_ / 2 + (i + .5) * step_ / 6; }
Vec2 origin()const {
  return origin_;
}
double step()const {
  return step_;
}
double maxDistance()const {
  return max_distance_;
}
int64_t stamp()const {
  return stamp_;
}
private:
std::vector<DepthSector> sectors_;
std::vector<DepthSector> directions_;
double step_, max_distance_;
Vec2 origin_;
int64_t stamp_ = 0;
};
}
