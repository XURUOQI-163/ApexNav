#pragma once
#include <plan_env/depth_descriptor.h>
#include <array>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

namespace apexnav_planner::sparse {
// Every cell intersecting the circular footprint must be known free. Sampling
// offsets about p misses cells when p is close to a grid boundary.
template<class CellFree>
bool footprintClear(Vec2 p, double radius, double resolution, Vec2 origin, CellFree free) {
  const int x0 = int(std::floor((p.x - radius - origin.x) / resolution));
  const int x1 = int(std::floor((p.x + radius - origin.x) / resolution));
  const int y0 = int(std::floor((p.y - radius - origin.y) / resolution));
  const int y1 = int(std::floor((p.y + radius - origin.y) / resolution));
  for (int x = x0; x <= x1; ++x) {
    for (int y = y0; y <= y1; ++y) {
      const double left = origin.x + x * resolution, bottom = origin.y + y * resolution;
      const double dx = std::max({left - p.x, 0., p.x - left - resolution});
      const double dy = std::max({bottom - p.y, 0., p.y - bottom - resolution});
      if (dx * dx + dy * dy <= radius * radius && !free(x, y)) {
        return false;
      }
    }
  }
  return true;
}
enum class NodeType {
  FRONTIER, WAYPOINT
};
struct DirectionalImage {
  int64_t stamp = 0;
  Vec2 camera_position;
  double camera_z = 0, yaw = 0;
  std::array<double, 4> orientation{0, 0, 0, 1};
  std::shared_ptr<const std::vector<unsigned char> > jpeg;
};
struct TopologyNode {
  uint64_t id = 0;
  NodeType type = NodeType::FRONTIER;
  Vec2 position;
  DepthDescriptor descriptor; // FRONTIER: source-referenced; WAYPOINT: measured here.
  bool measured_at_node = false;
  uint64_t source_id = 0;
  int64_t stamp = 0;
  std::map<size_t, DirectionalImage> images;
  std::map<int64_t, size_t> pending_images; // bounded exact-stamp matches; keep completed views
};
struct TopologyEdge {
  uint64_t a = 0, b = 0;
  double weight = 0;
  bool verified = true;
  int64_t verified_at = 0;
  std::vector<Vec2> path;
  bool traversed = false;
};
struct TopologyConfig {
  double sector_degrees = 30, max_distance = 5, open_distance = 4.5, jump_distance = 1.0;
  double candidate_step = 3.5, min_candidate_distance = 0.8, reach_radius = 0.4;
  double duplicate_radius = 0.7, waypoint_spacing = 1.5, clearance = 0.18, ray_step = 0.025;
  // Local-window exploration needs an observation opportunity around a candidate,
  // not merely an already-seen center. Zero preserves legacy point coverage.
  double coverage_radius = 0;
  bool fine_candidates = false; // Legacy observer retains coarse selection.
  double min_open_angle_degrees = 10;
  int min_open_sectors = 1, max_interval_sectors = 2;
};
struct TopologyStats {
  uint64_t created = 0, converted = 0, duplicates = 0, covered = 0, removed = 0, revisits = 0;
  uint64_t detours = 0;
};
// Last accepted graph update, not lifetime totals. Zero when away from anchor.
struct CandidateDiagnostics {
  uint64_t open_bins = 0, open_candidates = 0, jump_candidates = 0;
  uint64_t too_close = 0, unsafe = 0, no_path = 0, duplicate = 0, covered = 0;
  uint64_t accepted = 0, detours = 0, reconnected = 0;
  bool at_anchor = false;
};
class TopologyGraph {
public:
using Safe = std::function<bool (Vec2)>;
using Connect = std::function<std::vector<Vec2>(Vec2, Vec2)>;
explicit TopologyGraph(TopologyConfig config = {});
void update(Vec2 position, double yaw, int64_t stamp, const DepthDescriptor &frame, const Safe &safe,
            const Safe &blocked = {}, const Connect &connect = {});
void attachImage(const DirectionalImage &image);
uint64_t currentWaypoint() const { return current_; }
bool recordTraversal(uint64_t a, uint64_t b, std::vector<Vec2> path, int64_t stamp);
void invalidateEdge(uint64_t a, uint64_t b);
bool wantsImage(int64_t stamp) const;
void reset();
bool erase(uint64_t id);
const std::map<uint64_t, TopologyNode>&nodes()const {
  return nodes_;
}
const std::map<std::pair<uint64_t, uint64_t>, TopologyEdge>&edges()const {
  return edges_;
}
const TopologyStats&stats()const {
  return stats_;
}
const CandidateDiagnostics&diagnostics()const { return diagnostics_; }
size_t waypointCount()const;
size_t frontierCount()const {
  return nodes_.size() - waypointCount();
}
bool lineSafe(Vec2 a, Vec2 b, const Safe &safe)const;
private:
uint64_t create(NodeType type, Vec2 p, const DepthDescriptor &d, int64_t stamp, uint64_t source);
void storeView(TopologyNode &n, double yaw, int64_t stamp);
std::vector<Vec2> candidates(const DepthDescriptor &d);
TopologyConfig cfg_;
std::map<uint64_t, TopologyNode> nodes_;
std::map<std::pair<uint64_t, uint64_t>, TopologyEdge> edges_;
uint64_t next_id_ = 1, current_ = 0;
int64_t last_stamp_ = 0;
TopologyStats stats_;
CandidateDiagnostics diagnostics_;
};
}
