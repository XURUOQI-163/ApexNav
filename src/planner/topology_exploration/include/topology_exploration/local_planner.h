#pragma once
#include <plan_env/depth_descriptor.h>
#include <functional>
namespace apexnav_planner::topology {
using sparse::Vec2;
struct GridView {
  int n;
  double resolution;
  Vec2 origin;
  std::function<bool(Vec2)> safe;
};
struct LocalPath {
  bool complete = false;
  double cost = 0;
  std::vector<Vec2> points;
};
class LocalPlanner {
public:
LocalPath plan(const GridView &grid, Vec2 start, Vec2 goal)const;
std::vector<Vec2> orderedPrefix(const GridView &grid, Vec2 start, const std::vector<Vec2> &route)const;
static bool segmentSafe(const GridView &grid, Vec2 a, Vec2 b);
static std::vector<Vec2> nextLeg(const std::vector<Vec2> &route, Vec2 start, size_t &cursor);
};
}
