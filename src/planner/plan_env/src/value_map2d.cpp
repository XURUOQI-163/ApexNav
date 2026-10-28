#include <plan_env/value_map2d.h>
namespace apexnav_planner {
double ValueMap::getValue(const Eigen::Vector2d& frontier) const {
  for (const auto& entry : memory_.nodes()) {
    const auto& f=entry.second.geometry;
    if (std::hypot(f.x-frontier.x(),f.y-frontier.y())<1e-6) return entry.second.value;
  }
  return 0.;
}
}
