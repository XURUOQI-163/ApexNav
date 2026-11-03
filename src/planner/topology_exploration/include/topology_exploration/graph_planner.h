#pragma once
#include <plan_env/topology_graph.h>
#include <optional>
#include <queue>
namespace apexnav_planner::topology {
namespace sparse = apexnav_planner::sparse;
using sparse::Vec2;
struct Connection {
  uint64_t waypoint;
  double cost;
  std::vector<Vec2> path;
};
struct Route {
  uint64_t target = 0;
  double cost = 0;
  std::vector<uint64_t> nodes;
  std::vector<Vec2> path;
};
struct GoalStatus {
  unsigned failures = 0;
  double retry_after = 0;
  bool blocked = false;
};
class GraphPlanner {
public:
std::optional<Route> choose(const std::map<uint64_t, sparse::TopologyNode> &nodes,
                            const std::map<std::pair<uint64_t, uint64_t>, sparse::TopologyEdge> &edges,
                            const std::vector<Connection> &connections, double now)const;
void fail(uint64_t id, double now){
  auto &s = status_[id];
  s.failures++;
  s.retry_after = now + 30.;
  s.blocked = s.failures >= 3;
}
void reset(){
  status_.clear();
}
const std::map<uint64_t, GoalStatus> &status()const {
  return status_;
}
private:
std::map<uint64_t, GoalStatus> status_;
};
}
