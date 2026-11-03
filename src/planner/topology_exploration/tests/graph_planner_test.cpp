#include <topology_exploration/graph_planner.h>
#include <cassert>
#include <iostream>
using namespace apexnav_planner::topology;
int main(){
  std::map<uint64_t, sparse::TopologyNode> nodes;
  for (int id = 1; id <= 4; ++id) {
    sparse::TopologyNode n;
    n.id = id;
    n.type = id < 3?sparse::NodeType::WAYPOINT:sparse::NodeType::FRONTIER;
    n.position = {double(id), 0};
    nodes[id] = n;
  }
  std::map<std::pair<uint64_t, uint64_t>, sparse::TopologyEdge> edges;
  edges[{1, 2}] = {1, 2, 1};
  edges[{2, 3}] = {2, 3, 8};
  edges[{2, 4}] = {2, 4, 3};
  GraphPlanner p;
  auto route = p.choose(nodes, edges, {{1, .5, {{.5, 0}, {1, 0}}}}, 0);
  assert(route && route->target == 4 && std::abs(route->cost - 4.5) < 1e-9);
  edges[{2, 4}].verified = false;
  route = p.choose(nodes, edges, {{1, .5, {}}}, 0);
  assert(route && route->target == 3);
  p.fail(3, 0);
  assert(!p.choose(nodes, edges, {{1, 0, {}}}, 1));
  assert(p.choose(nodes, edges, {{1, 0, {}}}, 31));
  p.fail(3, 31);
  p.fail(3, 62);
  assert(!p.choose(nodes, edges, {{1, 0, {}}}, 999));
  p.reset();
  assert(p.choose(nodes, edges, {{1, 0, {}}}, 1000));
  // Historical geometry survives window motion; a newly observed obstacle on
  // the interior of an actual detour revokes the edge even if endpoints are far.
  auto view = [](sparse::Vec2 origin, int64_t stamp){
                sparse::DepthDescriptor d(30, 5, origin);
                for (double a = -.7; a <= .7; a += .005) {
                  d.addRay(a, 5, true, false, stamp);
                }
                d.finish(stamp);
                return d;
              };
  auto free = [](sparse::Vec2){
                return true;
              };
  auto clear = [](sparse::Vec2){
                 return false;
               };
  sparse::TopologyGraph graph;
  graph.update({0, 0}, 0, 1, view({0, 0}, 1), free, clear);
  uint64_t a = graph.currentWaypoint();
  graph.update({1.5, 0}, 0, 2, view({1.5, 0}, 2), free, clear);
  uint64_t b = graph.currentWaypoint();
  assert(a != b);
  assert(graph.recordTraversal(a, b, {{0, 0}, {0, .8}, {1.5, .8}, {1.5, 0}}, 3));
  auto key = std::minmax(a, b);
  assert(graph.edges().at(key).traversed && std::abs(graph.edges().at(key).weight - 3.1) < 1e-6);
  auto local = [](sparse::Vec2 p){
                 return p.distance({8, 0}) < 2.;
               };
  graph.update({8, 0}, 0, 4, view({8, 0}, 4), local, clear);
  assert(graph.nodes().count(a) && graph.nodes().count(b) && graph.edges().at(key).verified);
  assert(std::abs(graph.edges().at(key).weight - 3.1) < 1e-6);
  graph.update({8, 0}, 0, 5, view({8, 0}, 5), local, [](sparse::Vec2 p){
    return p.distance({.75, .8}) < .1;
  });
  assert(!graph.edges().at(key).verified);
  graph.update({8, 0}, 0, 6, view({8, 0}, 6), local, clear);
  assert(!graph.edges().at(key).verified); // Unknown is not a fresh traversal proof.
  sparse::TopologyConfig cfg;
  cfg.candidate_step = 2.2;
  cfg.clearance = .245;
  cfg.reach_radius = .35;
  cfg.coverage_radius = cfg.candidate_step + cfg.clearance;
  sparse::TopologyGraph progression(cfg);
  Vec2 center;
  auto full = [](Vec2 p, int64_t stamp){
                sparse::DepthDescriptor d(30, 5, p);
                for (double a = -sparse::PI; a < sparse::PI; a += .005) {
                  d.addRay(a, 5, true, false, stamp);
                }
                d.finish(stamp);
                return d;
              };
  auto window = [&](Vec2 p){
                  return std::abs(p.x - center.x) < 2.75 && std::abs(p.y - center.y) < 2.75;
                };
  progression.update(center, 0, 1, full(center, 1), window, clear);
  for (int step = 2; step <= 9; ++step) {
    Vec2 next = center;
    bool found = false;
    for (const auto & [id, n]:progression.nodes()) {
      if (n.type == sparse::NodeType::FRONTIER && window(n.position) && n.position.x > next.x) {
        next = n.position;
        found = true;
      }
    }
    assert(found);
    center = next;
    progression.update(center, 0, step, full(center, step), window, clear);
  }
  assert(center.x > 10 && progression.frontierCount() > 0);
  std::cout << "PASS complete-descriptor progression: x=" << center.x << " waypoints=" <<
  progression.waypointCount() << "\n";
  std::cout <<
  "PASS graph history: actual detour cost, rolling-window survival, interior obstacle invalidation\n";
  std::cout << "PASS graph A*: weighted reachable goal, invalid-edge exclusion, failure cooldown/block\n";
}
