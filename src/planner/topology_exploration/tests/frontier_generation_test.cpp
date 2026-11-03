#include <plan_env/topology_graph.h>
#include <topology_exploration/local_planner.h>
#include <cassert>
#include <iostream>
using namespace apexnav_planner::sparse;
using apexnav_planner::topology::GridView;
using apexnav_planner::topology::LocalPlanner;

DepthDescriptor doorway(int64_t stamp = 1) {
  DepthDescriptor d;
  // A nearby piece of furniture occupies part of a 30-degree sector, with
  // a ten-degree opening beside it. Both are valid obstacle observations.
  for (double a = -.25; a < .25; a += .002) {
    d.addRay(a, a > .087 && a < .262 ? 2.2 : .4, true, true, stamp);
  }
  d.finish(stamp);
  return d;
}
int main() {
  TopologyConfig cfg;
  cfg.open_distance = 1.2;
  cfg.jump_distance = 10; // This regression exercises open intervals alone.
  cfg.candidate_step = 2.;
  cfg.fine_candidates = true;
  TopologyGraph graph(cfg);
  auto free = [](Vec2 p) { return std::abs(p.x) < 3 && std::abs(p.y) < 3; };
  graph.update({}, 0, 1, doorway(), free);
  assert(graph.frontierCount() > 0 && "partial-sector opening must create a candidate");
  assert(doorway().sectors()[0].obstacle_distance == .4); // Keep the conservative minimum.
  const auto count = graph.nodes().size();
  graph.update({}, 0, 2, doorway(2), free);
  assert(graph.nodes().size() == count && graph.diagnostics().duplicate > 0);
  DepthDescriptor invalid;
  for (double a = -.25; a < .25; a += .002) invalid.addRay(a, 0, false, false, 3);
  invalid.finish(3);
  TopologyGraph unknown(cfg);
  unknown.update({}, 0, 3, invalid, free);
  assert(unknown.frontierCount() == 0);
  auto merged = doorway();
  assert(merged.merge(invalid));
  for (const auto& b : merged.directions()) assert(b.valid_samples == 0);
  assert(merged.merge(doorway(2))); // Old rays must not revive invalidated bins.
  for (const auto& b : merged.directions()) assert(b.valid_samples == 0);
  auto near = doorway(4);
  DepthDescriptor clear;
  for (double a = -.25; a < .25; a += .002) clear.addRay(a, 5, true, false, 5);
  clear.finish(5);
  assert(near.merge(clear));
  assert(near.directions()[0].free_range == 5 && "fresh valid depth must reopen a cleared direction");

  // The endpoint is free but a short furniture barrier blocks its direct ray.
  // A* has a safe route around the side; pure ray retraction stops before .8 m.
  auto safe = [](Vec2 p) {
    return std::abs(p.x) < 2.9 && std::abs(p.y) < 2.9 &&
           !(p.x > .25 && p.x < .65 && std::abs(p.y) < .6);
  };
  GridView grid{120, .05, {-3, -3}, safe};
  LocalPlanner local;
  auto connect = [&](Vec2 a, Vec2 b) { return local.plan(grid, a, b).points; };
  TopologyGraph detour(cfg);
  detour.update({}, 0, 1, doorway(), safe, {}, connect);
  assert(detour.frontierCount() > 0 && detour.diagnostics().detours > 0);
  for (const auto& [key, edge] : detour.edges()) {
    assert(edge.verified && !edge.traversed && edge.path.size() > 2);
    double cost = 0;
    for (size_t i = 1; i < edge.path.size(); ++i) {
      assert(LocalPlanner::segmentSafe(grid, edge.path[i-1], edge.path[i]));
      cost += edge.path[i-1].distance(edge.path[i]);
    }
    assert(std::abs(cost - edge.weight) < 1e-9);
    assert(cost > edge.path.front().distance(edge.path.back()) + .1);
  }
  // Never trust a callback's unverified shortcut across occupied/unknown cells.
  TopologyGraph unsafe(cfg);
  unsafe.update({}, 0, 1, doorway(), safe, {}, [](Vec2 a, Vec2 b) {
    return std::vector<Vec2>{a, b};
  });
  assert(unsafe.frontierCount() == 0 && unsafe.diagnostics().no_path > 0);
  auto blocked = [](Vec2 p) { return p.x > .25 && p.x < .65; };
  detour.update({}, 0, 2, doorway(2), [&](Vec2 p) { return safe(p) && !blocked(p); }, blocked,
                [](Vec2, Vec2) { return std::vector<Vec2>{}; });
  for (const auto& [key, edge] : detour.edges()) assert(!edge.verified);
  const auto ids = detour.nodes().size();
  detour.update({}, 0, 3, doorway(3), safe, [](Vec2) { return false; }, connect);
  assert(detour.nodes().size() == ids);
  for (const auto& [key, edge] : detour.edges()) assert(edge.verified && "fresh A* must repair an existing frontier edge");
  const auto old_path = detour.edges().begin()->second.path;
  Vec2 new_obstacle = (old_path[0] + old_path[1]) * .5;
  auto changed = [&](Vec2 p) { return safe(p) && p.distance(new_obstacle) > .07; };
  GridView changed_grid{120, .05, {-3, -3}, changed};
  auto alternate = [&](Vec2 a, Vec2 b) { return local.plan(changed_grid, a, b).points; };
  detour.update({}, 0, 4, doorway(4), changed,
                [&](Vec2 p) { return p.distance(new_obstacle) <= .07; }, alternate);
  assert(detour.nodes().size() == ids && detour.diagnostics().reconnected > 0);
  for (const auto& [key, edge] : detour.edges()) {
    assert(edge.verified);
    for (size_t i = 1; i < edge.path.size(); ++i)
      assert(LocalPlanner::segmentSafe(changed_grid, edge.path[i-1], edge.path[i]));
  }
  detour.reset();
  assert(detour.nodes().empty() && detour.diagnostics().open_bins == 0);
  std::cout << "PASS partial-sector candidate generation\n";
  std::cout << "PASS verified detour costs, invalid-depth rejection, deduplication, obstacle invalidation\n";
}
