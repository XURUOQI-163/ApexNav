#include <plan_env/topology_graph.h>
#include <iostream>
#include <stdexcept>
using namespace apexnav_planner::sparse;
void check(bool x, const char *text) {
  if (!x) {
    throw std::runtime_error(text);
  }
}
DepthDescriptor view(Vec2 p, double yaw, double range, int64_t stamp, bool invalid = false) {
  DepthDescriptor d(30, 5, p);
  for (double a = yaw - 0.65; a <= yaw + 0.65; a += 0.008) {
    d.addRay(a, range, !invalid, range < 4.99, stamp);
  }
  d.finish(stamp);
  return d;
}
int main() {
  check(!footprintClear({.049, .049}, .18, .05, {0, 0}, [](int x, int y) {
    return !(x == 4 && y == 1);
  }),
        "footprint must reject intersecting unknown cell even near grid boundary");
  check(footprintClear({.049, .049}, .18, .05, {0, 0}, [](int x, int y) {
    return !(x == 4 && y == 4);
  }),
        "nonintersecting diagonal cells must not block circular footprint");
  auto d = view({0, 0}, 0, 5, 1);
  check(d.sectors()[6].state == ObservationState::UNOBSERVED, "rear must stay unobserved");
  check(d.covers({2, 0}, 0.1), "forward free space must be covered");
  check(!d.covers({-2, 0}, 0.1), "rear must not be covered");
  auto bad = view({0, 0}, 0, 5, 2, true);
  check(!bad.covers({1, 0}, 0), "invalid rays cannot clear space");
  auto wall = view({0, 0}, 0, 2, 3);
  check(wall.sectors()[0].state == ObservationState::OBSTACLE, "valid obstacle must be distinct from free");
  check(wall.covers({1, 0}, .1) && !wall.covers({2.5, 0}, .1), "coverage cannot extend behind obstacle");
  auto back = view({0, 0}, PI, 5, 4);
  check(d.merge(back), "rotation at same anchor must merge");
  check(d.covers({-2, 0}, .1), "rotating camera must fill the rear descriptor");
  auto translated = view({1, 0}, 0, 5, 5);
  check(!d.merge(translated), "translation must not splice different origins");
  DepthDescriptor partial;
  partial.addRay(0, 5, true, false, 1);
  partial.finish(1);
  check(!partial.covers({1, 0}, .1), "one ray cannot clear a whole 30 degree sector");
  d = view({0, 0}, 0, 5, 1);
  TopologyConfig cfg;
  TopologyGraph graph(cfg);
  auto safe = [](Vec2 p) {
                return std::abs(p.x) < 20 && std::abs(p.y) < 20;
              };
  graph.update({0, 0}, 0, 1, d, safe);
  check(graph.waypointCount() == 1 && graph.frontierCount() > 0, "initial graph must contain waypoint/frontier");
  auto initial = graph.nodes().size();
  for (int i = 2; i <= 10; ++i) {
    graph.update({0, 0}, 0, i, view({0, 0}, 0, 5, i), safe);
  }
  check(graph.nodes().size() == initial, "same frames must not grow duplicate nodes");
  Vec2 goal;
  uint64_t target = 0;
  for (const auto& [id, n]:graph.nodes()) {
    if (n.type == NodeType::FRONTIER) {
      goal = n.position;
      target = id;
      break;
    }
  }
  goal.x -= .3;
  graph.update(goal, 0, 11, view(goal, 0, 5, 11), safe);
  check(graph.nodes().at(target).type == NodeType::WAYPOINT, "arrival must convert the same node ID");
  check(graph.nodes().at(target).descriptor.origin().distance(goal) < 1e-9, "visited descriptor must be local");
  std::cout << "fixture visited: waypoint=" << graph.waypointCount() << " frontier=" << graph.frontierCount()
            << " edges=" << graph.edges().size() << " duplicate_rejections=" << graph.stats().duplicates << "\n";
  for (const auto& [key, e]:graph.edges()) {
    check(std::abs(e.weight - graph.nodes().at(e.a).position.distance(graph.nodes().at(e.b).position)) < 1e-9, "arrival must refresh all edge weights");
  }
  TopologyGraph follow(cfg);
  follow.update({0, 0}, 0, 1, view({0, 0}, 0, 5, 1), safe);
  follow.update({1.5, 0}, 0, 2, view({1.5, 0}, 0, 5, 2), safe);
  bool anchors_connected = false;
  for (const auto& [key, e]:follow.edges()) {
    if (follow.nodes().at(e.a).type == NodeType::WAYPOINT && follow.nodes().at(e.b).type == NodeType::WAYPOINT) {
      anchors_connected = true;
    }
  }
  check(anchors_connected, "historical visibility must connect forward-moving anchors");
  auto visited = graph.waypointCount();
  graph.update({0, 0}, PI, 12, view({0, 0}, PI, 5, 12), safe);
  check(graph.waypointCount() == visited && graph.stats().revisits == 1, "revisit must reuse a waypoint");
  DirectionalImage image;
  image.stamp = 12;
  image.camera_position = {0, 0};
  image.yaw = PI;
  image.jpeg = std::make_shared<const std::vector<unsigned char> >(std::vector<unsigned char>{1, 2, 3});
  graph.attachImage(image);
  bool attached = false;
  for (const auto& [id, n]:graph.nodes()) {
    for (const auto& [slot, v]:n.images) {
      if (v.stamp == 12 && v.jpeg) {
        attached = true;
      }
    }
  }
  check(attached, "exact-stamp directional image must attach");
  check(!graph.wantsImage(999), "unmatched images must never attach");
  graph.update({0, 0}, PI, 13, view({0, 0}, PI, 5, 13), safe);
  bool retained = false;
  for (const auto& [id, n]:graph.nodes()) {
    for (const auto& [slot, v]:n.images) {
      if (v.stamp == 12 && v.jpeg) {
        retained = true;
      }
    }
  }
  check(retained, "a dropped RGB frame must not erase an already saved directional image");
  graph.update({0, 0}, PI, 14, view({0, 0}, PI, 5, 14), safe);
  check(graph.wantsImage(13), "delayed RGB must remain matchable after a newer depth frame");
  image.stamp = 14;
  graph.attachImage(image);
  image.stamp = 13;
  graph.attachImage(image);
  for (const auto& [id, n]:graph.nodes()) {
    if (n.type == NodeType::WAYPOINT && n.position.norm() < .1) {
      check(n.images.at(n.descriptor.index(PI)).stamp == 14, "late old image must not overwrite a newer image");
    }
  }
  size_t duplicates = 0;
  for (auto a = graph.nodes().begin(); a != graph.nodes().end(); ++a) {
    for (auto b = std::next(a); b != graph.nodes().end(); ++b) {
      duplicates += a->second.position.distance(b->second.position) < cfg.reach_radius;
    }
  }
  check(!duplicates, "graph must not contain colocated duplicates");
  auto barrier = [](Vec2 p) {
                   return !(p.x > 1 && p.x < 2);
                 };
  check(!graph.lineSafe({0, 0}, {3, 0}, barrier), "edges cannot cross an unseen or occupied strip");
  graph.update({0, 0}, PI, 15, view({0, 0}, PI, 5, 15), barrier);
  for (const auto& [key, e]:graph.edges()) {
    check(graph.lineSafe(graph.nodes().at(e.a).position, graph.nodes().at(e.b).position, barrier), "changed geometry must prune edges");
  }
  graph.update({0, 0}, 0, 16, view({0, 0}, 0, 5, 16), [](Vec2) {
    return false;
  });
  check(graph.nodes().empty(), "invalid node locations must be deleted");
  TopologyGraph empty(cfg);
  empty.update({0, 0}, 0, 1, bad, safe);
  check(empty.frontierCount() == 0, "invalid-only observations must not produce frontier candidates");
  TopologyGraph jump(cfg);
  DepthDescriptor discontinuity;
  for (double a = -.4; a < .9; a += .006) {
    discontinuity.addRay(a, a < .262 ? 1.5 : 3.5, true, true, 1);
  }
  discontinuity.finish(1);
  jump.update({0, 0}, 0, 1, discontinuity, safe);
  check(jump.frontierCount() > 0, "depth jump must generate a safe-side candidate even without max-range rays");
  std::cout << "fixture checks: actual_duplicate_nodes=" << duplicates << " invalid_frontiers=" << empty.frontierCount() << " jump_frontiers=" << jump.frontierCount() << "\n";
  graph.reset();
  check(graph.nodes().empty() && graph.edges().empty(), "reset must clear graph");
}
