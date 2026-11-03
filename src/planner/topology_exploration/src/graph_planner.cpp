#include <topology_exploration/graph_planner.h>
#include <limits>
namespace apexnav_planner::topology {
std::optional<Route> GraphPlanner::choose(const std::map<uint64_t, sparse::TopologyNode> &nodes,
                                          const std::map<std::pair<uint64_t, uint64_t>,
                                                         sparse::TopologyEdge> &edges,
                                          const std::vector<Connection> &connections, double now)const {
  std::optional<Route> best;
  for (const auto & [goal, n]:nodes) {
    if (n.type != sparse::NodeType::FRONTIER) {
      continue;
    }
    auto status = status_.find(goal);
    if (status != status_.end() && (status->second.blocked || now < status->second.retry_after)) {
      continue;
    }
    for (const auto &connection:connections) {
      if (!nodes.count(connection.waypoint) ||
          nodes.at(connection.waypoint).type != sparse::NodeType::WAYPOINT ||
          !std::isfinite(connection.cost) || connection.cost < 0) {
        continue;
      }
      using Item = std::pair<double, uint64_t>;
      std::priority_queue<Item, std::vector<Item>, std::greater<Item> > open;
      std::map<uint64_t, double> dist;
      std::map<uint64_t, uint64_t> parent;
      auto h = [&](uint64_t id){
                 return nodes.at(id).position.distance(n.position);
               };
      dist[connection.waypoint] = connection.cost;
      open.push({connection.cost + h(connection.waypoint), connection.waypoint});
      while (!open.empty()) {
        auto [priority, u] = open.top();
        open.pop();
        if (priority > dist.at(u) + h(u) + 1e-8) {
          continue;
        }
        if (u == goal) {
          break;
        }
        if (nodes.at(u).type != sparse::NodeType::WAYPOINT) {
          continue;
        }
        for (const auto & [key, e]:edges) {
          if (!e.verified || !std::isfinite(e.weight) || e.weight <= 0 || !nodes.count(e.a) ||
              !nodes.count(e.b)) {
            continue;
          }
          if (e.weight + 1e-6 < nodes.at(e.a).position.distance(nodes.at(e.b).position)) {
            continue;
          }
          uint64_t v = e.a == u?e.b:e.b == u?e.a:0;
          if (!v) {
            continue;
          }
          if (nodes.at(v).type == sparse::NodeType::FRONTIER && v != goal) {
            continue;
          }
          double next = dist.at(u) + e.weight;
          if (!dist.count(v) || next < dist[v]) {
            dist[v] = next;
            parent[v] = u;
            open.push({next + h(v), v});
          }
        }
      }
      if (!dist.count(goal) || (best && dist[goal] >= best->cost)) {
        continue;
      }
      Route result;
      result.target = goal;
      result.cost = dist[goal];
      for (uint64_t v = goal;; v = parent.at(v)) {
        result.nodes.push_back(v);
        if (v == connection.waypoint) {
          break;
        }
      }
      std::reverse(result.nodes.begin(), result.nodes.end());
      result.path = connection.path;
      if (result.path.empty()) {
        result.path.push_back(nodes.at(connection.waypoint).position);
      }
      for (size_t i = 1; i < result.nodes.size(); ++i) {
        uint64_t a = result.nodes[i - 1], b = result.nodes[i];
        const auto &e = edges.at(std::minmax(a, b));
        auto path = e.path;
        if (path.empty()) {
          path = {nodes.at(e.a).position, nodes.at(e.b).position};
        }
        if (a != e.a) {
          std::reverse(path.begin(), path.end());
        }
        result.path.insert(result.path.end(), path.begin() + 1, path.end());
      }
      best = std::move(result);
    }
  }
  return best;
}
}
