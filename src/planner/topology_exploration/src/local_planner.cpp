#include <topology_exploration/local_planner.h>
#include <queue>
#include <limits>
namespace apexnav_planner::topology {
std::vector<Vec2> LocalPlanner::nextLeg(const std::vector<Vec2> &route, Vec2 start, size_t &cursor){
  if (route.empty()) {
    return {};
  }
  cursor = std::max(size_t(1), cursor);
  while (cursor < route.size() && start.distance(route[cursor]) < .22) {
    ++cursor;
  }
  // A local shortcut may not bypass a vertex that still owns route progress.
  // Preserve edge-polyline order, including U corridors and actual detours.
  return {start, route[std::min(cursor, route.size() - 1)]};
}
bool LocalPlanner::segmentSafe(const GridView &g, Vec2 a, Vec2 b){
  int steps = std::max(1, int(std::ceil(a.distance(b) / (g.resolution * .5))));
  for (int i = 0; i <= steps; ++i) {
    if (!g.safe(a + (b - a) * (double(i) / steps))) {
      return false;
    }
  }
  return true;
}
LocalPath LocalPlanner::plan(const GridView &g, Vec2 start, Vec2 goal)const {
  LocalPath result;
  if (!g.safe(start) || !g.safe(goal)) {
    return result;
  }
  if (segmentSafe(g, start, goal)) {
    return {true, start.distance(goal), {start, goal}};
  }
  auto index = [&](Vec2 p){
                 int x = int(std::floor((p.x - g.origin.x) / g.resolution)),
       y = int(std::floor((p.y - g.origin.y) / g.resolution));
                 return x >= 0 && y >= 0 && x < g.n && y < g.n?x * g.n + y:-1;
               };
  auto point = [&](int a){
                 return g.origin + Vec2{(a / g.n + .5) * g.resolution, (a % g.n + .5) * g.resolution};
               };
  int from = index(start), to = index(goal), size = g.n * g.n;
  if (from < 0 || to < 0) {
    return result;
  }
  std::vector<double> cost(size, std::numeric_limits<double>::infinity());
  std::vector<int> parent(size, -1);
  std::vector<int8_t> safe(size, -1);
  auto pass = [&](int id){
                if (safe[id] < 0) {
                  safe[id] = g.safe(point(id))?1:0;
                }
                return safe[id] > 0;
              };
  if (!pass(from) || !pass(to) || !segmentSafe(g, start, point(from)) || !segmentSafe(g, point(to), goal)) {
    return result;
  }
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item> > open;
  cost[from] = start.distance(point(from));
  open.push({cost[from] + point(from).distance(goal), from});
  while (!open.empty()) {
    auto [f, u] = open.top();
    open.pop();
    if (f > cost[u] + point(u).distance(goal) + 1e-8) {
      continue;
    }
    if (u == to) {
      break;
    }
    int x = u / g.n, y = u % g.n;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if ((dx == 0 && dy == 0) || x + dx < 0 || y + dy < 0 || x + dx >= g.n || y + dy >= g.n) {
          continue;
        }
        int v = (x + dx) * g.n + y + dy;
        if (!pass(v)) {
          continue;
        }
        if (dx && dy && (!pass((x + dx) * g.n + y) || !pass(x * g.n + y + dy))) {
          continue;
        }
        if (!segmentSafe(g, point(u), point(v))) {
          continue;
        }
        double c = cost[u] + std::hypot(dx, dy) * g.resolution;
        if (c < cost[v]) {
          cost[v] = c;
          parent[v] = u;
          open.push({c + point(v).distance(goal), v});
        }
      }
    }
  }
  if (!std::isfinite(cost[to])) {
    return result;
  }
  std::vector<Vec2> dense{goal};
  for (int u = to; u != -1; u = parent[u]) {
    dense.push_back(point(u));
  }
  dense.push_back(start);
  std::reverse(dense.begin(), dense.end());
  result.points.push_back(start);
  for (size_t i = 0; i + 1 < dense.size();) {
    size_t j = dense.size() - 1;
    while (j > i + 1 && !segmentSafe(g, dense[i], dense[j])) {
      --j;
    }
    result.points.push_back(dense[j]);
    i = j;
  }
  result.complete = true;
  for (size_t i = 1; i < result.points.size(); ++i) {
    result.cost += result.points[i - 1].distance(result.points[i]);
  }
  return result;
}
std::vector<Vec2> LocalPlanner::orderedPrefix(const GridView &g, Vec2 start,
                                              const std::vector<Vec2> &route)const {
  std::vector<Vec2> out;
  if (route.empty() || !g.safe(start)) {
    return out;
  }
  out.push_back(start);
  // Route begins at its verified current-pose connection. Follow its order;
  // do not select a different branch by proximity to the remote goal.
  Vec2 previous = start;
  for (Vec2 end:route) {
    int n = std::max(1, int(std::ceil(previous.distance(end) / (g.resolution * .5))));
    for (int i = 1; i <= n; ++i) {
      Vec2 p = previous + (end - previous) * (double(i) / n);
      if (!g.safe(p)) {
        return out;
      }
      out.push_back(p);
    }
    previous = end;
  }
  return out;
}
}
