#include <plan_env/topology_graph.h>
#include <array>

namespace apexnav_planner::sparse {
TopologyGraph::TopologyGraph(TopologyConfig c) : cfg_(c) {
  if(!std::isfinite(c.coverage_radius)||c.coverage_radius<0)throw std::invalid_argument("invalid coverage radius");
  DepthDescriptor check(c.sector_degrees, c.max_distance);
  for (double v:{c.open_distance, c.jump_distance, c.candidate_step, c.min_candidate_distance,
                 c.reach_radius, c.duplicate_radius, c.waypoint_spacing, c.clearance, c.ray_step, c.min_open_angle_degrees}) {
    if (!std::isfinite(v) || v <= 0) {
      throw std::invalid_argument("topology distances must be finite and positive");
    }
  }
  if (c.open_distance > c.max_distance || c.min_candidate_distance >= c.candidate_step ||
      c.reach_radius >= c.duplicate_radius || c.waypoint_spacing <= c.duplicate_radius ||
      c.min_open_angle_degrees > 360 || c.min_open_sectors < 1 || c.max_interval_sectors < c.min_open_sectors) {
    throw std::invalid_argument("inconsistent topology thresholds");
  }
}
void TopologyGraph::reset() {
  nodes_.clear();
  edges_.clear();
  next_id_ = 1;
  current_ = 0;
  last_stamp_ = 0;
  stats_ = {};
  diagnostics_ = {};
}
size_t TopologyGraph::waypointCount()const {
  size_t n = 0;
  for (const auto& [id, node]:nodes_) {
    n += node.type == NodeType::WAYPOINT;
  }
  return n;
}
bool TopologyGraph::erase(uint64_t id) {
  if (!nodes_.erase(id)) {
    return false;
  }
  for (auto it = edges_.begin(); it != edges_.end();) {
    if (it->second.a == id || it->second.b == id) {
      it = edges_.erase(it);
    }
    else{
      ++it;
    }
  }
  if (current_ == id) {
    current_ = 0;
  }
  ++stats_.removed;
  return true;
}
bool TopologyGraph::lineSafe(Vec2 a, Vec2 b, const Safe &safe)const {
  int steps = std::max(1, int(std::ceil(a.distance(b) / cfg_.ray_step)));
  for (int i = 0; i <= steps; ++i) {
    if (!safe(a + (b - a) * (double(i) / steps))) {
      return false;
    }
  }
  return true;
}
uint64_t TopologyGraph::create(NodeType type, Vec2 p, const DepthDescriptor &d, int64_t stamp, uint64_t source) {
  TopologyNode n;
  n.id = next_id_++;
  n.type = type;
  n.position = p;
  n.descriptor = d;
  n.stamp = stamp;
  n.source_id = source;
  n.measured_at_node = type == NodeType::WAYPOINT;
  nodes_.emplace(n.id, std::move(n));
  ++stats_.created;
  return next_id_ - 1;
}
void TopologyGraph::storeView(TopologyNode &n, double yaw, int64_t stamp) {
  auto slot = n.descriptor.index(yaw);
  // Exact camera pose is supplied with the matching RGB; retain completed views.
  n.pending_images[stamp] = slot;
  while (n.pending_images.size() > 20) {
    n.pending_images.erase(n.pending_images.begin());
  }
}
std::vector<Vec2> TopologyGraph::candidates(const DepthDescriptor &d) {
  const auto &bins = cfg_.fine_candidates ? d.directions() : d.sectors();
  auto observed = [&](int i) {
    return cfg_.fine_candidates ? bins[i].valid_samples > 0 : bins[i].full();
  };
  auto center = [&](int i) { return cfg_.fine_candidates ? d.directionCenter(i) : d.center(i); };
  const int min_count = cfg_.fine_candidates ?
    std::max(2, int(std::ceil(cfg_.min_open_angle_degrees / (cfg_.sector_degrees / 6)))) : cfg_.min_open_sectors;
  const int max_count = std::max(min_count, cfg_.max_interval_sectors * (cfg_.fine_candidates ? 6 : 1));
  int n = bins.size();
  std::vector<Vec2> out;
  std::vector<bool> open(n, false);
  for (int i = 0; i < n; ++i) {
    open[i] = observed(i) && bins[i].free_range >= cfg_.open_distance;
    diagnostics_.open_bins += open[i];
  }
  int cut = 0;
  while (cut < n && open[cut]) {
    ++cut;
  }
  if (cut == n) {
    cut = 0;       // all-around observed open space, divide into bounded intervals
  }
  int scanned = 0;
  while (scanned < n) {
    int start = (cut + scanned) % n;
    if (!open[start]) {
      ++scanned;
      continue;
    }
    int len = 0;
    while (scanned + len < n && open[(start + len) % n]) {
      ++len;
    }
    for (int offset = 0; offset < len; offset += max_count) {
      int count = std::min(max_count, len - offset);
      if (count < min_count) {
        continue;
      }
      int l = (start + offset) % n, r = (start + offset + count - 1) % n;
      const double margin = cfg_.fine_candidates ? cfg_.clearance : 0.;
      Vec2 middle = (direction(center(l)) * std::min(bins[l].free_range - margin, cfg_.candidate_step) +
                     direction(center(r)) * std::min(bins[r].free_range - margin, cfg_.candidate_step)) * 0.5;
      out.push_back(d.origin() + middle);
      ++diagnostics_.open_candidates;
    }
    scanned += len;
  }
  // Unknown/invalid bins cannot create a discontinuity frontier.
  for (int i = 0; i < n; ++i) {
    int j = (i + 1) % n;
    if (!observed(i) || !observed(j) ||
        std::abs(bins[i].free_range - bins[j].free_range) < cfg_.jump_distance) {
      continue;
    }
    int far = bins[i].free_range > bins[j].free_range ? i : j;
    double r = std::min(cfg_.candidate_step, bins[far].free_range - cfg_.clearance);
    if (r >= cfg_.min_candidate_distance) {
      out.push_back(d.origin() + direction(center(far)) * r);
      ++diagnostics_.jump_candidates;
    }
  }
  return out;
}
void TopologyGraph::update(Vec2 p, double yaw, int64_t stamp, const DepthDescriptor &frame, const Safe &safe, const Safe &blocked, const Connect &connect) {
  if (stamp <= last_stamp_) {
    return;
  }
  last_stamp_ = stamp;
  diagnostics_ = {};
  // Current grid changes can invalidate candidates and visibility edges, including
  // nodes outside today's FoV. Lack of observation alone is never an obstacle.
  std::vector<uint64_t> invalid;
  for (const auto& [id, n]:nodes_) {
    if (blocked ? blocked(n.position) : !safe(n.position)) {
      invalid.push_back(id);
    }
  }
  for (auto id:invalid) {
    erase(id);
  }
  auto edgeClear = [&](const TopologyEdge& e) {
    auto points=e.path;
    if(points.empty()) points={nodes_.at(e.a).position,nodes_.at(e.b).position};
    for(size_t i=1;i<points.size();++i)
      if(!lineSafe(points[i-1],points[i],blocked ? Safe([&](Vec2 p){return !blocked(p);}) : safe))return false;
    return true;
  };
  for (auto it = edges_.begin(); it != edges_.end();) {
    if (!edgeClear(it->second)) {
      if(blocked){it->second.verified=false;++it;}else it = edges_.erase(it);
    }
    else{
      ++it;
    }
  }
  if (!safe(p)) {
    return;
  }
  uint64_t active = 0;
  double best = cfg_.reach_radius;
  for (const auto& [id, n]:nodes_) {
    if (n.position.distance(p) <= best && lineSafe(n.position, p, safe)) {
      best = n.position.distance(p);
      active = id;
    }
  }
  if (active) {
    auto &n = nodes_.at(active);
    if (n.type == NodeType::FRONTIER) {
      n.type = NodeType::WAYPOINT;
      n.position = p;
      n.descriptor = frame;
      n.images.clear();
      n.pending_images.clear();
      n.measured_at_node = true;
      ++stats_.converted;
    }
    else if (active != current_) {
      ++stats_.revisits;
    }
  }
  else{
    double closest = 1e9;
    for (const auto& [id, n]:nodes_) {
      if (n.type == NodeType::WAYPOINT) {
        closest = std::min(closest, n.position.distance(p));
      }
    }
    if (closest >= cfg_.waypoint_spacing) {
      active = create(NodeType::WAYPOINT, p, frame, stamp, 0);
    }
  }
  if (active) {
    current_ = active;
  }
  // Attach observations only near a visited anchor. Translate => retain actual
  // descriptor origin, do not splice sectors from displaced camera centers.
  if (!current_ || nodes_.at(current_).position.distance(p) > cfg_.reach_radius) {
    return;
  }
  diagnostics_.at_anchor = true;
  auto &source = nodes_.at(current_);
  if (!source.descriptor.merge(frame)) {
    source.descriptor = frame;
  }
  source.stamp = stamp;
  storeView(source, yaw, stamp);
  auto verifiedConnection = [&](Vec2 a, Vec2 b) {
    std::vector<Vec2> path;
    if (lineSafe(a, b, safe)) path = {a, b};
    else path = connect(a, b);
    bool valid = path.size() >= 2 && path.front().distance(a) < 1e-6 &&
                 path.back().distance(b) < 1e-6;
    double cost = 0;
    for (size_t i = 0; valid && i < path.size(); ++i) {
      valid = std::isfinite(path[i].x) && std::isfinite(path[i].y);
      if (valid && i) {
        valid = lineSafe(path[i-1], path[i], safe);
        cost += path[i-1].distance(path[i]);
      }
    }
    if (!valid || !std::isfinite(cost) || cost <= 0) path.clear();
    return path;
  };
  for (auto candidate:candidates(source.descriptor)) {
    Vec2 delta = candidate - p;
    double length = delta.norm();
    if (length < cfg_.min_candidate_distance) {
      ++diagnostics_.too_close;
      continue;
    }
    if (connect) {
      // First retract only an unsafe endpoint. A blocked direct ray is not
      // evidence that a known-free detour to this endpoint is impossible.
      double r = length;
      while (r >= cfg_.min_candidate_distance && !safe(p + delta * (r / length))) r -= cfg_.ray_step;
      if (r < cfg_.min_candidate_distance) {
        ++diagnostics_.unsafe;
        continue;
      }
      candidate = p + delta * (r / length);
    } else {
      // Legacy observer has only a line-of-sight connectivity verifier.
      double reachable = 0;
      for (double r = cfg_.ray_step; r <= length; r += cfg_.ray_step) {
        if (!safe(p + delta * (r / length))) break;
        reachable = r;
      }
      if (reachable < cfg_.min_candidate_distance) {
        ++diagnostics_.unsafe;
        continue;
      }
      candidate = p + delta * (reachable / length);
    }
    bool duplicate = false;
    for (const auto& [id, n]:nodes_) {
      if (n.position.distance(candidate) < cfg_.duplicate_radius) {
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      ++stats_.duplicates;
      ++diagnostics_.duplicate;
      continue;
    }
    bool covered = false;
    for (const auto& [id, n]:nodes_) {
      if (id != current_ && n.type == NodeType::WAYPOINT &&
          n.descriptor.covers(candidate, std::max(cfg_.clearance,cfg_.coverage_radius)) && lineSafe(n.descriptor.origin(), candidate, safe)) {
        covered = true;
        break;
      }
    }
    if (covered) {
      ++stats_.covered;
      ++diagnostics_.covered;
      continue;
    }
    std::vector<Vec2> path;
    bool detour = false;
    double cost = 0;
    if (connect) {
      path = verifiedConnection(source.position, candidate);
      if (path.empty()) {
        ++diagnostics_.no_path;
        continue;
      }
      detour = path.size() > 2;
      for (size_t i = 1; i < path.size(); ++i) cost += path[i-1].distance(path[i]);
    }
    auto id = create(NodeType::FRONTIER, candidate, source.descriptor, stamp, current_);
    ++diagnostics_.accepted;
    if (connect) {
      auto key = std::minmax(current_, id);
      if (current_ > id) std::reverse(path.begin(), path.end());
      edges_[key] = {key.first, key.second, cost, true, stamp, std::move(path), false};
      diagnostics_.detours += detour;
      stats_.detours += detour;
    }
    // Preserve the directional views that contributed to the source descriptor.
    // JPEG buffers are shared; arrival replaces them with local measurements.
    nodes_.at(id).images = source.images;
    nodes_.at(id).pending_images = source.pending_images;
    storeView(nodes_.at(id), yaw, stamp);
  }
  // Only measured WAYPOINT descriptors provide coverage; a candidate's source
  // descriptor never masquerades as a measurement at its unvisited position.
  invalid.clear();
  for (const auto& [id, n]:nodes_) {
    if (n.type == NodeType::FRONTIER) {
      for (const auto& [wid, w]:nodes_) {
        if (w.type == NodeType::WAYPOINT && wid != n.source_id &&
            w.descriptor.covers(n.position, std::max(cfg_.clearance,cfg_.coverage_radius)) && lineSafe(w.descriptor.origin(), n.position, safe)) {
          invalid.push_back(id);
          break;
        }
      }
    }
  }
  for (auto id:invalid) {
    erase(id);
  }
  for (const auto& [id, n]:nodes_) {
    if (id == current_ || p.distance(n.position) > cfg_.max_distance) {
      continue;
    }
    auto key = std::minmax(current_, id);
    auto existing = edges_.find(key);
    // A duplicate candidate reuses its node ID, but may still need a fresh
    // connection after an obstacle invalidated the previous detour. Rebuild
    // only with complete current known-free proof; Unknown cannot revive it.
    if (connect && (existing == edges_.end() || !existing->second.verified) &&
        safe(source.position) && safe(n.position)) {
      auto path = verifiedConnection(source.position, n.position);
      if (!path.empty()) {
        double cost = 0;
        for (size_t i = 1; i < path.size(); ++i) cost += path[i-1].distance(path[i]);
        stats_.detours += path.size() > 2;
        if (current_ > id) std::reverse(path.begin(), path.end());
        edges_[key] = {key.first, key.second, cost, true, stamp, std::move(path), false};
        ++diagnostics_.reconnected;
      }
      continue;
    }
    bool visible = source.descriptor.covers(n.position, cfg_.clearance) ||
                   (n.type == NodeType::WAYPOINT && n.descriptor.covers(source.position, cfg_.clearance));
    if (!visible || !lineSafe(source.position, n.position, safe)) {
      continue;
    }
    if(existing==edges_.end() || !existing->second.traversed || !existing->second.verified)
      edges_[key] = {key.first, key.second, source.position.distance(n.position), true, stamp,
                    {nodes_.at(key.first).position,nodes_.at(key.second).position},false};
  }
  // A relocated endpoint changes both the validated geometry and its cost.
  for (auto it=edges_.begin();it!=edges_.end();) {
    auto& e=it->second;
    if(e.path.empty()) e.path={nodes_.at(e.a).position,nodes_.at(e.b).position};
    e.path.front()=nodes_.at(e.a).position;e.path.back()=nodes_.at(e.b).position;
    if(!edgeClear(e)) {
      if(blocked){e.verified=false;++it;}else it=edges_.erase(it);
    }else{
      e.weight=0;for(size_t i=1;i<e.path.size();++i)e.weight+=e.path[i-1].distance(e.path[i]);
      ++it;
    }
  }
}
void TopologyGraph::invalidateEdge(uint64_t a,uint64_t b){
  auto it=edges_.find(std::minmax(a,b));if(it!=edges_.end())it->second.verified=false;
}
bool TopologyGraph::recordTraversal(uint64_t a,uint64_t b,std::vector<Vec2> path,int64_t stamp){
  if(a==b||!nodes_.count(a)||!nodes_.count(b)||path.size()<2)return false;
  if(path.front().distance(nodes_.at(a).position)>.5 || path.back().distance(nodes_.at(b).position)>.5)return false;
  path.front()=nodes_.at(a).position;path.back()=nodes_.at(b).position;
  double length=0;for(size_t i=1;i<path.size();++i){
    if(!std::isfinite(path[i].x)||!std::isfinite(path[i].y))return false;
    length+=path[i-1].distance(path[i]);
  }
  if(!std::isfinite(length)||length<=0)return false;
  if(a>b){std::swap(a,b);std::reverse(path.begin(),path.end());}
  edges_[{a,b}]={a,b,length,true,stamp,std::move(path),true};return true;
}
bool TopologyGraph::wantsImage(int64_t stamp)const {
  for (const auto& [id, n]:nodes_) {
    if (n.pending_images.count(stamp)) {
      return true;
    }
  }
  return false;
}
void TopologyGraph::attachImage(const DirectionalImage &image) {
  if (!image.jpeg || image.jpeg->empty()) {
    return;
  }
  for (auto& [id, n]:nodes_) {
    auto pending = n.pending_images.find(image.stamp);
    if (pending == n.pending_images.end()) {
      continue;
    }
    const auto slot = pending->second;
    auto old = n.images.find(slot);
    if (old == n.images.end() || old->second.stamp < image.stamp) {
      n.images[slot] = image;
    }
    n.pending_images.erase(pending);
  }
}
}
