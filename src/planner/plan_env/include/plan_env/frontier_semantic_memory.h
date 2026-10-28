#ifndef PLAN_ENV_FRONTIER_SEMANTIC_MEMORY_H
#define PLAN_ENV_FRONTIER_SEMANTIC_MEMORY_H
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace apexnav_planner {
struct FrontierCellGeometry { int key, ix, iy; double x, y; };
struct FrontierGeometry {
  int id; double x, y; bool active;
  std::vector<int> cells;
};
struct CellProjection { uint64_t id; double confidence; };
struct FrontierCellSemantic {
  FrontierCellGeometry geometry;
  uint64_t id;
  double value=0., confidence=0., raw_score=0.;
  int64_t last_stamp=0, retired_stamp=0;
  double base_value=0., base_confidence=0.;
  // Bounded, timestamp-ordered replay because confidence fusion is nonlinear.
  std::map<int64_t,std::pair<double,double>> evidence; // score, weight
};
struct FrontierSemanticNode {
  FrontierGeometry geometry;
  double value=0., confidence=0.;
  size_t scored_cells=0;
};
class FrontierSemanticMemory {
public:
  void sync(const std::vector<FrontierGeometry>& frontiers,
            const std::vector<FrontierCellGeometry>& geometry) {
    std::set<int> live;
    for (const auto& cell : geometry) {
      live.insert(cell.key);
      if (!keys_.count(cell.key)) {
        const uint64_t token=next_token_++;
        keys_[cell.key]=token;
        cells_.emplace(token,FrontierCellSemantic{cell,token});
      }
    }
    for (auto it=keys_.begin();it!=keys_.end();) {
      if (!live.count(it->first)) { cells_.erase(it->second); it=keys_.erase(it); }
      else ++it;
    }
    nodes_.clear();
    for (const auto& f : frontiers) nodes_.emplace(f.id,FrontierSemanticNode{f});
    aggregate();
  }
  size_t fuse(const std::vector<CellProjection>& projections,int64_t stamp,double score) {
    if (stamp<=0 || !std::isfinite(score) || score < -1. || score > 1.) return 0;
    size_t updated=0;
    for (const auto& p : projections) {
      auto it=cells_.find(p.id);
      if (it==cells_.end() || !std::isfinite(p.confidence) || p.confidence<=1e-9 || p.confidence>1.) continue;
      auto& c=it->second;
      if (stamp<=c.retired_stamp || c.evidence.count(stamp)) continue;
      c.evidence.emplace(stamp,std::make_pair(score,p.confidence));
      while (c.evidence.size()>20) {
        const auto first=c.evidence.begin();
        update(c.base_value,c.base_confidence,first->second.first,first->second.second);
        c.retired_stamp=first->first;
        c.evidence.erase(first);
      }
      c.value=c.base_value;c.confidence=c.base_confidence;
      for (const auto& e : c.evidence) update(c.value,c.confidence,e.second.first,e.second.second);
      c.last_stamp=c.evidence.rbegin()->first;c.raw_score=c.evidence.rbegin()->second.first;
      ++updated;
    }
    if (updated) aggregate();
    return updated;
  }
  uint64_t cellId(int key) const { return keys_.at(key); }
  const FrontierCellSemantic& cellAt(int key) const { return cells_.at(cellId(key)); }
  const std::map<uint64_t,FrontierCellSemantic>& cells() const { return cells_; }
  const std::map<int,FrontierSemanticNode>& nodes() const { return nodes_; }
private:
  static void update(double& v,double& c,double score,double w) {
    v=(w*score+c*v)/(w+c);
    c=(w*w+c*c)/(w+c);
  }
  void aggregate() {
    for (auto& entry : nodes_) {
      auto& node=entry.second;
      node.value=0.;node.confidence=0.;node.scored_cells=0;
      for (int key : node.geometry.cells) {
        const auto& c=cellAt(key);
        if (c.confidence<=0.) continue;
        if (!node.scored_cells || c.value>node.value) {node.value=c.value;node.confidence=c.confidence;}
        ++node.scored_cells;
      }
    }
  }
  uint64_t next_token_=1;
  std::map<int,uint64_t> keys_;
  std::map<uint64_t,FrontierCellSemantic> cells_;
  std::map<int,FrontierSemanticNode> nodes_;
};
} // namespace apexnav_planner
#endif
