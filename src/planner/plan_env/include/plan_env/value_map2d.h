#ifndef _VALUE_MAP_H_
#define _VALUE_MAP_H_
#include <plan_env/frontier_semantic_memory.h>
#include <plan_env/sdf_map2d.h>
namespace apexnav_planner {
// Sparse semantics only for currently existing frontier cells.
class ValueMap {
public:
  ValueMap(SDFMap2D*,rclcpp::Node::SharedPtr) {}
  void syncFrontiers(const std::vector<FrontierGeometry>& nodes,
                     const std::vector<FrontierCellGeometry>& cells) { memory_.sync(nodes,cells); }
  size_t fuse(const std::vector<CellProjection>& cells,int64_t stamp,double score) {
    return memory_.fuse(cells,stamp,score);
  }
  double getValue(const Eigen::Vector2d& frontier) const;
  const auto& nodes() const { return memory_.nodes(); }
  const auto& cells() const { return memory_.cells(); }
  const auto& cellAt(int key) const { return memory_.cellAt(key); }
private:
  FrontierSemanticMemory memory_;
};
}
#endif
