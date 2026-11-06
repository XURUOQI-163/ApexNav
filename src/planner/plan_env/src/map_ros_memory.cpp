#include <plan_env/map_ros.h>

namespace apexnav_planner {
// Default mutually-exclusive callback group: coherent with depth/fusion updates.
// Work is proportional to CURRENT frontier cells, never the dense global map.
void MapROS::publishFrontierMemory()
{
  if (!frontier_memory_pub_->get_subscription_count() || last_depth_stamp_ <= 0) return;
  plan_env::msg::FrontierMemoryState state;
  state.header.frame_id = frame_id_;
  state.header.stamp = node_->get_clock()->now();
  state.map_epoch_ns = map_start_time_.nanoseconds();
  state.last_depth_stamp_ns = last_depth_stamp_;
  state.backend = semantic_backend_;
  for (const auto& entry : map_->value_map_->nodes()) {
    const auto& source = entry.second;
    plan_env::msg::FrontierMemoryNode node;
    node.frontier_id = source.geometry.id;
    node.active = source.geometry.active;
    node.centroid.x = source.geometry.x;
    node.centroid.y = source.geometry.y;
    node.fused_score = source.value;
    node.confidence = source.confidence;
    node.scored_cells = source.scored_cells;
    for (int key : source.geometry.cells) {
      const auto& source_cell = map_->value_map_->cellAt(key);
      plan_env::msg::FrontierMemoryCell cell;
      cell.token = source_cell.id;
      cell.pixel_x = source_cell.geometry.ix;
      cell.pixel_y = source_cell.geometry.iy;
      cell.x = source_cell.geometry.x;
      cell.y = source_cell.geometry.y;
      cell.fused_score = source_cell.value;
      cell.confidence = source_cell.confidence;
      cell.raw_similarity = source_cell.raw_score;
      cell.last_frame_stamp_ns = source_cell.last_stamp;
      node.cells.push_back(std::move(cell));
    }
    state.nodes.push_back(std::move(node));
  }
  frontier_memory_pub_->publish(std::move(state));
}
}  // namespace apexnav_planner
