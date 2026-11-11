#pragma once
#include <plan_env/skeleton_frontier_links.h>
namespace apexnav_planner::skeleton {
struct Route {
  std::vector<cv::Point2d> path;
  std::vector<uint64_t> edge_ids;
  double length=0;
  std::string reason;
};
// All geometry (including attachment) is revalidated against CURRENT map cells.
// Distances use polyline arc lengths; no Unknown association is traversed.
Route shortestRoute(const Graph& graph, cv::Point2d start, const FrontierLink& goal,
    const std::function<uint8_t(cv::Point)>& state, double connection_radius=2.0);
}
