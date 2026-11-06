#pragma once
#include <plan_env/skeleton_graph.h>
#include <functional>
namespace apexnav_planner::skeleton {
// Snapshot of current Frontier geometry only; IDs remain owned by FrontierMap2D.
struct Frontier {
  int id=0;
  bool active=true;
  cv::Point2d centroid;
  std::vector<cv::Point> cells;
};
struct FrontierLink {
  int frontier_id=0;
  bool active=true,connected=false;
  uint64_t junction_id=0;
  uint64_t geometry_signature=0;
  cv::Point2d centroid,approach;
  std::vector<cv::Point> pixels;
  std::vector<cv::Point2d> path;
  std::vector<cv::Point2d> display_path; // Checked, shortened Junction -> known-free approach ONLY.
  double length=0; // Safe path length; approach -> centroid is a non-traversable association.
  std::string reason;
};
uint64_t frontierFingerprint(const std::vector<Frontier>& frontiers);
std::vector<FrontierLink> connectFrontiers(const GridInput& input,const Graph& graph,const std::vector<Frontier>& frontiers,
                                         double max_association_distance=.5);
bool discardStaleLinks(std::vector<FrontierLink>& links,const std::vector<Frontier>& current);
// Every segment is checked against current cells, including corner-touch cells.
// Only the final cell may be UNKNOWN, and only if it belongs to this Frontier.
bool frontierPathTraversable(const Geometry& geometry,const std::vector<cv::Point2d>& path,
    const std::function<uint8_t(cv::Point)>& state,bool endpoint_is_frontier);
// Drawing/association check only: Unknown is allowed, obstacles/inflation are not.
bool frontierAssociationClear(const Geometry& geometry,cv::Point2d approach,cv::Point2d centroid,
    const std::function<uint8_t(cv::Point)>& state);
}
