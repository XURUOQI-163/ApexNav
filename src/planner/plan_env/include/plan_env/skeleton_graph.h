#pragma once
// Geometry-only graph from a temporary occupancy snapshot; no ROS/model dependency.
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>
namespace apexnav_planner::skeleton {
enum Cell : uint8_t { UNKNOWN=0, BLOCKED=1, FREE=2, INFLATED=3 };
enum class NodeType : uint8_t { ENDPOINT=0, JUNCTION=1, LOOP_ANCHOR=2 };
enum class VisitState : uint8_t { CANDIDATE=0, CONFIRMED=1 };
struct Geometry {
  int width=0,height=0;
  double resolution=0,origin_x=0,origin_y=0,yaw=0;
  std::string frame_id;
  cv::Point2d world(cv::Point pixel) const;
  bool operator==(const Geometry& other) const;
};
struct GridInput {
  Geometry geometry;
  cv::Mat cells; // rows=y, columns=x; temporary classification only
};
struct Options {
  double min_branch_length=.3,unknown_boundary_distance=.5,node_match_distance=.4;
  double min_deadend_width=.2; // 0 disables width pruning; diameter after inflation
  double deadend_width_sample_fraction=.5; // Terminal fraction, measured from Endpoint
  double min_island_area=0; // m^2; discard smaller disconnected free regions, always keep largest; 0 disables
  double junction_merge_length=.5; // Direct Junction-Junction edges strictly shorter than this; 0 disables
  double visit_radius=.5; // Robot-base proximity required for Junction confirmation
  bool remove_deadends=false;
};
struct Node {
  uint64_t id=0;
  NodeType type=NodeType::ENDPOINT;
  cv::Point pixel;
  cv::Point2d position;
  std::vector<uint64_t> neighbors;
  uint32_t neighbor_count=0;
  double clearance=0;
  bool near_unknown_boundary=false,active=true;
  VisitState visit_state=VisitState::CANDIDATE; // Only applicable to JUNCTION
  int64_t confirmed_at_ns=0;
  cv::Point2d visit_position; // Actual robot-base position, not camera pose
};
struct Edge {
  uint64_t id=0,start=0,end=0;
  std::vector<cv::Point> pixels; // Cells containing path points; merged-centre connectors may repeat cells
  std::vector<cv::Point2d> path;
  std::vector<cv::Point2d> display_path; // Same geometry as path, including merged-centre connectors.
  double length=0,min_clearance=0;
};
struct Graph {
  Geometry geometry;
  std::vector<Node> nodes;
  std::vector<Edge> edges;
  uint64_t next_node_id=1,next_edge_id=1;
  uint32_t skeleton_pixels=0,pruned_branches=0,removed_deadends=0;
};
Graph build(const GridInput& input,const Options& options={},const Graph& previous={});
bool confirmVisitedJunctions(Graph& graph,cv::Point2d robot,double radius,int64_t stamp_ns);
void inheritJunctionVisits(Graph& graph,const Graph& previous,double radius);
const char* thinningBackend();
struct Fingerprint { Geometry geometry;std::vector<uint64_t> tiles; };
Fingerprint fingerprint(const GridInput& input,int tile_size);
// Only hashes persist: this is not a second occupancy map. Compare to last built
// snapshot (not last poll), so individually small changes accumulate.
class ChangeGate {
public:
  ChangeGate(int tile_size,int min_changed_tiles,double min_changed_ratio);
  bool shouldRebuild(const Fingerprint& current) const;
  size_t changedTiles(const Fingerprint& current) const;
  void accept(Fingerprint current) { baseline_=std::move(current); }
private:
  int min_tiles_;
  double min_ratio_;
  Fingerprint baseline_;
};
}
