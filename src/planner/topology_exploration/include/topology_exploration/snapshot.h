#pragma once
#include <plan_env/topology_graph.h>
#include <filesystem>

namespace apexnav_planner::topology {
// Called serially by the node/background job. Construction never modifies disk.
class SnapshotWriter {
public:
  explicit SnapshotWriter(std::filesystem::path directory);
  void begin(int64_t stamp);
  void write(const sparse::TopologyGraph &graph, int64_t stamp, const std::string &state);
  void end() { active_ = false; }
  bool active() const { return active_; }
  const std::filesystem::path &directory() const { return directory_; }
private:
  std::filesystem::path directory_;
  int64_t started_at_ = 0;
  bool active_ = false;
};
}
