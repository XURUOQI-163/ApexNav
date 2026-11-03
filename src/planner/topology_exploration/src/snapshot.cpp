#include <topology_exploration/snapshot.h>
#include <fstream>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>

namespace apexnav_planner::topology {
namespace fs = std::filesystem;
namespace {
void ensureDirectory(const fs::path &path) {
  if (fs::is_symlink(path)) throw std::runtime_error("snapshot directory is a symlink: " + path.string());
  fs::create_directories(path);
}
void atomicWrite(const fs::path &path, const char *data, size_t size) {
  auto tmp = path;
  tmp += ".tmp";
  if (fs::is_symlink(tmp) || fs::is_symlink(path)) throw std::runtime_error("snapshot file is a symlink");
  std::ofstream out;
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out.open(tmp, std::ios::binary | std::ios::trunc);
  out.write(data, size);
  out.close();
  fs::rename(tmp, path);
}
void number(std::ostream &out, double v) {
  if (std::isfinite(v)) out << v;
  else out << "null";
}
void point(std::ostream &out, sparse::Vec2 p) {
  out << '['; number(out, p.x); out << ','; number(out, p.y); out << ']';
}
void string(std::ostream &out, const std::string &s) {
  out << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    else out << c;
  }
  out << '"';
}
const char *stateName(sparse::ObservationState state) {
  switch (state) {
    case sparse::ObservationState::FREE: return "FREE";
    case sparse::ObservationState::OBSTACLE: return "OBSTACLE";
    case sparse::ObservationState::INVALID: return "INVALID";
    default: return "UNOBSERVED";
  }
}
void observations(std::ostream &out, const sparse::DepthDescriptor &d, bool fine) {
  const auto &bins = fine ? d.directions() : d.sectors();
  out << '[';
  for (size_t i = 0; i < bins.size(); ++i) {
    if (i) out << ',';
    const auto &s = bins[i];
    out << "\n        {\"index\":" << i << ",\"center_yaw_rad\":";
    number(out, sparse::wrap(fine ? d.directionCenter(i) : d.center(i)));
    out << ",\"state\":\"" << stateName(s.state) << "\",\"free_range_m\":";
    number(out, s.free_range);
    out << ",\"obstacle_distance_m\":"; number(out, s.obstacle_distance);
    out << ",\"stamp_ns\":" << s.stamp << ",\"valid_samples\":" << s.valid_samples
        << ",\"invalid_samples\":" << s.invalid_samples;
    if (!fine) out << ",\"valid_mask\":" << s.valid_mask;
    out << '}';
  }
  out << "\n      ]";
}
}
SnapshotWriter::SnapshotWriter(fs::path path) : directory_(fs::absolute(path).lexically_normal()) {
  // A dedicated, explicitly named directory bounds the new-run deletion scope.
  if (directory_.filename() != "topology_snapshot")
    throw std::invalid_argument("snapshot directory must end in topology_snapshot");
}
void SnapshotWriter::begin(int64_t stamp) {
  active_ = false;
  ensureDirectory(directory_);
  fs::remove(directory_ / "graph.json");
  fs::remove(directory_ / "graph.json.tmp");
  // remove_all does not follow a symlink at the images path.
  fs::remove_all(directory_ / "images");
  ensureDirectory(directory_ / "images");
  started_at_ = stamp;
  active_ = true;
  try { write(sparse::TopologyGraph{}, stamp, "STARTED"); }
  catch (...) { active_ = false; throw; }
}
void SnapshotWriter::write(const sparse::TopologyGraph &graph, int64_t stamp, const std::string &state) {
  if (!active_) return;
  ensureDirectory(directory_);
  ensureDirectory(directory_ / "images");
  std::set<fs::path> used;
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << std::boolalpha;
  out << "{\n  \"schema_version\":1,\n  \"frame_id\":\"world\",\n  \"started_at_ns\":" << started_at_
      << ",\n  \"captured_at_ns\":" << stamp << ",\n  \"planner_state\":";
  string(out, state);
  out << ",\n  \"current_waypoint\":" << graph.currentWaypoint()
      << ",\n  \"waypoint_count\":" << graph.waypointCount()
      << ",\n  \"frontier_count\":" << graph.frontierCount()
      << ",\n  \"edge_count\":" << graph.edges().size() << ",\n  \"nodes\":[";
  bool comma = false;
  for (const auto &[id, n] : graph.nodes()) {
    if (comma) out << ',';
    comma = true;
    const auto node_dir = directory_ / "images" / ("node_" + std::to_string(id));
    ensureDirectory(node_dir);
    used.insert(node_dir);
    out << "\n    {\"id\":" << id << ",\"type\":\""
        << (n.type == sparse::NodeType::WAYPOINT ? "WAYPOINT" : "FRONTIER")
        << "\",\"position_xy_m\":";
    point(out, n.position);
    out << ",\"source_id\":" << n.source_id << ",\"measured_at_node\":" << n.measured_at_node
        << ",\"stamp_ns\":" << n.stamp << ",\n      \"descriptor\":{\"origin_xy_m\":";
    point(out, n.descriptor.origin());
    out << ",\"stamp_ns\":" << n.descriptor.stamp() << ",\"max_distance_m\":";
    number(out, n.descriptor.maxDistance());
    out << ",\"sector_width_rad\":"; number(out, n.descriptor.step());
    out << ",\"fine_width_rad\":"; number(out, n.descriptor.step() / 6);
    out << ",\"sectors\":"; observations(out, n.descriptor, false);
    out << ",\"directions\":"; observations(out, n.descriptor, true);
    out << "},\n      \"images\":[";
    bool image_comma = false;
    for (const auto &[slot, img] : n.images) {
      if (!img.jpeg || img.jpeg->empty()) continue;
      const auto file = node_dir / ("slot_" + std::to_string(slot) + "_stamp_" + std::to_string(img.stamp) + ".jpg");
      if (fs::is_symlink(file)) throw std::runtime_error("snapshot image is a symlink");
      if (!fs::exists(file)) atomicWrite(file, reinterpret_cast<const char *>(img.jpeg->data()), img.jpeg->size());
      used.insert(file);
      if (image_comma) out << ',';
      image_comma = true;
      out << "\n        {\"slot\":" << slot << ",\"stamp_ns\":" << img.stamp << ",\"yaw_rad\":";
      number(out, img.yaw);
      out << ",\"camera_position_xyz_m\":[";
      number(out, img.camera_position.x); out << ','; number(out, img.camera_position.y); out << ',';
      number(out, img.camera_z);
      out << "],\"orientation_xyzw\":[";
      for (size_t i = 0; i < img.orientation.size(); ++i) {
        if (i) out << ',';
        number(out, img.orientation[i]);
      }
      out << "],\"path\":"; string(out, file.lexically_relative(directory_).generic_string());
      out << ",\"bytes\":" << img.jpeg->size() << '}';
    }
    out << "\n      ],\"pending_image_count\":" << n.pending_images.size() << '}';
  }
  out << "\n  ],\n  \"edges\":[";
  comma = false;
  for (const auto &[key, e] : graph.edges()) {
    if (comma) out << ',';
    comma = true;
    out << "\n    {\"a\":" << e.a << ",\"b\":" << e.b << ",\"weight_m\":";
    number(out, e.weight);
    out << ",\"verified\":" << e.verified << ",\"verified_at_ns\":" << e.verified_at
        << ",\"traversed\":" << e.traversed << ",\"path_xy_m\":[";
    for (size_t i = 0; i < e.path.size(); ++i) {
      if (i) out << ',';
      point(out, e.path[i]);
    }
    out << "]}";
  }
  out << "\n  ]\n}\n";
  const auto json = out.str();
  atomicWrite(directory_ / "graph.json", json.data(), json.size());
  // Only prune after publishing the new JSON, so its image references stay valid
  // even if a write failed or the process died before replacing the manifest.
  for (const auto &node : fs::directory_iterator(directory_ / "images")) {
    if (!used.count(node.path())) fs::remove_all(node.path());
    else for (const auto &file : fs::directory_iterator(node.path())) {
      if (!used.count(file.path())) fs::remove_all(file.path());
    }
  }
}
}
