#include <topology_exploration/snapshot.h>
#include <cassert>
#include <fstream>
#include <iostream>
using namespace apexnav_planner::topology;
namespace sparse = apexnav_planner::sparse;
namespace fs = std::filesystem;
std::string read(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), {}};
}
int main(int argc, char **argv) {
  assert(argc == 2);
  fs::path root = fs::path(argv[1]) / "topology_snapshot";
  sparse::TopologyGraph graph;
  sparse::DepthDescriptor d;
  for (double a = -.65; a <= .65; a += .008) d.addRay(a, 5, true, false, 10);
  d.finish(10);
  graph.update({}, 0, 10, d, [](sparse::Vec2) { return true; });
  sparse::DirectionalImage image;
  image.stamp = 10;
  image.camera_z = .88;
  image.jpeg = std::make_shared<const std::vector<unsigned char>>(std::vector<unsigned char>{1, 2, 3});
  graph.attachImage(image);
  SnapshotWriter writer(root);
  writer.begin(9);
  writer.write(graph, 11, "TRACK");
  const auto saved = read(root / "graph.json");
  std::ofstream(fs::path(argv[1]) / "fixture_graph.json") << saved;
  assert(saved.find("FRONTIER") != std::string::npos && fs::exists(root / "images/node_1/slot_0_stamp_10.jpg"));
  // If publishing the new manifest fails, all old image references must survive.
  fs::create_directory(root / "graph.json.tmp");
  bool failed = false;
  try { writer.write(graph, 12, "TRACK"); } catch (const std::exception &) { failed = true; }
  assert(failed && read(root / "graph.json") == saved);
  assert(fs::exists(root / "images/node_1/slot_0_stamp_10.jpg"));
  fs::remove(root / "graph.json.tmp");
  writer.end();
  graph.reset();
  writer.write(graph, 12, "WAIT_DATA");
  assert(read(root / "graph.json") == saved); // An inactive writer never overwrites disk.
  SnapshotWriter restarted(root);
  assert(read(root / "graph.json") == saved); // Planner restart alone is not a new simulation.
  restarted.begin(20);
  assert(read(root / "graph.json") != saved);
  assert(fs::is_empty(root / "images"));
  graph.update({}, 0, 30, d, [](sparse::Vec2) { return true; });
  image.stamp = 30;
  graph.attachImage(image);
  restarted.write(graph, 31, "TRACK");
  auto removed = graph.nodes().rbegin()->first;
  assert(removed != 1 && fs::exists(root / "images" / ("node_" + std::to_string(removed))));
  graph.erase(removed);
  restarted.write(graph, 32, "TRACK");
  assert(!fs::exists(root / "images" / ("node_" + std::to_string(removed))));
  std::ofstream(fs::path(argv[1]) / "pruned_graph.json") << read(root / "graph.json");
  std::cout << "PASS snapshot lifecycle: retained on exit/restart, cleared on new simulation\n";
}
