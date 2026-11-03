#include <topology_exploration/rolling_local_map.h>
#include <cassert>
#include <iostream>
using namespace apexnav_planner::topology;
int main(int argc, char * *argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rolling_map_test");
  RollingLocalMap map(node);
  assert(map.capacity() == 14400);
  map.reset({0, 0});
  assert(map.occupancy({4, 0}) == Cell::UNKNOWN);
  map.seedFootprint({0, 0});
  assert(map.safe({0, 0}));
  map.observe({0, 0}, {{{2, 0}, true}, {{2, .2}, true}});
  assert(map.occupancy({2, 0}) == Cell::OCCUPIED);
  assert(!map.safe({1.9, 0}));
  // New depth hits must stop execution immediately, including on cells made
  // confidently free by many past scans (not just UNKNOWN-to-hit transitions).
  for (int i = 0; i < 30; ++i) {
    map.observe({0, 0}, {{{2.5, 1}, false}});
  }
  assert(map.occupancy({1.25, .5}) == Cell::FREE);
  map.observe({0, 0}, {{{1.25, .5}, true}});
  assert(map.occupancy({1.25, .5}) == Cell::OCCUPIED);
  map.moveTo({1, 0});
  assert(map.capacity() == 14400 && map.occupancy({2, 0}) == Cell::OCCUPIED);
  map.moveTo({10, 0});
  assert(map.occupancy({2, 0}) == Cell::UNKNOWN && map.occupancy({10, 0}) == Cell::UNKNOWN);
  map.moveTo({0, 0});
  assert(map.occupancy({2, 0}) == Cell::UNKNOWN);
  std::cout <<
  "PASS rolling map: 14400 cells, overlap preserved, dropped cells forgotten, footprint checked\n";
  rclcpp::shutdown();
}
