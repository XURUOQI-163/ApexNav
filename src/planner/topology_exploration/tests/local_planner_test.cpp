#include <topology_exploration/local_planner.h>
#include <topology_exploration/controller_guard.h>
#include <cassert>
#include <iostream>
using namespace apexnav_planner::topology;
int main(){
  std::vector<Vec2> route{{0, 0}, {1, 1}, {2, 0}, {9, 0}};
  size_t cursor = 1;
  auto leg = LocalPlanner::nextLeg(route, {0, 0}, cursor);
  assert(leg.size() == 2 && leg.back().distance({1, 1}) < 1e-9 && cursor == 1);
  leg = LocalPlanner::nextLeg(route, {1, 1}, cursor);
  assert(leg.back().distance({2, 0}) < 1e-9 && cursor == 2);
  leg = LocalPlanner::nextLeg(route, {2, 0}, cursor);
  assert(leg.back().distance({9, 0}) < 1e-9 && cursor == 3);
  leg = LocalPlanner::nextLeg(route, {3.5, 0}, cursor);
  assert(leg.back().distance({9, 0}) < 1e-9 && cursor == 3);
  GridView grid;
  grid.n = 80;
  grid.resolution = .05;
  grid.origin = {-2, -2};
  grid.safe = [](Vec2 p){
                return p.x > -1.9 && p.x < 1.9 && p.y > -1.9 && p.y < 1.9 &&
                       !(std::abs(p.x) < .15 && p.y < .7);
              };
  LocalPlanner planner;
  auto path = planner.plan(grid, {-1, 0}, {1, 0});
  assert(path.complete && path.points.size() > 2 && path.cost > 2.3);
  auto none = planner.plan(grid, {-1, 0}, {.1, 0});
  assert(!none.complete && none.points.empty());
  std::vector<Vec2> prefix_route{{-1, 0}, {-1, 1}, {1, 1}, {3, 1}};
  auto prefix = planner.orderedPrefix(grid, {-1, 0}, prefix_route);
  assert(prefix.size() > 2 && prefix.back().x < 1.9);
  GuardConfig cfg;
  ControllerGuard guard(cfg);
  auto free = [](Vec2 p){
                return p.x < .5 && std::abs(p.y) < 1;
              };
  auto command = guard.limitAndCheck({.4, 1.}, {0, 0}, 0, .05, free, true);
  assert(command.v <= .025 + 1e-9 && command.w <= .05 + 1e-9);
  auto stopped = guard.limitAndCheck({.4, 1.}, {0, 0}, 0, .05, free, false);
  assert(stopped.v == 0 && stopped.w == 0);
  assert(guard.last().v == 0);
  for (int i = 0; i < 40; ++i) {
    command = guard.limitAndCheck({.4, 0}, {0, 0}, 0, .05, free, true);
  }
  command = guard.limitAndCheck({.4, 0}, {.48, 0}, 0, .05, free, true);
  assert(command.v == 0);
  std::cout <<
  "PASS local A*: wall detour and ordered prefix; control rate limits, braking and stale-data stop\n";
}
