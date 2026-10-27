#include <plan_env/semantic_observation_cache.h>
#include <cassert>
#include <limits>
#include <iostream>
using apexnav_planner::SemanticObservationCache;
// Observation is a stand-in for original camera pose and visible map cells.
struct Observation { int pose; int cell; };
int main() {
  SemanticObservationCache<Observation> c(3, 100, 0);
  assert(!c.addObservation(10, 10, {1, 11}));
  assert(!c.addObservation(20, 20, {2, 22}));
  auto delayed = c.addScore(10, 30, .7);
  assert(delayed && delayed->observation.pose == 1 && delayed->observation.cell == 11);
  assert(delayed->score == .7 && delayed->stamp == 10);
  assert(c.observation(10, 31)->cell == 11); // Detection can arrive after score.
  assert(!c.addScore(10, 31, .9)); // Same score cannot fuse twice.
  assert(!c.addObservation(10, 32, {9, 99}));
  assert(!c.addScore(40, 40, .2)); // Score before depth.
  auto early = c.addObservation(40, 41, {4, 44});
  assert(early && early->observation.cell == 44 && early->score == .2);
  assert(!c.addScore(20, 42, .8)); // Older than last fused result.
  assert(!c.observation(10, 200)); // Late detection cannot revive expired geometry.
  assert(!c.addObservation(50, 200, {5, 55})); // Expired.
  assert(!c.addScore(50, 200, .8));
  assert(!c.addScore(250, 240, .8)); // Future time.
  assert(!c.addScore(240, 240, std::numeric_limits<double>::quiet_NaN()));
  assert(!c.addScore(240, 240, 1.5));
  SemanticObservationCache<Observation> reset(3, 100, 500);
  assert(!reset.addObservation(490, 510, {1, 1})); // Previous episode.
  assert(!reset.addScore(490, 510, .2));
  assert(!reset.addObservation(501, 510, {1, 1}));
  assert(reset.addScore(501, 511, .4));
  SemanticObservationCache<Observation> bounded(2, 100, 0);
  bounded.addObservation(1,1,{1,1}); bounded.addObservation(2,2,{2,2});
  bounded.addObservation(3,3,{3,3});
  assert(bounded.size() == 2);
  assert(!bounded.addScore(1,4,.3)); // Evicted geometry never replaced by latest pose.
  assert(!bounded.addObservation(1,5,{1,1})); // Evicted stamps stay obsolete.
  assert(bounded.addScore(3,5,.4));
  std::cout << "semantic observation cache checks passed\n";
}
