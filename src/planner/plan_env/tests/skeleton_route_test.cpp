#include <plan_env/skeleton_route.h>
#include <cassert>
#include <cmath>
#include <iostream>
using namespace apexnav_planner::skeleton;
Node node(uint64_t id,double x,double y){Node n;n.id=id;n.position={x,y};n.active=true;return n;}
Edge edge(uint64_t id,uint64_t a,uint64_t b,std::vector<cv::Point2d> p){Edge e;e.id=id;e.start=a;e.end=b;e.path=p;return e;}
int main(){
  Graph g;g.geometry={100,100,.1,0,0,0,"world"};
  g.nodes={node(1,1.05,1.05),node(2,3.05,1.05),node(3,3.05,3.05)};
  g.edges={edge(11,1,2,{{1.05,1.05},{2.05,1.05},{3.05,1.05}}),
    edge(12,3,2,{{3.05,3.05},{3.05,2.05},{3.05,1.05}}),
    edge(13,1,3,{{1.05,1.05},{1.05,5.05},{3.05,5.05},{3.05,3.05}})};
  FrontierLink goal;goal.connected=true;goal.junction_id=3;goal.approach={3.55,3.05};
  goal.path={{3.05,3.05},goal.approach};
  auto free=[](cv::Point)->uint8_t{return FREE;};
  auto r=shortestRoute(g,{1.05,1.05},goal,free,.2);
  assert(r.reason.empty()&&std::abs(r.length-4.5)<1e-6);
  assert((r.edge_ids==std::vector<uint64_t>{11,12}));
  assert(cv::norm(r.path.back()-goal.approach)<1e-9);
  // Starting inside an edge must not detour to its farther endpoint.
  r=shortestRoute(g,{2.05,1.15},goal,free,.2);
  assert(r.reason.empty()&&std::abs(r.length-3.6)<1e-6);
  // Parallel routes retain their own geometry/cost, even if straight distance ties.
  g.edges.push_back(edge(14,1,3,{{1.05,1.05},{2.05,2.05},{3.05,3.05}}));
  r=shortestRoute(g,{1.05,1.05},goal,free,.05);
  assert((r.edge_ids==std::vector<uint64_t>{14}));
  auto blocked=[](cv::Point p)->uint8_t{return p==cv::Point(20,20)?BLOCKED:FREE;};
  r=shortestRoute(g,{1.05,1.05},goal,blocked,.05);
  assert((r.edge_ids==std::vector<uint64_t>{11,12}));
  auto unknown=[](cv::Point p)->uint8_t{return p==cv::Point(35,30)?UNKNOWN:FREE;};
  assert(!shortestRoute(g,{1.05,1.05},goal,unknown).reason.empty());
  assert(!shortestRoute(g,{8.05,8.05},goal,free,.2).reason.empty());
  // Disconnected valid components cannot be bridged across open map space.
  auto detached=g;detached.edges.clear();
  assert(shortestRoute(detached,{1.05,1.05},goal,free,.1).reason=="DISCONNECTED_GRAPH");
  auto invalid=goal;invalid.path={{3.05,3.05},{3.55,4.05}};
  assert(!shortestRoute(g,{1.05,1.05},invalid,free).reason.empty());
  // Closed edges work when joining their interior: choose the shorter arc.
  Graph ring;ring.geometry=g.geometry;ring.nodes={node(1,1.05,1.05)};
  ring.edges={edge(20,1,1,{{1.05,1.05},{3.05,1.05},{3.05,3.05},{1.05,3.05},{1.05,1.05}})};
  auto ring_goal=goal;ring_goal.junction_id=1;ring_goal.approach={1.05,1.05};ring_goal.path={ring_goal.approach};
  auto rr=shortestRoute(ring,{2.05,1.05},ring_goal,free,.1);
  assert(rr.reason.empty()&&std::abs(rr.length-1.)<1e-6);
  // Neither obstacles nor Unknown may be crossed by the start connector.
  auto blocked_start=[](cv::Point p)->uint8_t{return p==cv::Point(20,11)?INFLATED:FREE;};
  assert(shortestRoute(g,{2.05,1.15},goal,blocked_start,.2).reason=="START_NOT_FREE");
  goal.connected=false;assert(!shortestRoute(g,{1.05,1.05},goal,free).reason.empty());
  assert(!shortestRoute(Graph{}, {1,1},goal,free).reason.empty());
  // A merged mean lies off the raster centre. Use its verified geometric
  // Frontier attachment, not the rounded cell-centre witness.
  Graph mean;mean.geometry=g.geometry;mean.nodes={node(3,3.07,3.06)};
  auto mean_goal=goal;mean_goal.connected=true;
  mean_goal.display_path={mean.nodes[0].position,mean_goal.approach};
  auto mean_route=shortestRoute(mean,mean.nodes[0].position,mean_goal,free,.2);
  assert(mean_route.reason.empty()&&cv::norm(mean_route.path.front()-mean.nodes[0].position)<1e-9);
  std::cout<<"PASS skeleton shortest path, geometry, reversal, edge-interior start, obstacles and fallback reasons\n";
}
