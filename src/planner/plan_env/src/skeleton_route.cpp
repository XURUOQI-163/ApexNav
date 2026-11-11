#include <plan_env/skeleton_route.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
namespace apexnav_planner::skeleton {
namespace {
bool finite(cv::Point2d p){return std::isfinite(p.x)&&std::isfinite(p.y);}
double length(const std::vector<cv::Point2d>& p){double d=0;for(size_t i=1;i<p.size();++i)d+=cv::norm(p[i]-p[i-1]);return d;}
void append(std::vector<cv::Point2d>& out,cv::Point2d p){if(out.empty()||cv::norm(p-out.back())>1e-9)out.push_back(p);}
struct Arc {int from,to;uint64_t edge;double cost;std::vector<cv::Point2d> path;};
}
Route shortestRoute(const Graph& graph,cv::Point2d start,const FrontierLink& goal,
    const std::function<uint8_t(cv::Point)>& state,double radius) {
  Route result;
  auto fail=[&](const char* reason){result.reason=reason;return result;};
  if(graph.nodes.empty()||!std::isfinite(graph.geometry.resolution)||graph.geometry.resolution<=0)
    return fail("GRAPH_NOT_READY");
  if(!finite(start)||!std::isfinite(radius)||radius<=0)return fail("INVALID_REQUEST");
  auto safe=[&](const std::vector<cv::Point2d>& path){
    return !path.empty()&&std::all_of(path.begin(),path.end(),finite)&&
      frontierPathTraversable(graph.geometry,path,state,false);
  };
  if(!safe({start}))return fail("START_NOT_FREE");
  std::map<uint64_t,int> ids;
  std::vector<cv::Point2d> positions;
  for(const auto& n:graph.nodes)if(n.active&&finite(n.position)) {
    if(ids.count(n.id))return fail("DUPLICATE_NODE_ID");
    ids[n.id]=positions.size();positions.push_back(n.position);
  }
  // The verified shortened geometry starts at the exact arithmetic centre;
  // its raster witness uses cell centres and can differ by half a cell.
  const auto& goal_path=goal.display_path.empty()?goal.path:goal.display_path;
  if(!goal.connected||!ids.count(goal.junction_id)||goal_path.empty()||
     cv::norm(goal_path.front()-positions[ids.at(goal.junction_id)])>1e-6||
     cv::norm(goal_path.back()-goal.approach)>1e-6||!safe(goal_path))return fail("INVALID_FRONTIER_ATTACHMENT");
  const int source=positions.size(),target=source+1;
  std::vector<std::vector<int>> adj(positions.size()+2);
  std::vector<Arc> arcs;
  auto add=[&](int a,int b,std::vector<cv::Point2d> path,uint64_t eid) {
    double cost=length(path);int index=arcs.size();
    arcs.push_back({a,b,eid,cost,std::move(path)});adj[a].push_back(index);adj[b].push_back(index);
  };
  for(const auto& [id,index]:ids)if(cv::norm(start-positions[index])<=radius&&safe({start,positions[index]}))
    add(source,index,{start,positions[index]},0);
  for(const auto& e:graph.edges) {
    if(!ids.count(e.start)||!ids.count(e.end)||e.path.size()<2)continue;
    int a=ids.at(e.start),b=ids.at(e.end);
    if(cv::norm(e.path.front()-positions[a])>1e-6||cv::norm(e.path.back()-positions[b])>1e-6||!safe(e.path))continue;
    // A virtual attachment splits an edge without adding all raster pixels as nodes.
    double best=std::numeric_limits<double>::infinity();size_t segment=0;cv::Point2d attach;
    for(size_t i=1;i<e.path.size();++i) {
      auto d=e.path[i]-e.path[i-1];double norm=d.dot(d);if(norm<=1e-18)continue;
      double t=std::clamp((start-e.path[i-1]).dot(d)/norm,0.,1.);
      auto p=e.path[i-1]+t*d;double distance=cv::norm(start-p);
      if(distance<=radius&&distance<best&&safe({start,p})){best=distance;segment=i;attach=p;}
    }
    if(!segment){add(a,b,e.path,e.id);continue;}
    int joint=adj.size();adj.emplace_back();
    std::vector<cv::Point2d> left,right;
    for(size_t i=0;i<segment;++i)append(left,e.path[i]);
    append(left,attach);
    append(right,attach);for(size_t i=segment;i<e.path.size();++i)append(right,e.path[i]);
    add(a,joint,std::move(left),e.id);add(joint,b,std::move(right),e.id);
    add(source,joint,{start,attach},0);
  }
  add(ids.at(goal.junction_id),target,goal_path,0);
  if(adj[source].empty())return fail("NO_LOCAL_GRAPH_CONNECTION");
  const double inf=std::numeric_limits<double>::infinity();
  std::vector<double> distances(adj.size(),inf);
  std::vector<int> parent(adj.size(),-1),via(adj.size(),-1);
  using Item=std::pair<double,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
  distances[source]=0;open.emplace(0,source);
  while(!open.empty()) {
    auto [cost,u]=open.top();open.pop();if(cost>distances[u])continue;if(u==target)break;
    for(int index:adj[u]) {
      const auto& arc=arcs[index];int v=arc.from==u?arc.to:arc.from;
      double next=cost+arc.cost;if(next>=distances[v]-1e-12)continue;
      distances[v]=next;parent[v]=u;via[v]=index;open.emplace(next,v);
    }
  }
  if(!std::isfinite(distances[target]))return fail("DISCONNECTED_GRAPH");
  std::vector<int> route;for(int v=target;v!=source;v=parent[v])route.push_back(v);
  std::reverse(route.begin(),route.end());
  for(int v:route) {
    const auto& arc=arcs[via[v]];auto path=arc.path;
    if(arc.to!=v)std::reverse(path.begin(),path.end());
    for(auto p:path)append(result.path,p);
    if(arc.edge&&(result.edge_ids.empty()||result.edge_ids.back()!=arc.edge))result.edge_ids.push_back(arc.edge);
  }
  // Preserve every polyline corner, densifying long local connectors for lookahead.
  auto sparse=std::move(result.path);result.path.clear();append(result.path,sparse.front());
  for(size_t i=1;i<sparse.size();++i) {
    int steps=std::max(1,int(std::ceil(cv::norm(sparse[i]-sparse[i-1])/graph.geometry.resolution)));
    for(int j=1;j<=steps;++j)append(result.path,sparse[i-1]+(double(j)/steps)*(sparse[i]-sparse[i-1]));
  }
  result.length=length(result.path);return result;
}
}
