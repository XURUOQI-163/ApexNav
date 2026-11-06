#include <plan_env/skeleton_frontier_links.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <stdexcept>
namespace apexnav_planner::skeleton {
namespace {
cv::Point2d gridPosition(const Geometry& g,cv::Point2d p) {
  p-=cv::Point2d(g.origin_x,g.origin_y);
  return {(std::cos(g.yaw)*p.x+std::sin(g.yaw)*p.y)/g.resolution,
          (-std::sin(g.yaw)*p.x+std::cos(g.yaw)*p.y)/g.resolution};
}
cv::Point gridCell(const Geometry& g,cv::Point2d p) {
  auto q=gridPosition(g,p);return {int(std::floor(q.x)),int(std::floor(q.y))};
}
// Exact grid traversal, including both side cells when a ray touches a corner.
// Sampling a few points along a segment can miss a thin wall or corner.
bool visible(const Geometry& g,cv::Point2d a,cv::Point2d b,
             const std::function<uint8_t(cv::Point)>& state,bool terminal_unknown,
             std::vector<cv::Point>* raster=nullptr) {
  auto start=gridPosition(g,a),end=gridPosition(g,b),delta=end-start;
  auto p=gridCell(g,a),goal=gridCell(g,b);
  auto allowed=[&](cv::Point q) {
    if(q.x<0||q.y<0||q.x>=g.width||q.y>=g.height)return false;
    auto value=state(q);return value==FREE||(terminal_unknown&&q==goal&&value==UNKNOWN);
  };
  bool vertical=std::abs(delta.x)<1e-12&&std::abs(start.x-std::round(start.x))<1e-10;
  bool horizontal=std::abs(delta.y)<1e-12&&std::abs(start.y-std::round(start.y))<1e-10;
  auto checked=[&](cv::Point q) {
    return allowed(q)&&(!vertical||allowed(q+cv::Point(-1,0)))&&
           (!horizontal||allowed(q+cv::Point(0,-1)));
  };
  if(!checked(p)||!checked(goal))return false;
  if(raster){raster->clear();if(state(p)==FREE)raster->push_back(p);}
  int sx=(delta.x>0)-(delta.x<0),sy=(delta.y>0)-(delta.y<0);
  const double inf=std::numeric_limits<double>::infinity();
  double tx=sx?((sx>0?p.x+1:p.x)-start.x)/delta.x:inf;
  double ty=sy?((sy>0?p.y+1:p.y)-start.y)/delta.y:inf;
  double step_x=sx?1./std::abs(delta.x):inf,step_y=sy?1./std::abs(delta.y):inf;
  for(int steps=0;p!=goal&&steps<=g.width+g.height+2;++steps) {
    // Do not step beyond an endpoint lying exactly on a grid boundary.
    double nx=p.x==goal.x?inf:tx,ny=p.y==goal.y?inf:ty;
    if(std::abs(nx-ny)<1e-12) {
      if(!checked(p+cv::Point(sx,0))||!checked(p+cv::Point(0,sy)))return false;
      p+=cv::Point(sx,sy);tx+=step_x;ty+=step_y;
    } else if(nx<ny){p.x+=sx;tx+=step_x;}
    else {p.y+=sy;ty+=step_y;}
    if(!checked(p))return false;
    if(raster&&state(p)==FREE)raster->push_back(p);
  }
  return p==goal;
}
double pathLength(const std::vector<cv::Point2d>& path) {
  double out=0;for(size_t i=1;i<path.size();++i)out+=cv::norm(path[i]-path[i-1]);return out;
}
std::vector<cv::Point2d> shorten(const Geometry& g,const std::vector<cv::Point2d>& path,
    const std::function<uint8_t(cv::Point)>& state,bool terminal_unknown) {
  if(path.empty())return {};
  std::vector<cv::Point2d> out{path.front()};size_t i=0;
  while(i+1<path.size()) {
    size_t next=path.size()-1;
    while(next>i&&!visible(g,path[i],path[next],state,terminal_unknown&&next+1==path.size()))--next;
    if(next==i)return {};
    if(cv::norm(out.back()-path[next])>1e-9)out.push_back(path[next]);
    i=next;
  }
  return out;
}
}
bool frontierPathTraversable(const Geometry& g,const std::vector<cv::Point2d>& path,
    const std::function<uint8_t(cv::Point)>& state,bool endpoint_is_frontier) {
  if(path.empty())return false;
  if(path.size()==1)return visible(g,path.front(),path.back(),state,false);
  for(size_t i=1;i<path.size();++i)
    if(!visible(g,path[i-1],path[i],state,endpoint_is_frontier&&i+1==path.size()))return false;
  return true;
}
bool frontierAssociationClear(const Geometry& g,cv::Point2d approach,cv::Point2d centroid,
    const std::function<uint8_t(cv::Point)>& state) {
  return visible(g,approach,centroid,[&](cv::Point p)->uint8_t {
    auto value=state(p);return value==UNKNOWN?FREE:value;
  },false);
}
uint64_t frontierFingerprint(const std::vector<Frontier>& frontiers) {
  std::vector<uint64_t> hashes;
  for(const auto& f:frontiers) {
    uint64_t h=14695981039346656037ULL;
    auto add=[&](uint64_t x){for(int i=0;i<8;++i){h=(h^(x&255))*1099511628211ULL;x>>=8;}};
    add(f.id);add(f.active);
    uint64_t bits;std::memcpy(&bits,&f.centroid.x,sizeof bits);add(bits);std::memcpy(&bits,&f.centroid.y,sizeof bits);add(bits);
    std::vector<std::pair<int,int>> cells;for(auto p:f.cells)cells.emplace_back(p.x,p.y);
    std::sort(cells.begin(),cells.end());for(auto [x,y]:cells){add(x);add(y);}
    hashes.push_back(h);
  }
  std::sort(hashes.begin(),hashes.end());uint64_t out=14695981039346656037ULL;
  for(auto h:hashes)out=(out^h)*1099511628211ULL;
  return out;
}
std::vector<FrontierLink> connectFrontiers(const GridInput& in,const Graph& graph,const std::vector<Frontier>& frontiers,
                                         double max_association_distance) {
  if(in.cells.type()!=CV_8UC1||in.cells.cols!=in.geometry.width||in.cells.rows!=in.geometry.height||
     !std::isfinite(in.geometry.resolution)||in.geometry.resolution<=0||!(in.geometry==graph.geometry))
    throw std::invalid_argument("Frontier link map geometry mismatch");
  if(!std::isfinite(max_association_distance)||max_association_distance<=0)
    throw std::invalid_argument("Frontier association distance must be positive and finite");
  std::vector<FrontierLink> links;
  if(frontiers.empty())return links;
  auto state=[&](cv::Point p)->uint8_t{return p.x>=0&&p.y>=0&&p.x<in.cells.cols&&p.y<in.cells.rows?in.cells.at<uint8_t>(p):BLOCKED;};
  auto free=[&](cv::Point p){return state(p)==FREE;};
  cv::Mat mask=(in.cells==FREE);cv::Rect bounds=cv::boundingRect(mask);
  struct Source {const Node* node;cv::Point pixel;};
  std::vector<Source> sources;
  for(const auto& n:graph.nodes)if(n.active&&n.type==NodeType::JUNCTION) {
    auto p=gridCell(in.geometry,n.position);
    if(free(p))sources.push_back({&n,p}); // Route from the CURRENT centre, never a retired anchor.
  }
  std::sort(sources.begin(),sources.end(),[](const auto& a,const auto& b){
    return std::tie(a.node->id,a.pixel.y,a.pixel.x)<std::tie(b.node->id,b.pixel.y,b.pixel.x);
  });
  int w=bounds.width,h=bounds.height;
  auto index=[&](cv::Point p){return (p.y-bounds.y)*w+p.x-bounds.x;};
  auto point=[&](int idx){return cv::Point(idx%w+bounds.x,idx/w+bounds.y);};
  const double inf=std::numeric_limits<double>::infinity();
  std::vector<double> distance;std::vector<int> parent,owner;
  if(!sources.empty()) {
    distance.assign(size_t(w)*h,inf);parent.assign(size_t(w)*h,-1);owner.assign(size_t(w)*h,-1);
    using Item=std::tuple<double,int,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    for(size_t i=0;i<sources.size();++i) {
      int p=index(sources[i].pixel);double cost=cv::norm(sources[i].node->position-in.geometry.world(sources[i].pixel));
      if(cost>=distance[p])continue;
      distance[p]=cost;parent[p]=p;owner[p]=i;queue.emplace(cost,i,p);
    }
    while(!queue.empty()) {
      auto [cost,source,idx]=queue.top();queue.pop();if(cost>distance[idx]||source!=owner[idx])continue;
      auto p=point(idx);
      for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
        if(!(dx||dy))continue;
        auto q=p+cv::Point(dx,dy);if(!free(q))continue;
        if(dx&&dy&&(!free(p+cv::Point(dx,0))||!free(p+cv::Point(0,dy))))continue;
        int next=index(q);double value=cost+std::hypot(dx,dy)*in.geometry.resolution;
        if(value<distance[next]-1e-12||(std::abs(value-distance[next])<1e-12&&source<owner[next])) {
          distance[next]=value;owner[next]=source;parent[next]=idx;queue.emplace(value,source,next);
        }
      }
    }
  }
  for(const auto& f:frontiers) {
    FrontierLink c;c.frontier_id=f.id;c.active=f.active;c.centroid=f.centroid;c.geometry_signature=frontierFingerprint({f});
    if(sources.empty()){c.reason="NO_JUNCTION";links.push_back(std::move(c));continue;}
    if(!std::isfinite(f.centroid.x)||!std::isfinite(f.centroid.y)) {
      c.reason="FRONTIER_CENTER_BLOCKED";links.push_back(std::move(c));continue;
    }
    auto target_position=gridPosition(in.geometry,f.centroid);
    if(target_position.x<0||target_position.y<0||target_position.x>=in.geometry.width||target_position.y>=in.geometry.height) {
      c.reason="FRONTIER_CENTER_BLOCKED";links.push_back(std::move(c));continue;
    }
    auto target=gridCell(in.geometry,f.centroid);
    if(!free(target)&&state(target)!=UNKNOWN) {
      c.reason="FRONTIER_CENTER_BLOCKED";links.push_back(std::move(c));continue;
    }
    // Centroids of curved boundaries often lie in nonmember Unknown cells.
    // Pick a reachable FREE side of an actual boundary cell near that centroid;
    // the remaining short association is never part of the traversable path.
    std::set<std::pair<int,int>> approaches;
    if(free(target))approaches.emplace(target.x,target.y);
    else for(auto p:f.cells)if(state(p)==UNKNOWN)
      for(auto d:{cv::Point(1,0),cv::Point(-1,0),cv::Point(0,1),cv::Point(0,-1)}) {
        auto q=p+d;if(!free(q))continue;
        auto world=in.geometry.world(q);
        if(cv::norm(world-f.centroid)<=max_association_distance+1e-9&&
           frontierAssociationClear(in.geometry,world,f.centroid,state))approaches.emplace(q.x,q.y);
      }
    int best=-1;double best_gap=inf,best_cost=inf;
    for(auto [x,y]:approaches) {
      cv::Point q(x,y);int idx=index(q);if(owner[idx]<0)continue;
      auto approach=free(target)?f.centroid:in.geometry.world(q);
      if(!visible(in.geometry,in.geometry.world(q),approach,state,false))continue;
      double gap=cv::norm(approach-f.centroid);
      double cost=distance[idx]+cv::norm(in.geometry.world(q)-approach);
      if(gap<best_gap-1e-9||(std::abs(gap-best_gap)<=1e-9&&cost<best_cost)) {
        best=idx;best_gap=gap;best_cost=cost;
      }
    }
    if(best<0){c.reason=approaches.empty()?"NO_NEARBY_FREE_APPROACH":"UNREACHABLE";links.push_back(std::move(c));continue;}
    const auto* junction=sources[owner[best]].node;
    c.approach=free(target)?f.centroid:in.geometry.world(point(best));
    for(int p=best;;p=parent[p]){c.pixels.push_back(point(p));if(parent[p]==p)break;}
    std::reverse(c.pixels.begin(),c.pixels.end());for(auto p:c.pixels)c.path.push_back(in.geometry.world(p));
    auto route=c.path;
    if(cv::norm(route.front()-junction->position)>1e-9)route.insert(route.begin(),junction->position);
    if(cv::norm(route.back()-c.approach)>1e-9)route.push_back(c.approach);
    c.display_path=shorten(in.geometry,route,state,false);
    if(c.display_path.empty()){c.reason="UNREACHABLE";c.path.clear();c.pixels.clear();links.push_back(std::move(c));continue;}
    c.length=pathLength(c.display_path);
    // A grid's eight directions can slightly bias the owner. Prefer any
    // shorter verified direct connection to the same safe approach.
    for(const auto& source:sources) {
      double length=cv::norm(source.node->position-c.approach);
      if(length>=c.length-1e-9||!visible(in.geometry,source.node->position,c.approach,state,false))continue;
      junction=source.node;c.display_path={junction->position,c.approach};c.length=length;
    }
    // Keep the raster witness for the FINAL shortened route, not the discarded
    // grid detour. Every path cell, including the endpoint, is known FREE.
    c.pixels.clear();c.path.clear();
    if(c.display_path.size()==1)c.pixels.push_back(gridCell(in.geometry,c.display_path.front()));
    for(size_t i=1;i<c.display_path.size();++i) {
      std::vector<cv::Point> raster;
      visible(in.geometry,c.display_path[i-1],c.display_path[i],state,false,&raster);
      for(auto p:raster)if(c.pixels.empty()||c.pixels.back()!=p)c.pixels.push_back(p);
    }
    for(auto p:c.pixels)c.path.push_back(in.geometry.world(p));
    c.connected=true;c.reason="CONNECTED";c.junction_id=junction->id;
    links.push_back(std::move(c));
  }
  return links;
}
bool discardStaleLinks(std::vector<FrontierLink>& links,const std::vector<Frontier>& current) {
  std::map<int,uint64_t> signatures;for(const auto& f:current)signatures[f.id]=frontierFingerprint({f});
  size_t before=links.size();
  links.erase(std::remove_if(links.begin(),links.end(),[&](const FrontierLink& c){
    auto it=signatures.find(c.frontier_id);return it==signatures.end()||it->second!=c.geometry_signature;
  }),links.end());
  return before!=links.size();
}
}
