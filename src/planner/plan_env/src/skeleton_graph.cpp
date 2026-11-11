#include <plan_env/skeleton_graph.h>
#include <opencv2/imgproc.hpp>
#ifdef APEXNAV_HAVE_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace apexnav_planner::skeleton {
cv::Point2d Geometry::world(cv::Point p) const {
  double x=(p.x+.5)*resolution,y=(p.y+.5)*resolution;
  return {origin_x+std::cos(yaw)*x-std::sin(yaw)*y,origin_y+std::sin(yaw)*x+std::cos(yaw)*y};
}
bool Geometry::operator==(const Geometry& g) const {
  return width==g.width&&height==g.height&&resolution==g.resolution&&origin_x==g.origin_x&&
         origin_y==g.origin_y&&yaw==g.yaw&&frame_id==g.frame_id;
}
namespace {
void validate(const GridInput& in) {
  const auto& g=in.geometry;
  if(g.width<=0||g.height<=0||!std::isfinite(g.resolution)||g.resolution<=0||
     !std::isfinite(g.origin_x)||!std::isfinite(g.origin_y)||!std::isfinite(g.yaw)||
     g.frame_id.empty()||in.cells.type()!=CV_8UC1||in.cells.cols!=g.width||in.cells.rows!=g.height)
    throw std::invalid_argument("invalid skeleton map geometry/image");
}
cv::Mat thin(const cv::Mat& binary) {
  cv::Mat s;
#ifdef APEXNAV_HAVE_XIMGPROC
  cv::ximgproc::thinning(binary,s,cv::ximgproc::THINNING_ZHANGSUEN);
#else
  binary.convertTo(s,CV_8U,1./255.);
  bool changed;
  do {
    changed=false;
    for(int phase=0;phase<2;++phase) {
      std::vector<cv::Point> remove;
      for(int y=1;y<s.rows-1;++y)for(int x=1;x<s.cols-1;++x) {
        if(!s.at<uint8_t>(y,x))continue;
        int p[]={s.at<uint8_t>(y-1,x),s.at<uint8_t>(y-1,x+1),s.at<uint8_t>(y,x+1),
          s.at<uint8_t>(y+1,x+1),s.at<uint8_t>(y+1,x),s.at<uint8_t>(y+1,x-1),
          s.at<uint8_t>(y,x-1),s.at<uint8_t>(y-1,x-1)};
        int sum=0,transitions=0;
        for(int i=0;i<8;++i){sum+=p[i];transitions+=!p[i]&&p[(i+1)%8];}
        bool removable=phase==0 ? !(p[0]&&p[2]&&p[4]) && !(p[2]&&p[4]&&p[6])
                                : !(p[0]&&p[2]&&p[6]) && !(p[0]&&p[4]&&p[6]);
        if(sum>=2&&sum<=6&&transitions==1&&removable)remove.push_back({x,y});
      }
      for(auto p:remove)s.at<uint8_t>(p)=0;
      changed|=!remove.empty();
    }
  }while(changed);
  s*=255;
#endif
  return s;
}
using Adjacency=std::map<int,std::vector<int>>;
cv::Point pixel(int index,int width){return {index%width,index/width};}
uint64_t link(int a,int b){if(a>b)std::swap(a,b);return (uint64_t(uint32_t(a))<<32)|uint32_t(b);}
// Sparse GF(2) path basis. A filled 2x2 raster face is contractible; its
// perimeter must not become a graph cycle. Seed all such faces, then retain
// only independent path vectors. This also removes unions of adjacent filled
// faces, regardless of which spanning tree is chosen inside a junction.
class PathBasis {
public:
  bool insert(const std::vector<int>& path) {
    std::set<uint64_t> edges;
    for(size_t i=1;i<path.size();++i) {
      auto e=link(path[i-1],path[i]);if(!edges.erase(e))edges.insert(e);
    }
    while(!edges.empty()) {
      auto pivot=*edges.rbegin();auto found=basis_.find(pivot);
      if(found==basis_.end()){basis_.emplace(pivot,std::move(edges));return true;}
      for(auto e:found->second)if(!edges.erase(e))edges.insert(e);
    }
    return false;
  }
private:
  std::map<uint64_t,std::set<uint64_t>> basis_;
};
Adjacency adjacency(const cv::Mat& s) {
  Adjacency out;
  for(int y=1;y<s.rows-1;++y)for(int x=1;x<s.cols-1;++x) {
    if(!s.at<uint8_t>(y,x))continue;
    std::vector<int> neighbors;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      if(!(dx||dy)||!s.at<uint8_t>(y+dy,x+dx))continue;
      // Keep genuine diagonal paths, but suppress shortcuts around an existing
      // orthogonal corner; otherwise every T/L creates little triangle cycles.
      if(dx&&dy&&(s.at<uint8_t>(y,x+dx)||s.at<uint8_t>(y+dy,x)))continue;
      neighbors.push_back((y+dy)*s.cols+x+dx);
    }
    if(!neighbors.empty())out.emplace(y*s.cols+x,std::move(neighbors));
  }
  return out;
}
struct Region {NodeType type;std::vector<int> members;int representative;};
struct Traced {Graph graph;Adjacency adj;std::map<int,int> owner;};
std::vector<int> insidePath(int a,int b,int group,const Adjacency& adj,const std::map<int,int>& owner) {
  if(a==b)return {a};
  std::queue<int> todo;todo.push(a);std::unordered_map<int,int> parent;parent[a]=a;
  while(!todo.empty()&&!parent.count(b)) {
    int p=todo.front();todo.pop();
    for(int n:adj.at(p)) {
      auto it=owner.find(n);
      if(it!=owner.end()&&it->second==group&&!parent.count(n)){parent[n]=p;todo.push(n);}
    }
  }
  if(!parent.count(b))throw std::logic_error("disconnected junction cluster");
  std::vector<int> path{b};while(path.back()!=a)path.push_back(parent.at(path.back()));
  std::reverse(path.begin(),path.end());return path;
}
Traced trace(const cv::Mat& s,const cv::Mat& clearance,const cv::Mat& unknown_distance,
             const Geometry& geometry,cv::Point offset,const Options& options) {
  Traced r;r.graph.geometry=geometry;r.adj=adjacency(s);
  r.graph.skeleton_pixels=r.adj.size();
  std::vector<Region> regions;
  auto add=[&](NodeType type,std::vector<int> members) {
    double cx=0,cy=0;for(int p:members){auto q=pixel(p,s.cols);cx+=q.x;cy+=q.y;}
    cx/=members.size();cy/=members.size();
    int rep=members.front();double best=std::numeric_limits<double>::infinity();
    for(int p:members){auto q=pixel(p,s.cols);double d=std::hypot(q.x-cx,q.y-cy);if(d<best){best=d;rep=p;}}
    int group=regions.size();for(int p:members)r.owner[p]=group;
    regions.push_back({type,std::move(members),rep});
  };
  for(const auto& [p,ns]:r.adj) {
    if(ns.size()==1)add(NodeType::ENDPOINT,{p});
    else if(ns.size()>=3&&!r.owner.count(p)) {
      std::vector<int> members{p};std::set<int> seen{p};
      for(size_t i=0;i<members.size();++i)for(int n:r.adj.at(members[i]))
        if(r.adj.at(n).size()>=3&&seen.insert(n).second)members.push_back(n);
      add(NodeType::JUNCTION,std::move(members));
    }
  }
  // Components with no formal nodes are pure cycles, not empty graphs.
  std::set<int> seen;
  for(const auto& [p,ns]:r.adj)if(seen.insert(p).second) {
    std::vector<int> component{p};bool has_node=r.owner.count(p);
    for(size_t i=0;i<component.size();++i)for(int n:r.adj.at(component[i]))if(seen.insert(n).second) {
      component.push_back(n);has_node|=r.owner.count(n)>0;
    }
    if(!has_node)add(NodeType::LOOP_ANCHOR,{*std::min_element(component.begin(),component.end())});
  }
  for(size_t i=0;i<regions.size();++i) {
    auto p=pixel(regions[i].representative,s.cols);Node n;
    n.id=i+1;n.type=regions[i].type;n.pixel=p+offset;n.position=geometry.world(n.pixel);
    n.clearance=clearance.at<float>(p);
    n.near_unknown_boundary=unknown_distance.at<float>(p)*geometry.resolution<=options.unknown_boundary_distance;
    r.graph.nodes.push_back(n);
  }
  PathBasis paths;
  for(int y=1;y<s.rows-2;++y)for(int x=1;x<s.cols-2;++x)
    if(s.at<uint8_t>(y,x)&&s.at<uint8_t>(y,x+1)&&
       s.at<uint8_t>(y+1,x)&&s.at<uint8_t>(y+1,x+1)) {
      int p=y*s.cols+x;paths.insert({p,p+1,p+1+s.cols,p+s.cols,p});
    }
  auto appendEdge=[&](const std::vector<int>& points,size_t start,size_t end) {
    if(!paths.insert(points))return;
    Edge e;e.start=start+1;e.end=end+1;e.min_clearance=std::numeric_limits<double>::infinity();
    for(int idx:points) {
      auto local=pixel(idx,s.cols),p=local+offset;
      if(!e.pixels.empty())e.length+=cv::norm(p-e.pixels.back())*geometry.resolution;
      e.pixels.push_back(p);e.path.push_back(geometry.world(p));
      e.min_clearance=std::min(e.min_clearance,double(clearance.at<float>(local)));
    }
    e.display_path=e.path;
    r.graph.edges.push_back(std::move(e));
  };
  // Contract a junction cluster's spanning tree, but preserve every independent
  // internal cycle. Otherwise a small obstacle surrounded by branch pixels
  // would disappear from the graph when those pixels merge into one junction.
  for(size_t i=0;i<regions.size();++i)if(regions[i].members.size()>1) {
    int root=regions[i].representative;
    std::map<int,int> parent{{root,root}};
    std::vector<int> pending{root};std::set<uint64_t> tree;
    for(size_t j=0;j<pending.size();++j)for(int n:r.adj.at(pending[j])) {
      auto owner=r.owner.find(n);
      if(owner!=r.owner.end()&&owner->second==int(i)&&!parent.count(n)) {
        parent[n]=pending[j];tree.insert(link(n,pending[j]));pending.push_back(n);
      }
    }
    auto toRoot=[&](int p) {std::vector<int> path{p};while(path.back()!=root)path.push_back(parent.at(path.back()));return path;};
    for(int p:regions[i].members)for(int n:r.adj.at(p)) {
      auto owner=r.owner.find(n);
      if(p>=n||owner==r.owner.end()||owner->second!=int(i)||tree.count(link(p,n)))continue;
      auto path=toRoot(p);std::reverse(path.begin(),path.end());
      auto suffix=toRoot(n);path.insert(path.end(),suffix.begin(),suffix.end());
      appendEdge(path,i,i);
    }
  }
  std::unordered_set<uint64_t> visited;
  for(size_t i=0;i<regions.size();++i)for(int member:regions[i].members)for(int next:r.adj.at(member)) {
    if((r.owner.count(next)&&r.owner.at(next)==int(i))||visited.count(link(member,next)))continue;
    std::vector<int> corridor{member};int prev=member,cur=next;
    visited.insert(link(prev,cur));corridor.push_back(cur);
    while(!r.owner.count(cur)) {
      const auto& ns=r.adj.at(cur);
      if(ns.size()!=2)throw std::logic_error("unclassified skeleton pixel");
      int to=ns[0]==prev?ns[1]:ns[0];
      if(!visited.insert(link(cur,to)).second)throw std::logic_error("unanchored skeleton cycle");
      prev=cur;cur=to;corridor.push_back(cur);
    }
    int target=r.owner.at(cur);
    auto points=insidePath(regions[i].representative,member,i,r.adj,r.owner);
    points.insert(points.end(),corridor.begin()+1,corridor.end());
    auto suffix=insidePath(cur,regions[target].representative,target,r.adj,r.owner);
    points.insert(points.end(),suffix.begin()+1,suffix.end());
    appendEdge(points,i,target);
  }
  return r;
}
// Contract connected groups using ORIGINAL edge lengths, then average each
// original Junction once. No Euclidean-radius or cluster-size rule.
void mergeShortJunctions(Graph& g,double threshold,const cv::Mat& clearance,
                         const cv::Mat& unknown_distance,cv::Point offset,double unknown_limit) {
  if(threshold<=0)return;
  std::map<uint64_t,uint64_t> parent;
  for(const auto& n:g.nodes)if(n.type==NodeType::JUNCTION)parent[n.id]=n.id;
  auto root=[&](uint64_t id){while(parent.at(id)!=id)id=parent.at(id);return id;};
  bool changed=false;
  for(const auto& e:g.edges)if(e.start!=e.end&&e.length<threshold&&parent.count(e.start)&&parent.count(e.end)) {
    auto a=root(e.start),b=root(e.end);if(a!=b){parent[std::max(a,b)]=std::min(a,b);changed=true;}
  }
  if(!changed)return;
  std::map<uint64_t,std::vector<Node>> groups;
  for(const auto& n:g.nodes)groups[parent.count(n.id)?root(n.id):n.id].push_back(n);
  const auto& geo=g.geometry;
  auto cell=[&](cv::Point2d p) {
    p-=cv::Point2d(geo.origin_x,geo.origin_y);
    return cv::Point(int(std::floor((std::cos(geo.yaw)*p.x+std::sin(geo.yaw)*p.y)/geo.resolution)),
                     int(std::floor((-std::sin(geo.yaw)*p.x+std::cos(geo.yaw)*p.y)/geo.resolution)));
  };
  auto sample=[&](const cv::Mat& m,cv::Point p) {
    p-=offset;return p.x>=0&&p.y>=0&&p.x<m.cols&&p.y<m.rows?double(m.at<float>(p)):0.;
  };
  std::map<uint64_t,Node> merged;
  for(const auto& [id,members]:groups) {
    Node n=members.front();n.id=id;
    if(members.size()>1) {
      n.position={0,0};for(const auto& member:members)n.position+=member.position;
      n.position*=1./members.size();n.pixel=cell(n.position);
      n.clearance=sample(clearance,n.pixel);
      n.near_unknown_boundary=sample(unknown_distance,n.pixel)*geo.resolution<=unknown_limit;
    }
    merged[id]=std::move(n);
  }
  std::vector<Edge> edges;
  for(auto e:g.edges) {
    auto a=parent.count(e.start)?root(e.start):e.start,b=parent.count(e.end)?root(e.end):e.end;
    // Remove contracted short links, retaining pre-existing loops and longer
    // alternative routes even if their endpoints now belong to the same group.
    if(e.start!=e.end&&a==b&&e.length<threshold)continue;
    auto start=merged.at(a).position,end=merged.at(b).position;
    e.start=a;e.end=b;
    if(cv::norm(start-e.path.front())>1e-9||cv::norm(end-e.path.back())>1e-9) {
      auto original=std::move(e.path);e.path.clear();e.pixels.clear();e.length=0;
      auto append=[&](cv::Point2d p) {
        if(!e.path.empty()) {double d=cv::norm(p-e.path.back());if(d<1e-9)return;e.length+=d;}
        e.path.push_back(p);e.pixels.push_back(cell(p));
        e.min_clearance=std::min(e.min_clearance,sample(clearance,cell(p)));
      };
      auto connector=[&](cv::Point2d from,cv::Point2d to) {
        int steps=std::max(1,int(std::ceil(cv::norm(to-from)/(geo.resolution*.5))));
        for(int i=1;i<=steps;++i)append(from+(double(i)/steps)*(to-from));
      };
      append(start);connector(start,original.front());
      for(auto p:original)append(p);
      connector(original.back(),end);
      e.display_path=e.path;
    }
    edges.push_back(std::move(e));
  }
  g.nodes.clear();for(auto& [id,n]:merged)g.nodes.push_back(std::move(n));
  g.edges=std::move(edges);
}
void match(Graph& g,const Graph& old,double distance) {
  bool same_frame=g.geometry.frame_id==old.geometry.frame_id;
  g.next_node_id=old.next_node_id;g.next_edge_id=old.next_edge_id;
  std::vector<std::tuple<double,size_t,size_t>> candidates;
  if(same_frame)for(size_t i=0;i<g.nodes.size();++i)for(size_t j=0;j<old.nodes.size();++j)
    if(g.nodes[i].type==old.nodes[j].type) {
      double d=cv::norm(g.nodes[i].position-old.nodes[j].position);
      if(d<=distance)candidates.emplace_back(d,i,j);
    }
  std::sort(candidates.begin(),candidates.end());
  std::vector<bool> used_new(g.nodes.size()),used_old(old.nodes.size());
  std::map<uint64_t,uint64_t> remap;
  for(auto [d,i,j]:candidates)if(!used_new[i]&&!used_old[j]) {
    remap[g.nodes[i].id]=old.nodes[j].id;used_new[i]=used_old[j]=true;
  }
  for(size_t i=0;i<g.nodes.size();++i) {
    auto temp=g.nodes[i].id;
    g.nodes[i].id=used_new[i]?remap.at(temp):g.next_node_id++;
    remap[temp]=g.nodes[i].id;
  }
  std::map<uint64_t,std::set<uint64_t>> neighbors;std::set<uint64_t> used_edges;
  for(auto& e:g.edges) {
    e.start=remap.at(e.start);e.end=remap.at(e.end);
    double best=std::numeric_limits<double>::infinity();const Edge* found=nullptr;
    if(same_frame)for(const auto& p:old.edges) {
      if(used_edges.count(p.id)||!((p.start==e.start&&p.end==e.end)||(p.start==e.end&&p.end==e.start)))continue;
      double delta=std::abs(e.length-p.length)+cv::norm(e.path[e.path.size()/2]-p.path[p.path.size()/2]);
      if(delta<best){best=delta;found=&p;}
    }
    e.id=found?found->id:g.next_edge_id++;used_edges.insert(e.id);
    neighbors[e.start].insert(e.end);neighbors[e.end].insert(e.start);
  }
  for(auto& n:g.nodes){n.neighbors.assign(neighbors[n.id].begin(),neighbors[n.id].end());n.neighbor_count=n.neighbors.size();}
}
}
const char* thinningBackend() {
#ifdef APEXNAV_HAVE_XIMGPROC
  return "opencv_ximgproc_zhang_suen";
#else
  return "builtin_zhang_suen";
#endif
}
bool confirmVisitedJunctions(Graph& graph,cv::Point2d robot,double radius,int64_t stamp_ns) {
  if(!std::isfinite(robot.x)||!std::isfinite(robot.y)||stamp_ns<=0)return false;
  bool changed=false;
  for(auto& n:graph.nodes)if(n.active&&n.type==NodeType::JUNCTION&&n.visit_state==VisitState::CANDIDATE&&
                            cv::norm(n.position-robot)<=radius) {
    n.visit_state=VisitState::CONFIRMED;n.confirmed_at_ns=stamp_ns;n.visit_position=robot;changed=true;
  }
  return changed;
}
void inheritJunctionVisits(Graph& graph,const Graph& previous,double radius) {
  std::map<uint64_t,const Node*> old;
  for(const auto& n:previous.nodes)if(n.type==NodeType::JUNCTION)old[n.id]=&n;
  for(auto& n:graph.nodes)if(n.type==NodeType::JUNCTION) {
    n.visit_state=VisitState::CANDIDATE;n.confirmed_at_ns=0;n.visit_position={};
    auto found=old.find(n.id);if(found==old.end())continue;
    const auto& p=*found->second;
    if(p.visit_state==VisitState::CONFIRMED&&p.confirmed_at_ns>0&&cv::norm(n.position-p.visit_position)<=radius) {
      n.visit_state=p.visit_state;n.confirmed_at_ns=p.confirmed_at_ns;n.visit_position=p.visit_position;
    }
  }
}
Graph build(const GridInput& in,const Options& options,const Graph& previous) {
  if(!std::isfinite(options.visit_radius)||options.visit_radius<=0)throw std::invalid_argument("invalid Junction visit radius");
  validate(in);
  for(double v:{options.min_island_area,options.min_branch_length,options.min_deadend_width,options.unknown_boundary_distance,options.node_match_distance,options.junction_merge_length})
    if(!std::isfinite(v)||v<0)throw std::invalid_argument("invalid skeleton distance parameter");
  if(!std::isfinite(options.deadend_width_sample_fraction)||options.deadend_width_sample_fraction<=0||options.deadend_width_sample_fraction>1)
    throw std::invalid_argument("invalid deadend width sample fraction");
  cv::Mat all_free=(in.cells==FREE);
  if(options.min_island_area>0) {
    cv::Mat labels,stats,centres;
    // Four-connectivity rejects corner-only contact. Filter a temporary mask;
    // occupancy, inflation and Frontier data remain untouched.
    const int count=cv::connectedComponentsWithStats(all_free,labels,stats,centres,4,CV_32S);
    int largest=0;
    for(int i=1;i<count;++i)
      if(!largest||stats.at<int>(i,cv::CC_STAT_AREA)>stats.at<int>(largest,cv::CC_STAT_AREA))largest=i;
    std::vector<uint8_t> keep(count,0);
    for(int i=1;i<count;++i) {
      const double area=stats.at<int>(i,cv::CC_STAT_AREA)*in.geometry.resolution*in.geometry.resolution;
      keep[i]=(i==largest||area>=options.min_island_area)?255:0;
    }
    for(int y=0;y<all_free.rows;++y)for(int x=0;x<all_free.cols;++x)
      all_free.at<uint8_t>(y,x)=keep[labels.at<int>(y,x)];
  }
  std::vector<cv::Point> nonzero;cv::findNonZero(all_free,nonzero);
  if(nonzero.empty()){Graph empty;empty.geometry=in.geometry;empty.next_node_id=previous.next_node_id;empty.next_edge_id=previous.next_edge_id;return empty;}
  cv::Rect box=cv::boundingRect(nonzero);
  // Include known/unknown context around the crop for endpoint classification.
  int margin=std::max(1,int(std::ceil(options.unknown_boundary_distance/in.geometry.resolution))+1);
  box.x=std::max(0,box.x-margin);box.y=std::max(0,box.y-margin);
  int xmax=0,ymax=0;for(auto p:nonzero){xmax=std::max(xmax,p.x);ymax=std::max(ymax,p.y);}
  box.width=std::min(in.cells.cols,xmax+margin+1)-box.x;
  box.height=std::min(in.cells.rows,ymax+margin+1)-box.y;
  cv::Mat binary,known;
  cv::copyMakeBorder(all_free(box),binary,1,1,1,1,cv::BORDER_CONSTANT,0);
  cv::Mat local_known=(in.cells(box)!=UNKNOWN);
  cv::copyMakeBorder(local_known,known,1,1,1,1,cv::BORDER_CONSTANT,0);
  cv::Mat clearance,unknown_distance;
  cv::distanceTransform(binary,clearance,cv::DIST_L2,cv::DIST_MASK_PRECISE);
  // Subtract the blocked cell's half diagonal: conservative boundary clearance,
  // expressed in metres after inflation, not an optimistic centre-to-centre EDT.
  clearance=(clearance-std::sqrt(.5))*in.geometry.resolution;
  cv::max(clearance,0,clearance);
  cv::distanceTransform(known,unknown_distance,cv::DIST_L2,cv::DIST_MASK_PRECISE);
  cv::Mat s=thin(binary);cv::Point offset=box.tl()-cv::Point(1,1);
  uint32_t pruned=0;
  while(true) {
    auto result=trace(s,clearance,unknown_distance,in.geometry,offset,options);
    std::set<int> remove;
    for(const auto& e:result.graph.edges) {
      const auto& a=result.graph.nodes.at(e.start-1);const auto& b=result.graph.nodes.at(e.end-1);
      const Node* leaf=nullptr;int junction=-1;
      if(a.type==NodeType::ENDPOINT&&b.type==NodeType::JUNCTION){leaf=&a;junction=e.end-1;}
      if(b.type==NodeType::ENDPOINT&&a.type==NodeType::JUNCTION){leaf=&b;junction=e.start-1;}
      const bool isolated=a.type==NodeType::ENDPOINT&&b.type==NodeType::ENDPOINT;
      if(!leaf&&!isolated)continue;
      // Sample the terminal part by arc length, not pixel count. The maximum
      // avoids treating a wall-adjacent endpoint's tiny clearance as the width
      // of a whole dead end; excluding the mouth avoids wide junction bias.
      double max_width=0,travelled=0;bool sampled=false;
      const bool forward=a.type==NodeType::ENDPOINT;
      cv::Point previous_pixel;
      for(size_t i=0;i<e.pixels.size();++i) {
        const auto p=e.pixels[forward?i:e.pixels.size()-1-i]-offset;
        if(i)travelled+=cv::norm(p-previous_pixel)*in.geometry.resolution;
        if(!isolated&&travelled>e.length*options.deadend_width_sample_fraction)break;
        previous_pixel=p;
        const int idx=p.y*s.cols+p.x;
        if(result.owner.count(idx)&&result.owner.at(idx)==junction)continue;
        sampled=true;max_width=std::max(max_width,2.*clearance.at<float>(p));
      }
      const bool short_branch=e.length<options.min_branch_length;
      const bool narrow_branch=sampled&&options.min_deadend_width>0&&max_width<options.min_deadend_width;
      if(!short_branch&&!narrow_branch)continue;
      ++pruned;
      for(auto p:e.pixels){p-=offset;int idx=p.y*s.cols+p.x;
        if(!result.owner.count(idx)||result.owner.at(idx)!=junction)remove.insert(idx);}
    }
    if(remove.empty()) {
      auto g=std::move(result.graph);g.pruned_branches=pruned;
      if(options.remove_deadends) {
        // One pass over formal deadends. Do not recursively peel Junctions:
        // they remain the anchors for dynamic Frontier connections.
        std::map<uint64_t,std::set<uint64_t>> neighbors;
        for(const auto& e:g.edges){neighbors[e.start].insert(e.end);neighbors[e.end].insert(e.start);}
        std::set<uint64_t> removed;
        for(const auto& n:g.nodes)if(n.type==NodeType::ENDPOINT&&neighbors[n.id].size()==1)removed.insert(n.id);
        g.nodes.erase(std::remove_if(g.nodes.begin(),g.nodes.end(),[&](const Node& n){return removed.count(n.id);}),g.nodes.end());
        g.edges.erase(std::remove_if(g.edges.begin(),g.edges.end(),[&](const Edge& e){return removed.count(e.start)||removed.count(e.end);}),g.edges.end());
        g.removed_deadends=removed.size();
      }
      mergeShortJunctions(g,options.junction_merge_length,clearance,unknown_distance,offset,options.unknown_boundary_distance);
      match(g,previous,options.node_match_distance);
      inheritJunctionVisits(g,previous,options.visit_radius);return g;
    }
    for(int idx:remove)s.at<uint8_t>(pixel(idx,s.cols))=0;
  }
}
Fingerprint fingerprint(const GridInput& in,int tile_size) {
  validate(in);if(tile_size<=0)throw std::invalid_argument("invalid fingerprint tile size");
  Fingerprint f;f.geometry=in.geometry;
  for(int y=0;y<in.cells.rows;y+=tile_size)for(int x=0;x<in.cells.cols;x+=tile_size) {
    uint64_t h=14695981039346656037ULL;
    for(int j=y;j<std::min(y+tile_size,in.cells.rows);++j)for(int i=x;i<std::min(x+tile_size,in.cells.cols);++i)
      h=(h^in.cells.at<uint8_t>(j,i))*1099511628211ULL;
    f.tiles.push_back(h);
  }return f;
}
ChangeGate::ChangeGate(int tile_size,int min_changed_tiles,double min_changed_ratio):min_tiles_(min_changed_tiles),min_ratio_(min_changed_ratio) {
  if(tile_size<=0||min_tiles_<1||!std::isfinite(min_ratio_)||min_ratio_<0||min_ratio_>1)
    throw std::invalid_argument("invalid skeleton change threshold");
}
size_t ChangeGate::changedTiles(const Fingerprint& current) const {
  if(!(current.geometry==baseline_.geometry)||current.tiles.size()!=baseline_.tiles.size())return current.tiles.size();
  size_t changed=0;for(size_t i=0;i<current.tiles.size();++i)changed+=current.tiles[i]!=baseline_.tiles[i];return changed;
}
bool ChangeGate::shouldRebuild(const Fingerprint& current) const {
  if(!(current.geometry==baseline_.geometry)||baseline_.tiles.empty())return true;
  size_t n=changedTiles(current);
  return n>0&&(n>=size_t(min_tiles_)||(min_ratio_>0&&double(n)/current.tiles.size()>=min_ratio_));
}
}
