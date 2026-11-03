// Geometric contract tests: real raster inputs, no ROS/model mocks.
#include <plan_env/skeleton_graph.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <set>
using namespace apexnav_planner::skeleton;
void check(bool v, const std::string& message) { if (!v) throw std::runtime_error(message); }
GridInput blank(int w=81,int h=81) {
  GridInput g; g.geometry={w,h,.1,-4.,-4.,0.,"test_map"};
  g.cells=cv::Mat(h,w,CV_8UC1,cv::Scalar(BLOCKED)); return g;
}
void line(GridInput& g, cv::Point a, cv::Point b, int thickness=1) {
  cv::line(g.cells,a,b,cv::Scalar(FREE),thickness,cv::LINE_8);
}
int count(const Graph& g, NodeType type) { int n=0; for (auto& v:g.nodes) n+=v.type==type; return n; }
void validate(const GridInput& in, const Graph& g) {
  std::set<uint64_t> ids;
  for (auto& n:g.nodes) {
    check(ids.insert(n.id).second,"duplicate node ID");
    check(in.cells.at<uint8_t>(n.pixel)==FREE,"node outside traversable cells");
    check(n.active && n.clearance>=0,"node active/clearance");
    check(n.neighbors.size()==n.neighbor_count,"neighbor count");
  }
  for (auto& e:g.edges) {
    check(ids.count(e.start)&&ids.count(e.end),"missing edge endpoint");
    check(e.pixels.size()>=2 && e.path.size()==e.pixels.size(),"edge path compression lost pixels");
    double length=0;
    for (size_t i=0;i<e.pixels.size();++i) {
      check(in.cells.at<uint8_t>(e.pixels[i])==FREE,"edge entered unknown/blocked/inflated space");
      if (i) {
        auto d=e.pixels[i]-e.pixels[i-1];
        check(std::abs(d.x)<=1&&std::abs(d.y)<=1&&(d.x||d.y),"disconnected/duplicate path pixel");
        length+=std::hypot(d.x,d.y)*in.geometry.resolution;
      }
    }
    check(std::abs(e.length-length)<1e-9,"edge length must be full path length");
    check(e.min_clearance>=0,"edge clearance");
    for(auto& n:g.nodes) {
      if(n.id==e.start) check(n.pixel==e.pixels.front(),"path start must equal representative");
      if(n.id==e.end) check(n.pixel==e.pixels.back(),"path end must equal representative");
    }
  }
}
Graph extract(const GridInput& in, Options opts={}) { auto g=build(in,opts);validate(in,g);return g; }
int main() {
  Options opts; opts.min_branch_length=0;
  auto straight=blank();line(straight,{10,40},{70,40},7);
  auto before=straight.cells.clone();auto g=extract(straight,opts);
  check(count(g,NodeType::ENDPOINT)==2&&g.edges.size()==1&&g.nodes.size()==2,"straight: expected E2/J0/edge1");
  check(cv::countNonZero(before!=straight.cells)==0,"must not mutate input map");
  check(g.edges[0].min_clearance>.2,"thick corridor clearance");
  std::cout<<"PASS straight corridor: 2 endpoints, 1 edge\n";
  auto t=blank();line(t,{10,40},{70,40});line(t,{40,10},{40,40});
  auto tg=extract(t,opts);
  check(count(tg,NodeType::ENDPOINT)==3&&count(tg,NodeType::JUNCTION)==1&&tg.edges.size()==3,"T: diagonal triangles/false junctions");
  std::cout<<"PASS T corridor: 3 endpoints, 1 junction, 3 edges\n";
  auto cross=t;cross.cells=t.cells.clone();line(cross,{40,40},{40,70});
  auto cg=extract(cross,opts);
  check(count(cg,NodeType::ENDPOINT)==4&&count(cg,NodeType::JUNCTION)==1&&cg.edges.size()==4,"cross graph");
  std::cout<<"PASS cross: 4 endpoints, 1 junction, 4 edges\n";
  auto branches=blank();line(branches,{8,40},{72,40});line(branches,{25,15},{25,40});line(branches,{55,40},{55,65});
  auto bg=extract(branches,opts);
  check(bg.nodes.size()==6&&bg.edges.size()==5&&count(bg,NodeType::JUNCTION)==2,"branched indoor compression");
  check(bg.skeleton_pixels>bg.nodes.size()*5,"connectors must be compressed");
  std::cout<<"PASS branches: 4 endpoints, 2 junctions, 5 edges\n";
  auto noisy=blank();line(noisy,{10,40},{70,40});line(noisy,{40,37},{40,40});
  check(extract(noisy,opts).nodes.size()==4,"noise fixture");
  auto prune=opts;prune.min_branch_length=.4;
  auto ng=extract(noisy,prune);
  check(ng.nodes.size()==2&&ng.edges.size()==1,"prune short leaf by metric length");
  check(extract(t,prune).nodes.size()==4,"must keep long narrow branch");
  std::cout<<"PASS pruning: short leaf removed, long narrow branch preserved\n";
  auto unknown=noisy;unknown.cells=noisy.cells.clone();unknown.cells.at<uint8_t>(36,40)=UNKNOWN;
  auto ug=extract(unknown,prune);
  check(ug.nodes.size()==4,"protect unknown-facing temporary endpoint");
  bool near=false;for(auto& n:ug.nodes)if(n.pixel==cv::Point(40,37))near=n.near_unknown_boundary;
  check(near,"unknown endpoint not marked");
  std::cout<<"PASS unknown boundary flag and pruning protection\n";
  auto ring=blank();cv::rectangle(ring.cells,{15,15},{65,65},cv::Scalar(FREE),1);
  auto rg=extract(ring,prune);
  check(rg.nodes.size()==1&&count(rg,NodeType::LOOP_ANCHOR)==1&&rg.edges.size()==1,"closed ring needs loop anchor");
  check(rg.edges[0].start==rg.edges[0].end&&std::abs(rg.edges[0].length-20.)<1e-9,"closed ring path/length");
  std::cout<<"PASS closed ring: 1 loop anchor, 1 closed edge, 20m\n";
  auto corner=blank();line(corner,{10,10},{30,10});line(corner,{30,10},{30,30});
  auto kg=extract(corner,opts);check(kg.nodes.size()==2&&kg.edges.size()==1,"L corner created spurious branch");
  check(std::abs(kg.edges[0].length-4.)<1e-9,"L corner cut diagonal");
  auto diag=blank();line(diag,{10,10},{30,30});
  check(extract(diag,opts).nodes.size()==2,"real diagonal disconnected");
  auto noise=blank();noise.cells.at<uint8_t>(40,40)=FREE;
  check(extract(noise,opts).nodes.empty(),"isolated pixel not removed");
  auto border=blank();line(border,{0,0},{25,0});
  check(extract(border,opts).nodes.size()==2,"image border corridor lost");
  std::cout<<"PASS diagonal handling, isolated noise, map border\n";
  auto metadata=t;metadata.geometry={81,81,.2,10.,-7.,std::acos(-1.)/2,"rotated_map"};
  auto mg=extract(metadata,opts);
  for(auto& n:mg.nodes) {
    check(std::abs(n.position.x-(10.-(n.pixel.y+.5)*.2))<1e-9,"origin/yaw X");
    check(std::abs(n.position.y-(-7.+(n.pixel.x+.5)*.2))<1e-9,"origin/yaw Y");
  }
  auto same=build(t,opts,tg);validate(t,same);
  check(same.nodes.size()==tg.nodes.size(),"repeat node count");
  for(size_t i=0;i<tg.nodes.size();++i)check(tg.nodes[i].id==same.nodes[i].id,"repeat build IDs changed");
  auto grown=t;grown.cells=t.cells.clone();line(grown,{70,40},{72,40});
  auto gg=build(grown,opts,tg);validate(grown,gg);
  std::set<uint64_t> old,newids;for(auto& n:tg.nodes)old.insert(n.id);for(auto& n:gg.nodes)newids.insert(n.id);
  check(old==newids,"nearby same-type node should retain ID");
  std::cout<<"PASS origin/resolution/frame/yaw and stable IDs\n";
  ChangeGate gate(16,2,0.); auto f=fingerprint(t,16);
  check(gate.shouldRebuild(f),"initial build");gate.accept(f);
  check(!gate.shouldRebuild(f),"unchanged map rebuild");
  auto changes=t;changes.cells=t.cells.clone();changes.cells.at<uint8_t>(2,2)=UNKNOWN;
  check(!gate.shouldRebuild(fingerprint(changes,16)),"one tile below threshold");
  changes.cells.at<uint8_t>(70,70)=UNKNOWN;
  check(gate.shouldRebuild(fingerprint(changes,16)),"small changes must accumulate since accepted build");
  std::cout<<"PASS map change gate accumulation\nALL SKELETON TESTS PASSED\n";
  auto thick=blank();line(thick,{10,40},{70,40},7);line(thick,{40,10},{40,70},7);
  auto thickg=extract(thick,prune);
  check(count(thickg,NodeType::ENDPOINT)==4&&count(thickg,NodeType::JUNCTION)==1&&thickg.edges.size()==4,"thick cross junction clustering");
  auto parallel=ring;parallel.cells=ring.cells.clone();line(parallel,{15,40},{65,40});
  auto pg=extract(parallel,opts);
  check(count(pg,NodeType::JUNCTION)==2&&pg.edges.size()==3,"parallel paths between two junctions must survive");
  auto inflated=straight;inflated.cells=straight.cells.clone();
  cv::rectangle(inflated.cells,{39,0},{41,80},cv::Scalar(INFLATED),-1);
  auto ig=extract(inflated,opts);
  check(ig.edges.size()==2&&count(ig,NodeType::ENDPOINT)==4,"inflated barrier should split corridor");
  auto empty=blank();auto eg=extract(empty,opts);check(eg.nodes.empty()&&eg.edges.empty(),"all-blocked input");
  empty.cells.setTo(UNKNOWN);check(extract(empty,opts).nodes.empty(),"all-unknown input");
  auto other_frame=t;other_frame.geometry.frame_id="other_map";
  auto fg=build(other_frame,opts,tg);
  for(auto& n:fg.nodes)check(!old.count(n.id),"different coordinate frames cannot reuse IDs");
  std::cout<<"PASS thick junction, parallel edges, inflated barrier, empty maps and frame change\n";
  auto diamond=blank(11,11);
  line(diamond,{5,1},{5,4});line(diamond,{1,5},{4,5});
  line(diamond,{6,5},{9,5});line(diamond,{5,6},{5,9});
  auto dg=extract(diamond,opts);
  check(dg.nodes.size()==5&&dg.edges.size()==5,"junction clustering must preserve internal diamond cycle");
  int self_edges=0;for(auto& e:dg.edges)self_edges+=e.start==e.end;
  check(self_edges==1,"junction internal cycle needs a closed edge");
  std::cout<<"PASS loop inside merged junction cluster\n";
  auto x=blank(23,23);
  line(x,{3,3},{10,10});line(x,{11,10},{18,3});
  line(x,{10,11},{3,18});line(x,{11,11},{18,18});
  auto xg=extract(x,opts);
  std::cout<<"X residue graph: nodes="<<xg.nodes.size()<<" edges="<<xg.edges.size()<<"\n";
  check(xg.nodes.size()==5&&xg.edges.size()==4,"filled 2x2 junction residue must not create a false loop");
  std::cout<<"PASS filled 2x2 junction cleanup preserves four branches without false loop\n";
  auto joined_faces=blank(11,11);
  for(cv::Point p:std::vector<cv::Point>{{5,1},{2,2},{4,2},{6,2},{3,3},{4,3},{5,3},
      {3,4},{4,4},{5,4},{3,5},{5,5},{4,6},{6,6},{2,7},{3,7},{4,7},{6,7},{5,8},{7,8},{6,9}})
    joined_faces.cells.at<uint8_t>(p)=FREE;
  auto jf=extract(joined_faces,opts);
  check(int(jf.edges.size())-int(jf.nodes.size())+1==4,
        "adjacent filled faces must not create an extra cycle; preserve four actual holes");
  std::cout<<"PASS adjacent filled faces: four true holes, no extra cycle\n";
  auto filtered=opts;filtered.remove_deadends=true;
  auto filtered_t=extract(t,filtered);
  check(filtered_t.nodes.size()==1&&filtered_t.nodes[0].type==NodeType::JUNCTION&&filtered_t.edges.empty(),"remove all deadends, retaining isolated Junction anchor");
  check(filtered_t.removed_deadends==3,"filtered deadend count");
  check(extract(straight,filtered).nodes.empty(),"pure corridor has no invented junction");
  check(extract(ring,filtered).edges.size()==1,"loop anchor with one unique neighbor is not a deadend");
  check(extract(diamond,filtered).edges.size()==1,"real junction loop survives deadend filter");
  std::cout<<"PASS deadend removal preserves junction anchors and real loops\n";
 auto nearby=blank();line(nearby,{5,40},{75,40});
 line(nearby,{30,10},{30,40});line(nearby,{42,40},{42,70});
 auto merging=opts;merging.junction_merge_distance=2.;
 auto merged=build(nearby,merging);
 check(count(merged,NodeType::JUNCTION)==1&&merged.edges.size()==4,"nearby Junctions must merge and internal edge disappear");
 auto j=*std::find_if(merged.nodes.begin(),merged.nodes.end(),[](const Node& n){return n.type==NodeType::JUNCTION;});
 check(cv::norm(j.position-cv::Point2d(-.35,.05))<1e-9,"merged Junction must be exact midpoint");
 check(j.anchor_pixels.size()==2&&j.neighbor_count==4,"merged Junction lost geometry anchors or external neighbors");
 for(const auto& e:merged.edges) {
   check(e.display_path.size()>=2,"edge display path missing");
   if(e.start==j.id)check(e.display_path.front()==j.position,"rewired edge must start at merged centre");
   if(e.end==j.id)check(e.display_path.back()==j.position,"rewired edge must end at merged centre");
 }
 auto repeat=build(nearby,merging,merged);
 auto repeated=*std::find_if(repeat.nodes.begin(),repeat.nodes.end(),[](const Node& n){return n.type==NodeType::JUNCTION;});
 check(repeated.id==j.id,"merged Junction ID unstable on unchanged map");
 check(count(build(branches,merging),NodeType::JUNCTION)==2,"Junctions beyond 2m merged");
 auto boundary=blank();line(boundary,{5,40},{75,40});line(boundary,{30,10},{30,40});line(boundary,{50,40},{50,70});
 check(count(build(boundary,merging),NodeType::JUNCTION)==2,"exactly 2m must not merge");
 auto three=nearby;three.cells=nearby.cells.clone();line(three,{47,40},{47,20});
 auto triple=build(three,merging);
 check(count(triple,NodeType::JUNCTION)==1,"nearby three-way grouping incomplete");
 for(const auto& n:triple.nodes)if(n.type==NodeType::JUNCTION)
   check(std::abs(n.position.x-(-4.+(30.+42.+47.)/3.*.1+.05))<1e-9&&n.anchor_pixels.size()==3,
         "multi-node centre must average original members, not biased successive midpoints");
 std::cout<<"PASS Junction midpoint/centroid merging, distance threshold, rewiring and stable IDs\n";
 auto loops_merge=merging;loops_merge.junction_merge_distance=6.;
 auto merged_loops=build(parallel,loops_merge);
 check(merged_loops.nodes.size()==1&&merged_loops.edges.size()==2,
       "contracting nearby Junctions must retain independent real cycles");
 // Two disconnected T junctions separated by a blocked centre: exact midpoint
 // is a logical location, while both original free geometry anchors survive.
 auto separated=blank();line(separated,{10,40},{28,40});line(separated,{20,20},{20,40});
 line(separated,{35,40},{55,40});line(separated,{40,40},{40,65});
 loops_merge.junction_merge_distance=2.1;
 auto separated_graph=build(separated,loops_merge);
 for(const auto& n:separated_graph.nodes)if(n.type==NodeType::JUNCTION) {
   check(cv::norm(n.position-cv::Point2d(-.95,.05))<1e-9,"abstract midpoint unexpectedly snapped to a free anchor");
   check(n.anchor_pixels.size()==2,"logical merge lost original safe anchor locations");
   check(separated.cells.at<uint8_t>(n.pixel)==BLOCKED&&n.clearance==0,"midpoint occupancy/clearance misrepresented");
 }
 std::cout<<"PASS merged centres retain real loops and original geometry anchors\n";
}
