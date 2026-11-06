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
  Options opts; opts.min_branch_length=0;opts.min_deadend_width=0;
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
  check(ug.nodes.size()==2&&ug.edges.size()==1,"short leaf near Unknown must also be pruned");
  auto unknown_unpruned=extract(unknown,opts);
  bool near=false;for(auto& n:unknown_unpruned.nodes)if(n.pixel==cv::Point(40,37))near=n.near_unknown_boundary;
  check(near,"unknown endpoint not marked");
  std::cout<<"PASS unknown boundary metadata without pruning exemption\n";
  // A long furniture slit exceeds the length threshold but is narrow along
  // its terminal half; the wide main corridor and wide dead ends must survive.
  auto slit=blank();line(slit,{10,40},{70,40},7);line(slit,{40,10},{40,40});
  auto width_prune=prune;width_prune.min_deadend_width=.2;
  check(extract(slit,prune).nodes.size()==4,"long slit fixture should have a junction");
  auto slit_before=slit.cells.clone();auto slit_graph=extract(slit,width_prune);
  check(slit_graph.nodes.size()==2&&slit_graph.edges.size()==1&&slit_graph.pruned_branches==1,
        "narrow long deadend must be deleted and obsolete junction recompressed");
  check(cv::countNonZero(slit_before!=slit.cells)==0,"width pruning changed occupancy");
  for(const auto& edge:slit_graph.edges)for(auto p:edge.pixels)
    check(p.y>=36,"pruned slit pixels still in graph");
  auto wide=blank();line(wide,{10,40},{70,40},7);line(wide,{40,10},{40,40},7);
  check(extract(wide,width_prune).nodes.size()==4,"wide terminal branch must survive");
  auto lower=blank();line(lower,{10,40},{70,40},7);line(lower,{40,40},{40,70});
  check(extract(lower,width_prune).nodes.size()==2,"width sampling must start at either endpoint orientation");
  auto unknown_slit=slit;unknown_slit.cells=slit.cells.clone();unknown_slit.cells.at<uint8_t>(9,40)=UNKNOWN;
  check(extract(unknown_slit,width_prune).nodes.size()==2,"Unknown must not exempt narrow long leaf");
  // A short tapered cap alone must not erase a broad terminal region.
  auto tapered=wide;tapered.cells=wide.cells.clone();line(tapered,{40,6},{40,10});
  check(extract(tapered,width_prune).nodes.size()==4,"narrow tip alone must not delete a broad deadend");
  check(extract(straight,width_prune).nodes.size()==2,"standalone corridor must not be treated as junction spur");
  auto bad=width_prune;bad.min_deadend_width=-1;bool rejected=false;
  try{build(slit,bad);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"negative width threshold accepted");
  bad=width_prune;bad.deadend_width_sample_fraction=0;rejected=false;
  try{build(slit,bad);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"invalid width sampling fraction accepted");
  std::cout<<"PASS narrow leaf removal, wide/tapered leaves, reversed edges, Unknown and parameter validation\n";
  auto island=blank();line(island,{40,40},{42,40});
  check(extract(island,opts).nodes.size()==2,"isolated short segment fixture");
  check(extract(island,prune).nodes.empty(),"short Endpoint-Endpoint island must be pruned");
  auto narrow_island=blank();line(narrow_island,{20,40},{60,40});
  check(extract(narrow_island,width_prune).nodes.empty(),"long narrow isolated segment must be pruned");
  check(extract(straight,width_prune).nodes.size()==2,"long wide isolated corridor must survive");
  check(extract(slit,width_prune).nodes.size()==2,"branch pruning must not consume the wide remaining corridor");
  std::cout<<"PASS isolated short/narrow components removed, wide corridor retained\n";
  // These tests disable length/width pruning, isolating actual-frontier distance.
  auto facing=t;facing.cells=t.cells.clone();facing.cells.at<uint8_t>(9,40)=UNKNOWN;
  check(extract(facing,opts).nodes.size()==4,"generic Unknown proximity must not prune without a real frontier");
  facing.frontier_cells={{40,9}};
  auto facing_before=facing.cells.clone();auto facing_graph=extract(facing,opts);
  check(facing_graph.nodes.size()==2&&facing_graph.edges.size()==1&&facing_graph.pruned_branches==1,
        "reachable actual frontier must prune terminal branch even if long");
  check(cv::countNonZero(facing_before!=facing.cells)==0,"frontier pruning modified occupancy");
  auto disabled=opts;disabled.frontier_deadend_distance=0;
  check(extract(facing,disabled).nodes.size()==4,"frontier pruning switch ignored");
  auto exact=opts;exact.frontier_deadend_distance=.05;
  check(extract(facing,exact).nodes.size()==4,"distance equal to threshold must not prune");
  exact.frontier_deadend_distance=.0501;
  check(extract(facing,exact).nodes.size()==2,"half-cell distance to shared frontier face not used");
  // A nearby frontier across a wall requires a detour longer than 0.2m.
  auto wall=t;wall.cells=t.cells.clone();wall.geometry.resolution=.05;
  wall.cells.at<uint8_t>(10,42)=UNKNOWN;wall.frontier_cells={{42,10}};
  line(wall,{42,11},{42,20});line(wall,{40,20},{42,20});
  auto wall_graph=extract(wall,opts);
  bool kept=false;for(const auto& n:wall_graph.nodes)kept|=n.pixel==cv::Point(40,10);
  check(kept,"nearby frontier across wall must use long free-space detour");
  auto corner_block=t;corner_block.cells=t.cells.clone();corner_block.geometry.resolution=.05;
  corner_block.cells.at<uint8_t>(9,39)=UNKNOWN;corner_block.frontier_cells={{39,9}};
  corner_block.cells.at<uint8_t>(9,38)=FREE; // Actual frontier has a free neighbour on its far side.
  check(extract(corner_block,opts).nodes.size()==4,"diagonal Unknown proximity must not cut blocked corners");
  auto stale=facing;stale.cells=facing.cells.clone();stale.cells.at<uint8_t>(9,40)=BLOCKED;
  check(extract(stale,opts).nodes.size()==4,"obsolete frontier cell now occupied must be ignored");
  auto no_frontiers=facing;no_frontiers.frontier_cells.clear();
  auto recovered=build(no_frontiers,opts,facing_graph);validate(no_frontiers,recovered);
  check(recovered.nodes.size()==4,"frontier removal must allow geometry reconstruction from unchanged occupancy");
  std::cout<<"PASS actual frontier boundary pruning, strict threshold, walls, corner blocking, stale cells and recovery\n";
  auto islands=blank();line(islands,{5,15},{70,15},7);line(islands,{30,55},{50,55});
  auto island_filter=opts;island_filter.min_island_area=1.;
  auto input_before=islands.cells.clone();auto cleaned=extract(islands,island_filter);
  check(cleaned.nodes.size()==2&&cleaned.edges.size()==1,"small disconnected free region must not generate skeleton");
  for(const auto& n:cleaned.nodes)check(n.pixel.y<30,"small island retained");
  check(cv::countNonZero(input_before!=islands.cells)==0,"island filter changed original occupancy");
  check(extract(islands,opts).nodes.size()==4,"disabled area filter must retain both components");
  auto rooms=islands;rooms.cells=islands.cells.clone();line(rooms,{30,55},{70,55},7);
  check(extract(rooms,island_filter).nodes.size()==4,"large disconnected room must survive area filter");
  auto tiny=blank();line(tiny,{10,10},{20,10});
  check(extract(tiny,island_filter).nodes.size()==2,"largest component must survive even below area threshold");
  auto opened=islands;opened.cells=islands.cells.clone();line(opened,{40,15},{40,55});
  bool restored=false;for(const auto& e:extract(opened,island_filter).edges)for(auto p:e.pixels)restored|=p.y>=55;
  check(restored,"new connection must restore previously isolated region");
  auto inflated_island=opened;inflated_island.cells=opened.cells.clone();
  cv::rectangle(inflated_island.cells,{0,34},{80,36},cv::Scalar(INFLATED),-1);
  for(const auto& n:extract(inflated_island,island_filter).nodes)check(n.pixel.y<34,"inflation-separated small island must be removed");
  auto touching=blank();line(touching,{10,10},{30,10});line(touching,{31,11},{35,11});
  for(const auto& n:extract(touching,island_filter).nodes)check(n.pixel.x<=30,"diagonal corner touching must not join islands");
  auto bad_area=opts;bad_area.min_island_area=-1;bool area_rejected=false;
  try{build(islands,bad_area);}catch(const std::invalid_argument&){area_rejected=true;}
  check(area_rejected,"negative island area accepted");
  std::cout<<"PASS geometry-only small islands, large rooms, largest component, opening/inflation and corner separation\n";
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
 // Two distinct branch pixels only 0.4m apart must not move to their midpoint.
 auto nearby=blank();line(nearby,{5,40},{75,40});
 line(nearby,{30,10},{30,40});line(nearby,{34,40},{34,70});
 auto separate=extract(nearby,opts);
 check(count(separate,NodeType::JUNCTION)==2&&separate.edges.size()==5,
       "nearby real junctions and their connecting edge must remain distinct");
 std::set<int> branch_x;
 for(const auto& n:separate.nodes)if(n.type==NodeType::JUNCTION) {
   check(n.pixel.y==40,"Junction moved away from actual branch pixel");
   branch_x.insert(n.pixel.x);
 }
 check(branch_x==std::set<int>({30,34}),"Junction moved to an arithmetic midpoint");
 for(const auto& e:separate.edges)check(e.display_path==e.path,"display must follow actual skeleton, without centre hooks");
 auto repeat=build(nearby,opts,separate);validate(nearby,repeat);
 for(size_t i=0;i<separate.nodes.size();++i)
   check(separate.nodes[i].id==repeat.nodes[i].id,"unmerged node IDs changed on identical map");
 auto after_filter=extract(nearby,filtered);
 check(count(after_filter,NodeType::JUNCTION)==2&&after_filter.edges.size()==1,
       "deadend filtering must not spatially merge remaining junctions");
 std::cout<<"PASS distinct nearby junctions: actual branch positions, full paths, stable IDs\n";
 // Separate free-space components must not become connected by averaging nodes.
 auto separated=blank();line(separated,{10,40},{28,40});line(separated,{20,20},{20,40});
 line(separated,{35,40},{55,40});line(separated,{40,40},{40,65});
 auto disconnected=extract(separated,opts);
 check(count(disconnected,NodeType::JUNCTION)==2&&disconnected.nodes.size()==8&&disconnected.edges.size()==6,
       "disconnected T junctions must remain separate, without a blocked midpoint");
 std::cout<<"PASS disconnected junctions retain their real geometry\n";
}
