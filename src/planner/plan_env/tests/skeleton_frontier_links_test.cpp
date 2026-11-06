#include <plan_env/skeleton_frontier_links.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
using namespace apexnav_planner::skeleton;
void check(bool v,const char* msg){if(!v)throw std::runtime_error(msg);}
GridInput map(){GridInput in;in.geometry={14,12,.1,-1.,2.,0.,"world"};in.cells=cv::Mat(12,14,CV_8U,cv::Scalar(BLOCKED));return in;}
Node junction(const Geometry& g,uint64_t id,cv::Point p){Node n;n.id=id;n.type=NodeType::JUNCTION;n.pixel=p;n.position=g.world(p);return n;}
Frontier frontier(const Geometry& g,int id,cv::Point p){return {id,true,g.world(p),{p}};}
void verify(const GridInput& in,const FrontierLink& c){
 check(c.connected&&!c.pixels.empty(),"connection missing");double length=0;
 for(size_t i=0;i<c.pixels.size();++i){auto p=c.pixels[i];check(in.cells.at<uint8_t>(p)==FREE,"unsafe path pixel");if(i){auto d=p-c.pixels[i-1];check(abs(d.x)<=1&&abs(d.y)<=1&&(d.x||d.y),"path not connected");if(d.x&&d.y){check(in.cells.at<uint8_t>(p.y-d.y,p.x)==FREE&&in.cells.at<uint8_t>(p.y,p.x-d.x)==FREE,"corner cut");}length+=cv::norm(d)*in.geometry.resolution;}}
 double displayed=0;for(size_t i=1;i<c.display_path.size();++i)displayed+=cv::norm(c.display_path[i]-c.display_path[i-1]);
 check(std::abs(displayed-c.length)<1e-9,"cost must describe current-centre shortened route");
 auto state=[&](cv::Point p)->uint8_t {return p.x>=0&&p.y>=0&&p.x<in.cells.cols&&p.y<in.cells.rows?in.cells.at<uint8_t>(p):BLOCKED;};
 check(frontierPathTraversable(in.geometry,c.display_path,state,false),"safe route crossed obstacle/Unknown/corner");
}
int main(){
 // A bent frontier's arithmetic centroid need not be a frontier member.
 auto bent=map();cv::rectangle(bent.cells,{1,1},{7,10},cv::Scalar(FREE),-1);
 cv::rectangle(bent.cells,{8,1},{12,10},cv::Scalar(UNKNOWN),-1);
 Graph bg;bg.geometry=bent.geometry;bg.nodes={junction(bent.geometry,51,{3,5})};
 Frontier bf;bf.id=502;bf.cells={{8,4},{9,5},{8,6}};
 for(auto p:bf.cells)bf.centroid+=bent.geometry.world(p);
 bf.centroid*=1./bf.cells.size();
 auto bl=connectFrontiers(bent,bg,{bf})[0];
 check(bl.connected,"bent frontier must connect through its reachable known-free side despite unknown nonmember centroid");
 check(bl.centroid==bf.centroid&&bl.frontier_id==502,"association must preserve Frontier identity and sphere centre");
 check(bl.display_path.back()==bl.approach&&bl.approach!=bl.centroid,"safe route must stop before the unknown association segment");
 verify(bent,bl);
 auto limited=connectFrontiers(bent,bg,{bf},.1)[0];
 check(!limited.connected&&limited.reason=="NO_NEARBY_FREE_APPROACH","association radius must prevent a distant centroid hook");
 auto bent_state=[&](cv::Point p)->uint8_t{return bent.cells.at<uint8_t>(p);};
 check(frontierAssociationClear(bent.geometry,bl.approach,bl.centroid,bent_state),"short association should allow Unknown");
 bent.cells.at<uint8_t>(5,8)=BLOCKED;
 check(!connectFrontiers(bent,bg,{bf})[0].connected,"centroid inside a real obstacle must remain disconnected");
 check(!frontierAssociationClear(bent.geometry,bent.geometry.world({7,5}),bent.geometry.world({10,5}),bent_state),
       "drawing association must not cross a real wall even if its endpoint is Unknown");
 std::cout<<"PASS unknown nonmember centroid uses separate safe approach\n";
 auto in=map();cv::rectangle(in.cells,{1,1},{12,10},cv::Scalar(FREE),-1);
 for(int y=1;y<9;++y)in.cells.at<uint8_t>(y,5)=BLOCKED;
 in.cells.at<uint8_t>(2,6)=UNKNOWN;
 Graph graph;graph.geometry=in.geometry;graph.nodes={junction(in.geometry,17,{4,2}),junction(in.geometry,31,{10,5})};
 auto f=frontier(in.geometry,123,{6,2});
 auto links=connectFrontiers(in,graph,{f});check(links.size()==1&&links[0].junction_id==31,"must select nearest reachable path, not Euclidean through wall");verify(in,links[0]);
 check(links[0].frontier_id==123&&links[0].centroid==f.centroid,"Frontier identity/centroid lost");
 check(abs(links[0].pixels.back().x-6)+abs(links[0].pixels.back().y-2)==1,"approach must touch original Frontier");
 graph.nodes.pop_back();links=connectFrontiers(in,graph,{f});verify(in,links[0]);
 check(links[0].length>1.,"wall detour shortcut");
 check(connectFrontiers(in,graph,{}).empty(),"vanished frontier retained");
 std::cout<<"PASS closest traversable junction, wall detour, safe approach and removal\n";
 auto other=f;other.active=false;auto fp=frontierFingerprint({f});check(fp!=frontierFingerprint({other}),"active/dormant change not detected");
  auto retained=links;check(!discardStaleLinks(retained,{f}),"fresh connection discarded");
  check(discardStaleLinks(retained,{} )&&retained.empty(),"late async result restored a vanished Frontier");
  retained=links;check(discardStaleLinks(retained,{other}),"changed active/dormant result accepted");
 other=f;other.cells.push_back({6,3});check(fp!=frontierFingerprint({other}),"membership change not detected");
 check(frontierFingerprint({f,other})==frontierFingerprint({other,f}),"input order should not affect signature");
 graph.nodes.clear();links=connectFrontiers(in,graph,{f});check(!links[0].connected&&links[0].reason=="NO_JUNCTION","missing junction not reported");
 std::cout<<"PASS Frontier-only updates and explicit no-junction status\n";
 auto diagonal=map();diagonal.cells.at<uint8_t>(2,2)=FREE;diagonal.cells.at<uint8_t>(3,3)=FREE;diagonal.cells.at<uint8_t>(3,4)=UNKNOWN;
 Graph dg;dg.geometry=diagonal.geometry;dg.nodes={junction(diagonal.geometry,1,{2,2})};
 auto df=frontier(diagonal.geometry,8,{4,3});links=connectFrontiers(diagonal,dg,{df});check(!links[0].connected&&links[0].reason=="UNREACHABLE","diagonal through blocked corner");
 diagonal.cells.at<uint8_t>(2,3)=FREE;diagonal.cells.at<uint8_t>(3,2)=FREE;
 links=connectFrontiers(diagonal,dg,{df});verify(diagonal,links[0]);
 diagonal.cells.at<uint8_t>(3,3)=INFLATED;
 links=connectFrontiers(diagonal,dg,{df});check(!links[0].connected,"inflated boundary approach accepted");
 std::cout<<"PASS no diagonal corner cutting, Unknown and inflated exclusion\n";
 // Opening one door must refresh links even below the geometry rebuild gate.
 auto door=map();cv::line(door.cells,{4,1},{4,9},cv::Scalar(FREE));
 cv::line(door.cells,{4,5},{11,5},cv::Scalar(FREE));
 door.cells.at<uint8_t>(5,8)=BLOCKED;door.cells.at<uint8_t>(5,12)=UNKNOWN;
 Options options;options.remove_deadends=true;options.min_branch_length=0;options.min_deadend_width=0;
 auto door_graph=build(door,options);auto door_f=frontier(door.geometry,90,{12,5});
 auto blocked=connectFrontiers(door,door_graph,{door_f});
 check(blocked.size()==1&&blocked[0].reason=="UNREACHABLE","closed door fixture");
 ChangeGate geometry_gate(4,2,0.),link_gate(4,1,0.);
 auto baseline=fingerprint(door,4);geometry_gate.accept(baseline);link_gate.accept(baseline);
 door.cells.at<uint8_t>(5,8)=FREE;auto opened=fingerprint(door,4);
 check(!geometry_gate.shouldRebuild(opened)&&link_gate.shouldRebuild(opened),"small opening must refresh links independently of geometry");
 auto connected=connectFrontiers(door,door_graph,{door_f});verify(door,connected[0]);
 link_gate.accept(opened);door.cells.at<uint8_t>(1,1)=UNKNOWN;
 check(geometry_gate.shouldRebuild(fingerprint(door,4)),"link refresh must not lose accumulated geometry changes");
 std::cout<<"PASS single-tile door opening reconnects, geometry baseline preserved\n";
 check(connected[0].display_path.size()>=2&&connected[0].display_path.back()==connected[0].approach,
       "solid connection must stop at the safe approach");
 check(connected[0].path.back()!=door_f.centroid,"Unknown centroid must not be labelled safe geometry");
 // Invalid/off-map centroids must not be connected by an unchecked long hook.
 auto extended=door_f;extended.centroid={1.37,2.83};
 auto extended_link=connectFrontiers(door,door_graph,{extended})[0];
 check(!extended_link.connected&&extended_link.reason=="FRONTIER_CENTER_BLOCKED","invalid centroid needs explicit unreachable state");
 std::cout<<"PASS safe path excludes Frontier centre, invalid centre rejected\n";
 auto open=map();cv::rectangle(open.cells,{1,1},{12,10},cv::Scalar(FREE),-1);
 Graph current_graph;current_graph.geometry=open.geometry;
 auto centre=junction(open.geometry,70,{5,5});
 current_graph.nodes={centre};open.cells.at<uint8_t>(5,12)=UNKNOWN;
 auto centre_f=frontier(open.geometry,101,{12,5});
 auto straight=connectFrontiers(open,current_graph,{centre_f})[0];
 check(straight.display_path.size()==2,"open connection must be straight, without old-anchor or approach hooks");
 check(straight.pixels.front()==cv::Point(5,5),"route must start from current Junction, not obsolete anchor");
 check(std::abs(straight.length-.6)<1e-9,"cost must exclude the 0.1m unknown association");
 check(straight.approach==open.geometry.world({11,5}),"straight route must end at the known-free side");
 current_graph.nodes={junction(open.geometry,71,{11,5})};
 auto zero=connectFrontiers(open,current_graph,{centre_f})[0];verify(open,zero);
 check(zero.display_path.size()==1&&zero.length==0,"junction already at approach must keep zero-length safe attachment");
 current_graph.nodes={centre};
 open.cells.at<uint8_t>(5,5)=BLOCKED;
 auto no_centre=connectFrontiers(open,current_graph,{centre_f})[0];
 check(!no_centre.connected,"blocked Junction must not be used as a route source");
 std::cout<<"PASS current-centre direct route, actual distance, blocked-centre rejection\n";
}
