// Real SDFMap2D -> read-only manager -> ROS graph/markers, without a controller.
#include <plan_env/skeleton_graph_manager.h>
#include <plan_env/sdf_map2d.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace apexnav_planner;
using namespace std::chrono_literals;
void require(bool v,const char* s){if(!v)throw std::runtime_error(s);}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("skeleton_manager_check");
  node->declare_parameter("sdf_map.resolution",.05);
  node->declare_parameter("sdf_map.map_size_x",6.);
  node->declare_parameter("sdf_map.map_size_y",6.);
  node->declare_parameter("sdf_map.obstacles_inflation",.1);
  node->declare_parameter("sdf_map.max_ray_length",2.5);
  node->declare_parameter("skeleton_graph.update_hz",10.);
  node->declare_parameter("skeleton_graph.remove_deadends",false);
  auto map=std::make_unique<SDFMap2D>();map->initMap(node,false);
  auto cloud=pcl::make_shared<pcl::PointCloud<pcl::PointXY>>();
  for(int i=-100;i<=100;++i){pcl::PointXY p;p.x=i*.02f;p.y=.8f;cloud->push_back(p);p.y=-.8f;cloud->push_back(p);}
  for(int i=-40;i<=40;++i){pcl::PointXY p;p.x=2.f;p.y=i*.02f;cloud->push_back(p);p.x=-2.f;cloud->push_back(p);}
  for(int i=0;i<3;++i){std::vector<Eigen::Vector2i> grids;map->inputDepthCloud2D(cloud,Eigen::Vector3d(0,0,.8),grids);}
  auto states=[&]{std::vector<int> out;for(int x=0;x<120;++x)for(int y=0;y<120;++y){Eigen::Vector2i p(x,y);out.push_back(map->getOccupancy(p));out.push_back(map->getInflateOccupancy(p));}return out;};
  auto before=states();
  std::vector<plan_env::msg::SkeletonGraph> graphs;
  visualization_msgs::msg::MarkerArray markers;
  auto sub=node->create_subscription<plan_env::msg::SkeletonGraph>("/skeleton_graph/graph",rclcpp::QoS(1).transient_local(),
    [&](plan_env::msg::SkeletonGraph::SharedPtr m){graphs.push_back(*m);});
  auto msub=node->create_subscription<visualization_msgs::msg::MarkerArray>("/skeleton_graph/markers",rclcpp::QoS(1).transient_local(),
    [&](visualization_msgs::msg::MarkerArray::SharedPtr m){markers=*m;});
  auto spin=[&](double seconds){auto end=std::chrono::steady_clock::now()+std::chrono::duration<double>(seconds);while(std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);std::this_thread::sleep_for(5ms);}};
  auto manager=std::make_unique<SkeletonGraphManager>(node,map.get(),"custom_map");
  spin(1.);
  require(!graphs.empty()&&!graphs.back().nodes.empty()&&!graphs.back().edges.empty(),"real map produced no graph");
  const auto graph=graphs.back();
  require(graph.header.frame_id=="custom_map"&&graph.map_info.origin.position.x==-3.,"map metadata/frame not preserved");
  for(const auto& n:graph.nodes){require(std::abs(n.position.x-(-3.+(n.pixel_x+.5)*.05))<1e-10,"node x conversion");require(std::abs(n.position.y-(-3.+(n.pixel_y+.5)*.05))<1e-10,"node y conversion");}
  require(before==states(),"skeleton extraction changed the map");
  size_t line_points=0;bool clear=false;
  for(const auto& m:markers.markers){if(m.action==m.DELETEALL)clear=true;if(m.ns=="edges"){require(m.color.b==1&&m.color.r==0,"edge marker color");line_points=m.points.size();}}
  size_t expected=0;for(const auto& e:graph.edges)expected+=2*(e.display_path.size()-1);
  require(clear&&line_points==expected,"RViz edges must use complete polylines");
  spin(.4);require(graphs.back().revision==graph.revision,"unchanged map rebuilt");
  std::cout<<"PASS read-only SDFMap2D, metadata, full-path markers, unchanged-map throttle: nodes="<<graph.nodes.size()<<" edges="<<graph.edges.size()<<"\n";
  // An obstacle inside the known rectangle creates branching routes around it.
  // Camera/map updates alone must leave their Junctions as candidates.
  map->setForceOccGrid(Eigen::Vector2d(0,0));spin(.6);
  auto branch=std::find_if(graphs.back().nodes.begin(),graphs.back().nodes.end(),
    [](const auto& n){return n.node_type==n.JUNCTION;});
  require(branch!=graphs.back().nodes.end(),"visit fixture needs a Junction");
  const auto target=*branch;
  require(target.visit_state==target.CANDIDATE,"camera-only construction confirmed Junction");
  auto odom_pub=node->create_publisher<nav_msgs::msg::Odometry>("/odom_world",10);spin(.15);
  auto visited=[&] {
    for(const auto& n:graphs.back().nodes)if(n.node_id==target.node_id)return n.visit_state==n.CONFIRMED;
    return false;
  };
  nav_msgs::msg::Odometry odom;odom.pose.pose.position=target.position;odom.pose.pose.orientation.w=1.;
  odom.header.frame_id="wrong_frame";odom.header.stamp=node->now();odom_pub->publish(odom);spin(.1);
  require(!visited(),"wrong-frame odometry confirmed Junction");
  odom.header.frame_id="custom_map";odom.header.stamp=node->now()-rclcpp::Duration::from_seconds(2.);
  odom_pub->publish(odom);spin(.1);require(!visited(),"stale odometry confirmed Junction");
  odom.header.stamp=node->now();odom_pub->publish(odom);spin(.15);
  require(visited(),"robot visit did not publish CONFIRMED state");
  bool dark=false;for(const auto& m:markers.markers)
    if(m.ns=="junctions_confirmed")dark=!m.points.empty()&&std::abs(m.color.g-.4f)<1e-6;
  require(dark,"confirmed Junction must use dark green marker");
  odom.pose.pose.position.x+=2.;odom.header.stamp=node->now();odom_pub->publish(odom);spin(.1);
  require(visited(),"leaving Junction erased confirmation");
  std::cout<<"PASS candidate creation, wrong/stale pose rejection, actual robot visit and dark-green marker\n";
  // Destroy an observer while it may have a job, then reconstruct map/manager.
  map->setForceOccGrid(Eigen::Vector2d(0,0));spin(.11);manager.reset();
  map=std::make_unique<SDFMap2D>();map->initMap(node,false);
  manager=std::make_unique<SkeletonGraphManager>(node,map.get(),"custom_map");spin(.6);
  require(graphs.back().nodes.empty()&&graphs.back().edges.empty(),"old graph resurrected after reset");
  require(graphs.back().revision<=1,"new map generation not reset");
  require(node->count_publishers("/cmd_vel")==0,"skeleton manager must not control motion");
  manager.reset();
  std::cout<<"PASS reset during work, empty graph replaces old markers, no velocity publisher\n";
  rclcpp::shutdown();
}
