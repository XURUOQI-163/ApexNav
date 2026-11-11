#include <plan_env/skeleton_graph_manager.h>
#include <plan_env/sdf_map2d.h>
#include <plan_env/frontier_map2d.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace apexnav_planner {
namespace {
template<class T>T param(const rclcpp::Node::SharedPtr& n,const std::string& key,T fallback) {
  std::string name="skeleton_graph."+key;
  if(!n->has_parameter(name))n->declare_parameter<T>(name,fallback);
  return n->get_parameter(name).get_value<T>();
}
geometry_msgs::msg::Point point(cv::Point2d p,double z=0) {
  geometry_msgs::msg::Point out;out.x=p.x;out.y=p.y;out.z=z;return out;
}
using Clock=std::chrono::steady_clock;
double milliseconds(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
}
SkeletonGraphManager::SkeletonGraphManager(const rclcpp::Node::SharedPtr& node,SDFMap2D* map,const std::string& frame)
 :node_(node),map_(map),frame_(frame) {
  const double hz=param(node,"update_hz",1.);
  tile_size_=param(node,"change_tile_size",32);
  gate_=std::make_unique<skeleton::ChangeGate>(tile_size_,param(node,"min_changed_tiles",1),param(node,"min_changed_ratio",0.));
  // Connectivity must react to even a single opened/closed door, independently
  // of the more expensive geometry rebuild threshold.
  link_gate_=std::make_unique<skeleton::ChangeGate>(tile_size_,1,0.);
  options_.min_branch_length=param(node,"min_branch_length",.3);
  options_.unknown_boundary_distance=param(node,"unknown_boundary_distance",.5);
  options_.node_match_distance=param(node,"node_match_distance",.4);
  options_.min_deadend_width=param(node,"min_deadend_width",.2);
  options_.deadend_width_sample_fraction=param(node,"deadend_width_sample_fraction",.5);
  options_.min_island_area=param(node,"min_island_area",1.);
  options_.junction_merge_length=param(node,"junction_merge_length",.5);
  options_.visit_radius=param(node,"visit_radius",.5);
  odom_timeout_=param(node,"visit_odom_timeout",1.);
  options_.remove_deadends=param(node,"remove_deadends",true);
  connect_frontiers_=param(node,"connect_frontiers",true);
  max_association_distance_=param(node,"frontier_max_association_distance",.5);
  association_dash_length_=param(node,"frontier_association_dash_length",.04);
  z_=param(node,"visualization_z",.12);node_scale_=param(node,"node_scale",.16);
  frontier_z_=param(node,"frontier_visualization_z",.36); // Original MapROS Frontier centre height.
  edge_width_=param(node,"edge_width",.035);label_scale_=param(node,"label_scale",.14);
  labels_=param(node,"show_node_ids",true);
  if(!std::isfinite(hz)||hz<=0||hz>30)throw std::invalid_argument("skeleton_graph.update_hz must be in (0,30]");
  if(!std::isfinite(options_.visit_radius)||options_.visit_radius<=0||!std::isfinite(odom_timeout_)||odom_timeout_<=0)
    throw std::invalid_argument("Junction visit radius and odometry timeout must be positive");
  for(double v:{options_.min_island_area,options_.min_branch_length,options_.min_deadend_width,options_.unknown_boundary_distance,options_.node_match_distance,options_.junction_merge_length})
    if(!std::isfinite(v)||v<0)throw std::invalid_argument("skeleton graph distances must be finite and nonnegative");
  if(!std::isfinite(options_.deadend_width_sample_fraction)||options_.deadend_width_sample_fraction<=0||options_.deadend_width_sample_fraction>1)
    throw std::invalid_argument("skeleton_graph.deadend_width_sample_fraction must be in (0,1]");
  for(double v:{node_scale_,edge_width_,label_scale_,max_association_distance_,association_dash_length_})
    if(!std::isfinite(v)||v<=0)throw std::invalid_argument("skeleton graph marker scales must be positive");
  if(!std::isfinite(z_)||!std::isfinite(frontier_z_))throw std::invalid_argument("invalid skeleton marker height");
  graph_pub_=node->create_publisher<plan_env::msg::SkeletonGraph>("/skeleton_graph/graph",rclcpp::QoS(1).transient_local());
  markers_pub_=node->create_publisher<visualization_msgs::msg::MarkerArray>("/skeleton_graph/markers",rclcpp::QoS(1).transient_local());
  Eigen::Vector2d origin,size;map_->getRegion(origin,size);double resolution=map_->getResolution();
  graph_.geometry={int(std::ceil(size.x()/resolution)),int(std::ceil(size.y()/resolution)),resolution,origin.x(),origin.y(),0,frame_};
  sampled_at_=node->now();publish(node); // New map episode clears prior latched graph/markers.
  created_at_ns_=node->now().nanoseconds();
  odom_sub_=node->create_subscription<nav_msgs::msg::Odometry>("/odom_world",10,
    [this](nav_msgs::msg::Odometry::SharedPtr msg){odometryCallback(msg);});
  timer_=node->create_wall_timer(std::chrono::duration<double>(1./hz),[this]{tick();});
  RCLCPP_INFO(node->get_logger(),"[Skeleton] enabled: %.2f Hz, %s, read-only SDFMap2D observer",hz,skeleton::thinningBackend());
  RCLCPP_INFO(node->get_logger(),"[Skeleton] leaf pruning: length < %.2fm OR terminal %.0f%% max width < %.2fm; no Unknown exemption",
    options_.min_branch_length,100*options_.deadend_width_sample_fraction,options_.min_deadend_width);
  RCLCPP_INFO(node->get_logger(),"[Skeleton] isolated segments use full-path width; no frontier-proximity pruning");
  RCLCPP_INFO(node->get_logger(),"[Skeleton] Junction clustering: direct edge length < %.2fm, arithmetic mean position (0 disables)",options_.junction_merge_length);
  RCLCPP_INFO(node->get_logger(),"[Skeleton] Junction confirmation requires robot odometry within %.2fm",options_.visit_radius);
  RCLCPP_INFO(node->get_logger(),"[Skeleton] discard isolated free regions smaller than %.2fm^2; retain largest component",options_.min_island_area);
}
SkeletonGraphManager::~SkeletonGraphManager() {
  if(timer_)timer_->cancel();
  // No worker captures this/map/node; join before releasing the last input.
  if(job_.valid())job_.wait();
}
void SkeletonGraphManager::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto node=node_.lock();if(!node||msg->header.frame_id!=frame_)return;
  int64_t stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
  double age=(node->now().nanoseconds()-stamp)*1e-9;
  const auto& p=msg->pose.pose.position;
  if(stamp<created_at_ns_||stamp<=robot_stamp_ns_||age<0||age>odom_timeout_||!std::isfinite(p.x)||!std::isfinite(p.y))return;
  robot_stamp_ns_=stamp;robot_position_={p.x,p.y};
  if(skeleton::confirmVisitedJunctions(graph_,robot_position_,options_.visit_radius,stamp)) {
    ++revision_;geometry_rebuilt_=false;sampled_at_=msg->header.stamp;publish(node);
  }
}
skeleton::GridInput SkeletonGraphManager::sample() const {
  skeleton::GridInput in;
  Eigen::Vector2d origin,size;map_->getRegion(origin,size);
  double resolution=map_->getResolution();
  in.geometry={int(std::ceil(size.x()/resolution)),int(std::ceil(size.y()/resolution)),resolution,origin.x(),origin.y(),0,frame_};
  in.cells=cv::Mat(in.geometry.height,in.geometry.width,CV_8UC1);
  for(int y=0;y<in.geometry.height;++y)for(int x=0;x<in.geometry.width;++x) {
    Eigen::Vector2i idx(x,y);int state=map_->getOccupancy(idx);
    uint8_t cell=skeleton::UNKNOWN;
    if(state==SDFMap2D::OCCUPIED)cell=skeleton::BLOCKED;
    else if(state==SDFMap2D::FREE)
      cell=map_->getInflateOccupancy(idx)==0?skeleton::FREE:skeleton::INFLATED;
    in.cells.at<uint8_t>(y,x)=cell;
  }
  return in;
}
std::vector<skeleton::Frontier> SkeletonGraphManager::sampleFrontiers() const {
  std::vector<skeleton::Frontier> out;
  if(!connect_frontiers_||!map_->frontier_map2d_)return out;
  auto append=[&](const auto& list,bool active) {
    for(const auto& f:list) {
      skeleton::Frontier copy;copy.id=f.id_;copy.active=active;copy.centroid={f.average_.x(),f.average_.y()};
      for(const auto& p:f.cells_){Eigen::Vector2i idx;map_->posToIndex(p,idx);copy.cells.emplace_back(idx.x(),idx.y());}
      out.push_back(std::move(copy));
    }
  };
  append(map_->frontier_map2d_->activeFrontiers(),true);
  append(map_->frontier_map2d_->dormantFrontiers(),false);
  return out;
}
skeleton::Route SkeletonGraphManager::routeToFrontier(cv::Point2d start,cv::Point2d frontier,double radius) const {
  skeleton::Route failed;failed.reason="FRONTIER_ATTACHMENT_NOT_READY";
  // Match current members, not only an old ID or centroid from an async snapshot.
  for(const auto& f:sampleFrontiers())if(cv::norm(f.centroid-frontier)<1e-6) {
    const auto signature=skeleton::frontierFingerprint({f});
    for(const auto& link:links_)if(link.frontier_id==f.id&&link.connected&&link.geometry_signature==signature) {
      auto state=[&](cv::Point p)->uint8_t {
        Eigen::Vector2i idx(p.x,p.y);int occupancy=map_->getOccupancy(idx);
        if(occupancy==SDFMap2D::UNKNOWN)return skeleton::UNKNOWN;
        if(occupancy!=SDFMap2D::FREE)return skeleton::BLOCKED;
        return map_->getInflateOccupancy(idx)==0?skeleton::FREE:skeleton::INFLATED;
      };
      return skeleton::shortestRoute(graph_,start,link,state,radius);
    }
  }
  return failed;
}
bool SkeletonGraphManager::validateLinks(const std::vector<skeleton::Frontier>& current) {
  bool changed=skeleton::discardStaleLinks(links_,current);
  auto free=[&](cv::Point p){Eigen::Vector2i idx(p.x,p.y);return map_->getOccupancy(idx)==SDFMap2D::FREE&&map_->getInflateOccupancy(idx)==0;};
  auto state=[&](cv::Point p)->uint8_t {
    Eigen::Vector2i idx(p.x,p.y);int value=map_->getOccupancy(idx);
    if(value==SDFMap2D::UNKNOWN)return skeleton::UNKNOWN;
    if(value==SDFMap2D::FREE)return map_->getInflateOccupancy(idx)==0?skeleton::FREE:skeleton::INFLATED;
    return skeleton::BLOCKED;
  };
  for(auto& c:links_)if(c.connected) {
    // Validate shortened segments too, not just the old raster path: a newly
    // observed obstacle may fall on the shortcut but outside the raster path.
    bool safe=skeleton::frontierPathTraversable(graph_.geometry,c.display_path,state,false)&&
      skeleton::frontierAssociationClear(graph_.geometry,c.approach,c.centroid,state);
    for(size_t i=0;i<c.pixels.size()&&safe;++i) {
      auto p=c.pixels[i];safe=free(p);
      if(i){auto prev=c.pixels[i-1];if(p.x!=prev.x&&p.y!=prev.y)safe&=free({p.x,prev.y})&&free({prev.x,p.y});}
    }
    if(!safe){c.connected=false;c.reason="MAP_CHANGED";c.junction_id=0;c.approach={};c.path.clear();c.display_path.clear();c.pixels.clear();c.length=0;changed=true;}
  }
  return changed;
}
void SkeletonGraphManager::tick() {
  auto node=node_.lock();if(!node)return;
  try {
    auto current=sampleFrontiers();
    if(validateLinks(current)) {
      accepted_frontiers_=0;++revision_;publish(node);
    }
    if(job_.valid()) {
      if(job_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
      auto result=job_.get();
      // Accept the timestamped snapshot, then refresh links if frontiers
      // changed during the job. Dropping every changed snapshot would starve
      // publication while the robot is continuously exploring.
      // Preserve visits received while the worker held an older graph snapshot.
      skeleton::inheritJunctionVisits(result.graph,graph_,options_.visit_radius);
      const double age=(node->now().nanoseconds()-robot_stamp_ns_)*1e-9;
      if(robot_stamp_ns_>0&&age>=0&&age<=odom_timeout_)
        skeleton::confirmVisitedJunctions(result.graph,robot_position_,options_.visit_radius,robot_stamp_ns_);
      graph_=std::move(result.graph);build_ms_=result.build_ms;link_ms_=result.link_ms;
      geometry_rebuilt_=result.geometry_rebuilt;
      // Link-only refreshes must not forget accumulated small map changes.
      if(geometry_rebuilt_)gate_->accept(result.fingerprint);
      link_gate_->accept(std::move(result.fingerprint));
      links_=std::move(result.links);accepted_frontiers_=result.frontier_signature;
      if(validateLinks(current))accepted_frontiers_=0;
      ++revision_;publish(node);
      size_t connected=0;for(const auto& c:links_)connected+=c.connected;
      RCLCPP_INFO(node->get_logger(),"[Skeleton] revision=%lu nodes=%zu edges=%zu removed_deadends=%u frontier_links=%zu/%zu geometry_rebuilt=%d sample=%.1fms build=%.1fms links=%.1fms",
        revision_,graph_.nodes.size(),graph_.edges.size(),graph_.removed_deadends,connected,links_.size(),geometry_rebuilt_,sampling_ms_,build_ms_,link_ms_);
    }
    auto start=Clock::now();auto in=sample();auto hashes=skeleton::fingerprint(in,tile_size_);
    auto signature=skeleton::frontierFingerprint(current);
    const bool connectivity_changed=options_.min_island_area>0&&link_gate_->shouldRebuild(hashes);
    bool rebuild=gate_->shouldRebuild(hashes)||connectivity_changed;
    bool links_changed=connect_frontiers_&&!current.empty()&&link_gate_->shouldRebuild(hashes);
    if(!rebuild&&!links_changed&&signature==accepted_frontiers_)return;
    changed_tiles_=gate_->changedTiles(hashes);sampled_at_=node->now();sampling_ms_=milliseconds(start);
    job_=std::async(std::launch::async,[in=std::move(in),hashes=std::move(hashes),options=options_,previous=graph_,current=std::move(current),signature,rebuild,connect_frontiers=connect_frontiers_,max_association_distance=max_association_distance_]() mutable {
      auto start=Clock::now();auto graph=rebuild?skeleton::build(in,options,previous):std::move(previous);
      double build_ms=milliseconds(start);start=Clock::now();auto links=connect_frontiers?skeleton::connectFrontiers(in,graph,current,max_association_distance):std::vector<skeleton::FrontierLink>{};
      return Result{std::move(graph),std::move(hashes),std::move(links),signature,rebuild,build_ms,milliseconds(start)};
    });
  }catch(const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(),"[Skeleton] update failed; retaining last graph: %s",e.what());
  }
}
void SkeletonGraphManager::publish(const rclcpp::Node::SharedPtr& node) {
  plan_env::msg::SkeletonGraph msg;msg.header.frame_id=frame_;msg.header.stamp=sampled_at_;
  msg.map_info.resolution=graph_.geometry.resolution;msg.map_info.width=graph_.geometry.width;msg.map_info.height=graph_.geometry.height;
  msg.map_info.origin.position.x=graph_.geometry.origin_x;msg.map_info.origin.position.y=graph_.geometry.origin_y;
  msg.map_info.origin.orientation.z=std::sin(graph_.geometry.yaw/2);msg.map_info.origin.orientation.w=std::cos(graph_.geometry.yaw/2);
  msg.revision=revision_;msg.skeleton_pixels=graph_.skeleton_pixels;msg.pruned_branches=graph_.pruned_branches;
  msg.changed_tiles=changed_tiles_;msg.sampling_ms=sampling_ms_;msg.build_ms=build_ms_;msg.thinning_backend=skeleton::thinningBackend();
  for(const auto& n:graph_.nodes) {
    plan_env::msg::SkeletonNode out;out.node_id=n.id;out.node_type=uint8_t(n.type);out.pixel_x=n.pixel.x;out.pixel_y=n.pixel.y;
    out.position=point(n.position);out.neighbor_node_ids=n.neighbors;out.neighbor_count=n.neighbor_count;
    // Cell containing the node position; a merged arithmetic centre need not be a pixel centre.
    out.anchor_pixel_x.push_back(n.pixel.x);out.anchor_pixel_y.push_back(n.pixel.y);
    out.clearance=n.clearance;out.near_unknown_boundary=n.near_unknown_boundary;out.active=n.active;msg.nodes.push_back(std::move(out));
    auto& saved=msg.nodes.back();
    saved.visit_state=n.type==skeleton::NodeType::JUNCTION?uint8_t(n.visit_state):plan_env::msg::SkeletonNode::NOT_APPLICABLE;
    if(n.type==skeleton::NodeType::JUNCTION) {
      saved.visit_radius=options_.visit_radius;saved.confirmed_at_ns=n.confirmed_at_ns;
      saved.visit_position=point(n.visit_position,0);
    }
  }
  for(const auto& e:graph_.edges) {
    plan_env::msg::SkeletonEdge out;out.edge_id=e.id;out.start_node_id=e.start;out.end_node_id=e.end;out.length=e.length;out.min_clearance=e.min_clearance;
    for(auto p:e.pixels){out.pixel_x.push_back(p.x);out.pixel_y.push_back(p.y);}
    for(auto p:e.path)out.path.push_back(point(p));
    for(auto p:e.display_path)out.display_path.push_back(point(p));
    msg.edges.push_back(std::move(out));
  }
  msg.removed_deadends=graph_.removed_deadends;msg.geometry_rebuilt=geometry_rebuilt_;msg.frontier_link_ms=link_ms_;
  for(const auto& c:links_) {
    plan_env::msg::SkeletonFrontierLink out;out.frontier_id=c.frontier_id;out.active=c.active;out.connected=c.connected;
    out.junction_node_id=c.junction_id;out.frontier_centroid=point(c.centroid);out.approach=point(c.approach);
    out.length=c.length;out.reason=c.reason;
    for(auto p:c.pixels){out.pixel_x.push_back(p.x);out.pixel_y.push_back(p.y);}
    for(auto p:c.path)out.path.push_back(point(p));
    for(auto p:c.display_path)out.display_path.push_back(point(p));
    msg.frontier_links.push_back(std::move(out));
  }
  graph_pub_->publish(msg);
  using Marker=visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray markers;
  Marker clear;clear.header=msg.header;clear.action=Marker::DELETEALL;markers.markers.push_back(clear);
  auto marker=[&](const std::string& ns,int type,float red,float green,float blue) {
    Marker m;m.header=msg.header;m.ns=ns;m.type=type;m.action=Marker::ADD;m.pose.orientation.w=1;
    m.color.r=red;m.color.g=green;m.color.b=blue;m.color.a=1;return m;
  };
  auto edges=marker("edges",Marker::LINE_LIST,0,0,1);edges.scale.x=edge_width_;
  for(const auto& e:graph_.edges)for(size_t i=1;i<e.display_path.size();++i){edges.points.push_back(point(e.display_path[i-1],z_));edges.points.push_back(point(e.display_path[i],z_));}
  markers.markers.push_back(std::move(edges));
  auto endpoints=marker("endpoints",Marker::SPHERE_LIST,1,0,0);
  auto junctions=marker("junctions",Marker::SPHERE_LIST,.60,1,.60);
  auto confirmed=marker("junctions_confirmed",Marker::SPHERE_LIST,0,.40,.08);
  auto loops=marker("loop_anchors",Marker::SPHERE_LIST,.65,0,1);
  for(auto* m:{&endpoints,&junctions,&confirmed,&loops})m->scale.x=m->scale.y=m->scale.z=node_scale_;
  int label_id=0;
  for(const auto& n:graph_.nodes) {
    auto* m=n.type==skeleton::NodeType::ENDPOINT?&endpoints:n.type==skeleton::NodeType::JUNCTION?&junctions:&loops;
    if(n.type==skeleton::NodeType::JUNCTION&&n.visit_state==skeleton::VisitState::CONFIRMED)m=&confirmed;
    m->points.push_back(point(n.position,z_));
    if(labels_) {
      auto text=marker("node_ids",Marker::TEXT_VIEW_FACING,.15,.15,.15);text.id=label_id++;
      text.pose.position=point(n.position,z_+node_scale_);text.scale.z=label_scale_;text.text=std::to_string(n.id);
      markers.markers.push_back(std::move(text));
    }
  }
  markers.markers.push_back(std::move(endpoints));markers.markers.push_back(std::move(junctions));markers.markers.push_back(std::move(loops));
  markers.markers.push_back(std::move(confirmed));
  auto connections=marker("frontier_links",Marker::LINE_LIST,0,.75,1);connections.scale.x=edge_width_;
  auto associations=marker("frontier_associations",Marker::LINE_LIST,1,.55,0);associations.scale.x=edge_width_;
  int frontier_label=0;
  for(const auto& c:links_)if(c.connected) {
    for(size_t i=1;i<c.display_path.size();++i) {
      connections.points.push_back(point(c.display_path[i-1],z_));
      connections.points.push_back(point(c.display_path[i],z_));
    }
    // Orange dashes reach the unchanged sphere; they are not traversability edges.
    if(cv::norm(c.approach-c.centroid)>1e-9) {
      double length=std::hypot(cv::norm(c.centroid-c.approach),frontier_z_-z_);
      for(double d=0;d<length;d+=2*association_dash_length_) {
        for(double t:{d/length,std::min(d+association_dash_length_,length)/length})
          associations.points.push_back(point(c.approach+(c.centroid-c.approach)*t,z_+(frontier_z_-z_)*t));
      }
    }
    if(labels_) {
      auto text=marker("frontier_ids",Marker::TEXT_VIEW_FACING,0,.5,.7);text.id=frontier_label++;
      text.pose.position=point(c.centroid,frontier_z_+node_scale_);text.scale.z=label_scale_;text.text="F"+std::to_string(c.frontier_id);
      markers.markers.push_back(std::move(text));
    }
  }
  markers.markers.push_back(std::move(connections));
  markers.markers.push_back(std::move(associations));
  markers_pub_->publish(markers);
}
}
