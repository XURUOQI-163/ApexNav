#pragma once
#include <topology_exploration/rolling_local_map.h>
#include <topology_exploration/local_planner.h>
#include <topology_exploration/graph_planner.h>
#include <topology_exploration/controller_guard.h>
#include <topology_exploration/snapshot.h>
#include <future>
#include <controller/mpc.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
namespace apexnav_planner::topology {
class ExplorerNode : public rclcpp::Node {
public:
ExplorerNode();
~ExplorerNode() override;
void initialize();
void finalize();
private:
using Image = sensor_msgs::msg::Image;
using Odom = nav_msgs::msg::Odometry;
using Sync = message_filters::sync_policies::ExactTime<Image, Odom>;
void depth(const Image::ConstSharedPtr &image, const Odom::ConstSharedPtr &pose);
void odom(const Odom::SharedPtr msg);
void rgb(const Image::ConstSharedPtr &image);
void attach(int64_t stamp);
void tick();
void plan();
void visualize();
void reset(bool new_simulation = true);
void saveSnapshot(bool wait = false);
void stop(const std::string &reason);
void startObserve();
void failGoal(const std::string &reason);
GridView grid()const;
std::vector<Connection> connections();
std::vector<Vec2> remainingRoute();
std::vector<Eigen::Vector3d> reference(const std::vector<Vec2> &path)const;
bool fresh()const;
void publishCommand(Command command);
template<class T>T parameter(const std::string &name, T value){
  return declare_parameter<T>(name, value);
}
std::unique_ptr<RollingLocalMap> map_;
std::unique_ptr<sparse::TopologyGraph> graph_;
std::unique_ptr<SnapshotWriter> snapshot_;
std::future<void> snapshot_job_;
GraphPlanner global_;
LocalPlanner local_;
MPC mpc_;
ControllerGuard guard_;
MapConfig map_cfg_;
GuardConfig control_cfg_;
int horizon_ = 20;
double dt_ = .05, fx_ = 388.1910413097385, fy_ = 422.0475153598262, cx_ = 320, cy_ = 240;
double sensor_max_ = 4.99, min_height_ = .05, max_height_ = 1.25, depth_timeout_ = 1., odom_timeout_ = .5;
double camera_yaw_ = 0, yaw_ = 0, speed_ = 0, angular_ = 0, observe_turn_ = 0, progress_time_ = 0,
       last_plan_ = 0;
Vec2 position_, progress_position_;
int64_t reset_stamp_ = 0, odom_stamp_ = 0, depth_stamp_ = 0, graph_stamp_ = 0;
bool have_odom_ = false, seeded_ = false, observing_ = true, external_stop_ = false,
     localization_fault_ = false;
std::string state_ = "WAIT_DATA", reason_ = "waiting for synchronized depth and odometry";
std::string reported_state_;
std::optional<Route> route_;
size_t route_cursor_ = 1;
LocalPath local_path_;
uint64_t trace_anchor_ = 0;
std::vector<Vec2> trace_;
std::map<int64_t, Image::ConstSharedPtr> rgb_cache_;
std::map<int64_t, sparse::DirectionalImage> views_;
std::shared_ptr<message_filters::Subscriber<Image> > depth_sub_;
std::shared_ptr<message_filters::Subscriber<Odom> > pose_sub_;
std::shared_ptr<message_filters::Synchronizer<Sync> > sync_;
rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
rclcpp::Subscription<Image>::SharedPtr rgb_sub_;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_, controller_reset_service_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_, local_path_pub_;
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
rclcpp::TimerBase::SharedPtr control_timer_, vis_timer_;
rclcpp::TimerBase::SharedPtr snapshot_timer_;
};
}
