#include <controller/mpc.h>
#include <cassert>
int main(int argc, char * *argv){
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("topology_mpc_test");
  n->declare_parameter("mpc.dt", .05);
  n->declare_parameter("mpc.predict_steps", 20);
  n->declare_parameter("mpc.du_threshold", .001);
  n->declare_parameter("mpc.max_iter", 4);
  n->declare_parameter("mpc.delay_num", 0);
  n->declare_parameter("mpc.matrix_q", std::vector<double>{30, 30, 2.5, 2.5});
  n->declare_parameter("mpc.matrix_r", std::vector<double>{.1, .01});
  n->declare_parameter("mpc.matrix_rd", std::vector<double>{.01, .01});
  n->declare_parameter("max_correction_vel", .4);
  n->declare_parameter("max_correction_omega", 1.);
  n->declare_parameter("mpc.strict_command_limits", true);
  n->declare_parameter("mpc.max_accel", .5);
  n->declare_parameter("mpc.max_angular_accel", 1.);
  MPC controller;
  controller.init(n);
  controller.setOdom({0, 0, 0, 0});
  controller.setPreviousCommand({0, 0});
  std::vector<Eigen::Vector3d> reference(20, Eigen::Vector3d(1, 0, 0));
  auto v = controller.calCmd(reference);
  assert(controller.healthy());
  assert(v[0] >= -1e-5 && v[0] <= .02501 && std::abs(v[1]) <= .05001);
  assert(controller.calCmd({}).norm() == 0 && !controller.healthy());
  reference.assign(20, Eigen::Vector3d(0, 0, 3.1));
  controller.reset();
  controller.setOdom({0, 0, -3.1, 0});
  auto rotation = controller.calCmd(reference);
  assert(controller.healthy() && rotation.allFinite() && std::abs(rotation[1]) <= .05001);
  // Inconsistent first-command bounds must fail closed, not replay the last
  // successful command when OSQP rejects the problem.
  controller.setPreviousCommand({5, 5});
  auto failed = controller.calCmd(reference);
  assert(!controller.healthy() && failed.norm() == 0.);
  std::cout <<
  "PASS MPC: first-command bounds, invalid-reference/solver fail stop, pi-crossing rotation\n";
  rclcpp::shutdown();
}
