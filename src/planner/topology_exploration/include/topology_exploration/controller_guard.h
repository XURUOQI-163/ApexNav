#pragma once
#include <plan_env/depth_descriptor.h>
#include <functional>
namespace apexnav_planner::topology {
using sparse::Vec2;
struct Command {
  double v = 0, w = 0;
};
struct GuardConfig {
  double max_v = .4, max_w = 1., accel = .5, alpha = 1., reaction = .15;
};
class ControllerGuard {
public:
explicit ControllerGuard(GuardConfig cfg = {}) : cfg_(cfg){
}
Command last()const {
  return previous_;
}
void stop(){
  previous_ = {};
}
Command limitAndCheck(Command requested, Vec2 pose, double yaw, double dt,
                      const std::function<bool(Vec2)> &safe, bool fresh){
  if (!fresh || !std::isfinite(requested.v) || !std::isfinite(requested.w) || dt <= 0 || dt > .25 ||
      !safe(pose)) {
    stop();
    return {};
  }
  requested.v = std::clamp(requested.v, 0., cfg_.max_v);
  requested.w = std::clamp(requested.w, -cfg_.max_w, cfg_.max_w);
  requested.v = std::clamp(requested.v, previous_.v - cfg_.accel * dt, previous_.v + cfg_.accel * dt);
  requested.w = std::clamp(requested.w, previous_.w - cfg_.alpha * dt, previous_.w + cfg_.alpha * dt);
  // Check the actual command after limiting, then both decelerating axes.
  Vec2 p = pose;
  double heading = yaw, v = requested.v, w = requested.w;
  const double h = .01, horizon = cfg_.reaction + dt + std::max(v / cfg_.accel, std::abs(w) / cfg_.alpha);
  for (double t = 0; t <= horizon; t += h) {
    p = p + sparse::direction(heading + w * h * .5) * (v * h);
    heading += w * h;
    if (!safe(p)) {
      stop();
      return {};
    }
    if (t >= cfg_.reaction + dt) {
      v = std::max(0., v - cfg_.accel * h);
      w = std::copysign(std::max(0., std::abs(w) - cfg_.alpha * h), w);
    }
  }
  previous_ = requested;
  return requested;
}
private:
GuardConfig cfg_;
Command previous_;
};
}
