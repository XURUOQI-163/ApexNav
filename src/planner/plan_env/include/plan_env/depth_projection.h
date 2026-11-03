#pragma once
#include <Eigen/Dense>
namespace apexnav_planner {
// Shared optical +Z pinhole projection; validity/range handling belongs to the caller.
inline Eigen::Vector3d projectDepthPixel(int u,int v,double depth,double fx,double fy,double cx,double cy,
                                        const Eigen::Matrix3d& rotation,const Eigen::Vector3d& position){
  return rotation*Eigen::Vector3d((u-cx)*depth/fx,(v-cy)*depth/fy,depth)+position;
}
}
