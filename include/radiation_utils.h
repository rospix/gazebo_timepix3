#ifndef RADIATION_UTILS_H
#define RADIATION_UTILS_H

#include <string>
#include <set>
#include <eigen3/Eigen/Core>
#include <geometry_utils.h>
#include <gazebo/common/Time.hh>

class Source {
public:
  Source();
  ~Source();
  Source(std::string material = "Am", double activity = 10e9, Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0));

  std::string          material;
  double               activity;
  Eigen::Vector3d      position;
  double               apparent_activity = 0.0;
  gazebo::common::Time last_ray_time     = 0.0;
};

#endif
