#ifndef RADIATION_UTILS_H
#define RADIATION_UTILS_H

#include <string>
#include <set>
#include <eigen3/Eigen/Core>
#include <geometry_utils.h>
#include <ros/time.h>

class Source {
public:
  Source();
  ~Source();
  Source(std::string material = "Am", double activity = 10e9, Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0));

  std::string     material;
  double          activity;
  Eigen::Vector3d position;
  double          apparent_activity = 0.0;
  ros::Time       last_contact      = ros::TIME_MIN;
  double          time_slot_percentage = 1.0;
};

#endif
