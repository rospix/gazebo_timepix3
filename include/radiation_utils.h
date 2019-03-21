#ifndef RADIATION_UTILS_H
#define RADIATION_UTILS_H

#include <string>
#include <set>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <geometry_utils.h>
#include <ros/time.h>

class Source {
public:
  Source();
  ~Source();
  Source(unsigned int id, std::string material = "Am", double activity = 10e9, std::vector<double> apparent_activities = {0.0, 0.0, 0.0},
         Eigen::Vector3d relative_position = Eigen::Vector3d(0.0, 0.0, 0.0));
  unsigned int        id = 1;
  std::string         material;
  double              activity;
  std::vector<double> apparent_activities;
  std::vector<int>    exposed_sides;
  Eigen::Vector3d     relative_position;

  std::chrono::high_resolution_clock::time_point last_contact;
};

#endif
