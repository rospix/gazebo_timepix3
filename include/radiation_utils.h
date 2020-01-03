#ifndef RADIATION_UTILS_H
#define RADIATION_UTILS_H

#include <string>
#include <set>
#include <chrono>
#include <vector>
#include <eigen3/Eigen/Core>
#include <geometry_visual_utils/radiation_geometry_utils.h>
#include <ros/time.h>
#include <materials.h>

/*
 * This library provides wrapper objects for the 3D objects interacting with the simulated radiation
 *
  */

class Source {
public:
  Source();
  ~Source();
  Source(unsigned int id, std::string material = "Am241", double activity = 10e9, Eigen::Vector3d relative_position = Eigen::Vector3d(0.0, 0.0, 0.0));
  unsigned int        id = 1;
  std::string         material;
  double              activity;
  std::vector<double> apparent_activities;
  std::vector<int>    exposed_sides;
  Eigen::Vector3d     relative_position;

  std::chrono::high_resolution_clock::time_point last_contact;
};

class Obstacle {
public:
  Obstacle();
  ~Obstacle();
  Obstacle(Cuboid cuboid, Eigen::Vector3d center, Eigen::Quaterniond orientation, Material material);

  unsigned int       id = 1;
  Material           material;
  Eigen::Vector3d    center;
  Eigen::Quaterniond orientation;
  Cuboid             cuboid;

  std::chrono::high_resolution_clock::time_point last_contact;
};

#endif
