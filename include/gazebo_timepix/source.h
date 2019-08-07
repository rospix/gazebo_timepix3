#ifndef RADIATION_UTILS_SOURCE_H
#define RADIATION_UTILS_SOURCE_H

#include <string>
#include <vector>
#include <chrono>
#include <eigen3/Eigen/Core>

class Source {
public:
  ~Source();
  Source(unsigned int gazebo_id = 1, std::string material = "Am241", double activity = 10e9,
         Eigen::Vector3d relative_position = Eigen::Vector3d(0.0, 0.0, 0.0));

  unsigned int getId();
  std::string  getMaterial();
  double       getActivity();

  std::vector<double> getApparentActivities();
  std::vector<int>    getExposedSides();
  Eigen::Vector3d     getRelativePosition();
  Eigen::Vector3d     getWorldPosition();

  void setApparentActivities(std::vector<double> apparent_activities);
  void setExposedSides(std::vector<int> exposed_sides);
  void setWorldPosition(Eigen::Vector3d world_position);
  void updateRelativePosition(Eigen::Vector3d detector_position);

private:
  unsigned int gazebo_id;
  std::string  material;
  double       activity;

  std::vector<double> apparent_activities;
  std::vector<int>    exposed_sides;
  Eigen::Vector3d     world_position;
  Eigen::Vector3d     relative_position = Eigen::Vector3d::Zero();
};

Source::~Source() {
}

Source::Source(unsigned int gazebo_id, std::string material, double activity, Eigen::Vector3d world_position) {
  this->gazebo_id      = gazebo_id;
  this->material       = material;
  this->activity       = activity;
  this->world_position = world_position;
}

unsigned int Source::getId() {
  return gazebo_id;
}

std::string Source::getMaterial() {
  return material;
}

double Source::getActivity() {
  return activity;
}

std::vector<double> Source::getApparentActivities() {
  return apparent_activities;
}

std::vector<int> Source::getExposedSides() {
  return exposed_sides;
}

Eigen::Vector3d Source::getRelativePosition() {
  return relative_position;
}

Eigen::Vector3d Source::getWorldPosition() {
  return world_position;
}

void Source::setApparentActivities(std::vector<double> apparent_activities) {
  this->apparent_activities = apparent_activities;
}

void Source::setExposedSides(std::vector<int> exposed_sides) {
  this->exposed_sides = exposed_sides;
}

void Source::setWorldPosition(Eigen::Vector3d world_position) {
  this->world_position = world_position;
}

void Source::updateRelativePosition(Eigen::Vector3d detector_position) {
  relative_position = world_position - detector_position;
}

#endif /* RADIATION_UTILS_SOURCE_H */
