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

  std::chrono::high_resolution_clock::time_point getLastContact();

  void setApparentActivities(std::vector<double> apparent_activities);
  void setExposedSides(std::vector<int> exposed_sides);
  void setRelativePosition(Eigen::Vector3d relative_position);
  void setLastContact(std::chrono::high_resolution_clock::time_point last_contact);

private:
  unsigned int gazebo_id;
  std::string  material;
  double       activity;

  std::vector<double> apparent_activities;
  std::vector<int>    exposed_sides;
  Eigen::Vector3d     relative_position;

  std::chrono::high_resolution_clock::time_point last_contact;
};

Source::~Source() {
}

Source::Source(unsigned int gazebo_id, std::string material, double activity, Eigen::Vector3d relative_position) {
  this->gazebo_id         = gazebo_id;
  this->material          = material;
  this->activity          = activity;
  this->relative_position = relative_position;
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

std::chrono::high_resolution_clock::time_point Source::getLastContact() {
  return last_contact;
}

void Source::setApparentActivities(std::vector<double> apparent_activities) {
  this->apparent_activities = apparent_activities;
}

void Source::setExposedSides(std::vector<int> exposed_sides) {
  this->exposed_sides = exposed_sides;
}

void Source::setRelativePosition(Eigen::Vector3d relative_position) {
  this->relative_position = relative_position;
}

void Source::setLastContact(std::chrono::high_resolution_clock::time_point last_contact) {
  this->last_contact = last_contact;
}

#endif /* RADIATION_UTILS_SOURCE_H */
