#include <radiation_utils.h>

Source::Source() {
}
Source::~Source() {
}
Source::Source(unsigned int id, std::string material, double activity, std::vector<double> apparent_activities, Eigen::Vector3d relative_position) {
  this->id                  = id;
  this->material            = material;
  this->activity            = activity;
  this->apparent_activities = apparent_activities;
  this->relative_position   = relative_position;
}

Obstacle::Obstacle() {
}

Obstacle::~Obstacle() {
}

Obstacle::Obstacle(Eigen::Vector3d center, Cuboid cuboid, std::string material) {
  this->center = center;
  this->cuboid = cuboid;
  this->material = material;
}
