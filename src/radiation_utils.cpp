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

Obstacle::Obstacle(Cuboid cuboid, Eigen::Vector3d center, Eigen::Quaterniond orientation, Material material) {
  this->cuboid = cuboid;
  this->center = center;
  this->orientation = orientation;
  this->material = material;
}
