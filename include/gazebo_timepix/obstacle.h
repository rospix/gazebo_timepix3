#ifndef RADIATION_UTILS_OBSTACLE_H
#define RADIATION_UTILS_OBSTACLE_H

#include <string>
#include <vector>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <gazebo/physics/physics.hh>

class Obstacle {
public:
  ~Obstacle();
  Obstacle(unsigned int gazebo_id, std::string material, Eigen::Vector3d world_position, Eigen::Quaterniond orientation, Eigen::Vector3d scale);

  unsigned int       getId();
  std::string        getMaterial();
  Eigen::Vector3d    getWorldPosition();
  Eigen::Vector3d    getRelativePosition();
  Eigen::Quaterniond getOrientation();
  Eigen::Vector3d    getScale();
  void               updateRelativePosition(Eigen::Vector3d detector_position);
  void               setWorldPosition(Eigen::Vector3d world_position);
  void               setOrientation(Eigen::Quaterniond world_position);

private:
  unsigned int       gazebo_id = 1;
  std::string        material;
  Eigen::Vector3d    relative_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d    world_position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d    scale;
};

Obstacle::~Obstacle() {
}

Obstacle::Obstacle(unsigned int gazebo_id, std::string material, Eigen::Vector3d world_position, Eigen::Quaterniond orientation, Eigen::Vector3d scale) {
  this->gazebo_id      = gazebo_id;
  this->material       = material;
  this->world_position = world_position;
  this->orientation    = orientation;
  this->scale          = scale;
}

unsigned int Obstacle::getId() {
  return gazebo_id;
}

std::string Obstacle::getMaterial() {
  return material;
}

Eigen::Vector3d Obstacle::getWorldPosition() {
  return world_position;
}

Eigen::Vector3d Obstacle::getRelativePosition() {
  return relative_position;
}

Eigen::Quaterniond Obstacle::getOrientation() {
  return orientation;
}

Eigen::Vector3d Obstacle::getScale() {
  return scale;
}

void Obstacle::setWorldPosition(Eigen::Vector3d world_position) {
  this->world_position = world_position;
}

void Obstacle::setOrientation(Eigen::Quaterniond orientation) {
  this->orientation = orientation;
}

void Obstacle::updateRelativePosition(Eigen::Vector3d detector_position) {
  relative_position = world_position - detector_position;
}

#endif /* RADIATION_UTILS_OBSTACLE_H */
