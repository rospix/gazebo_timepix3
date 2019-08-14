#ifndef RADIATION_UTILS_OBSTACLE_H
#define RADIATION_UTILS_OBSTACLE_H

#include <string>
#include <vector>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <gazebo/physics/physics.hh>
#include <gazebo_timepix/geometry_utils.h>

class Obstacle {
public:
  ~Obstacle();
  Obstacle(unsigned int gazebo_id, std::string material, Eigen::Vector3d world_position, Eigen::Quaterniond orientation, Eigen::Vector3d scale);

  unsigned int           getId();
  std::string            getMaterial();
  Eigen::Vector3d        getRelativePosition();
  Eigen::Quaterniond     getRelativeOrientation();
  Eigen::Vector3d        getScale();
  std::vector<Rectangle> getSides();

  void updatePose(Eigen::Vector3d pos, Eigen::Quaterniond ori, Eigen::Vector3d sca);

private:
  unsigned int           gazebo_id = 1;
  std::string            material;
  Eigen::Vector3d        relative_position;
  Eigen::Quaterniond     relative_orientation;
  Eigen::Vector3d        scale;
  std::vector<Rectangle> sides;
  void                   buildSides();
};

Obstacle::~Obstacle() {
}

Obstacle::Obstacle(unsigned int gazebo_id, std::string material, Eigen::Vector3d relative_position, Eigen::Quaterniond relative_orientation,
                   Eigen::Vector3d scale) {
  this->gazebo_id            = gazebo_id;
  this->material             = material;
  this->relative_position    = relative_position;
  this->relative_orientation = relative_orientation;
  this->scale                = scale;
  buildSides();
}

unsigned int Obstacle::getId() {
  return gazebo_id;
}

std::string Obstacle::getMaterial() {
  return material;
}

Eigen::Vector3d Obstacle::getRelativePosition() {
  return relative_position;
}

Eigen::Quaterniond Obstacle::getRelativeOrientation() {
  return relative_orientation;
}

Eigen::Vector3d Obstacle::getScale() {
  return scale;
}

std::vector<Rectangle> Obstacle::getSides() {
  return sides;
}

void Obstacle::buildSides() {
  Eigen::Vector3d A(scale[0] / 2.0, -scale[1] / 2.0, -scale[2] / 2.0);
  Eigen::Vector3d B(scale[0] / 2.0, scale[1] / 2.0, -scale[2] / 2.0);
  Eigen::Vector3d C(scale[0] / 2.0, scale[1] / 2.0, scale[2] / 2.0);
  Eigen::Vector3d D(scale[0] / 2.0, -scale[1] / 2.0, scale[2] / 2.0);

  Eigen::Vector3d E(-scale[0] / 2.0, scale[1] / 2.0, -scale[2] / 2.0);
  Eigen::Vector3d F(-scale[0] / 2.0, -scale[1] / 2.0, -scale[2] / 2.0);
  Eigen::Vector3d G(-scale[0] / 2.0, -scale[1] / 2.0, scale[2] / 2.0);
  Eigen::Vector3d H(-scale[0] / 2.0, scale[1] / 2.0, scale[2] / 2.0);

  sides.push_back(Rectangle(A, B, C, D));
  sides.push_back(Rectangle(E, F, G, H));
  sides.push_back(Rectangle(B, E, H, C));
  sides.push_back(Rectangle(F, A, D, G));
  sides.push_back(Rectangle(F, E, B, A));
  sides.push_back(Rectangle(D, C, H, G));
}

#endif /* RADIATION_UTILS_OBSTACLE_H */
