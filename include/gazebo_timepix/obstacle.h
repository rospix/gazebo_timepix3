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
  Obstacle(unsigned int gazebo_id, std::string material, gazebo::physics::ModelPtr collider_);

  unsigned int              getId();
  std::string               getMaterial();
  gazebo::physics::ModelPtr getCollider();

  std::chrono::high_resolution_clock::time_point getLastContact();

  void setLastContact(std::chrono::high_resolution_clock::time_point last_contact);


private:
  unsigned int              gazebo_id = 1;
  std::string               material;
  gazebo::physics::ModelPtr collider_;

  std::chrono::high_resolution_clock::time_point last_contact;
};

Obstacle::~Obstacle() {
}

Obstacle::Obstacle(unsigned int gazebo_id, std::string material, gazebo::physics::ModelPtr collider_) {
  this->gazebo_id = gazebo_id;
  this->material  = material;
  this->collider_ = collider_;
}

unsigned int Obstacle::getId() {
  return gazebo_id;
}

std::string Obstacle::getMaterial() {
  return material;
}

gazebo::physics::ModelPtr Obstacle::getCollider() {
  return collider_;
}

std::chrono::high_resolution_clock::time_point Obstacle::getLastContact() {
  return last_contact;
}

void Obstacle::setLastContact(std::chrono::high_resolution_clock::time_point last_contact) {
  this->last_contact = last_contact;
}

#endif /* RADIATION_UTILS_OBSTACLE_H */
