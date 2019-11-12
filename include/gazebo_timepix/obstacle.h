#ifndef RADIATION_UTILS_OBSTACLE_H
#define RADIATION_UTILS_OBSTACLE_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ignition/math.hh>

class Obstacle {
private:
  unsigned int gazebo_id;
  std::string  material;

  Eigen::Vector3d    relative_position;
  Eigen::Quaterniond relative_orientation;
  Eigen::Vector3d    size;

public:
  Obstacle();
  ~Obstacle();
  Obstacle(unsigned int gazebo_id, std::string material, Eigen::Vector3d relative_position, Eigen::Quaterniond relative_orientation, Eigen::Vector3d size);

  bool operator==(Obstacle const &o1) {
    return this->gazebo_id == o1.gazebo_id;
  }

  bool operator==(const unsigned int &i) {
    return this->gazebo_id == i;
  }

  bool operator==(unsigned int &i) {
    return this->gazebo_id == i;
  }

  unsigned int getId() {
    return gazebo_id;
  }

  std::string getMaterial() {
    return material;
  }

  Eigen::Vector3d getRelativePosition() {
    return relative_position;
  }

  void setRelativePosition(Eigen::Vector3d relative_position) {
    this->relative_position = relative_position;
  }

  Eigen::Quaterniond getRelativeOrientation() {
    return relative_orientation;
  }

  void setRelativeOrientation(Eigen::Quaterniond relative_orientation) {
    this->relative_orientation = relative_orientation;
  }

  Eigen::Vector3d getSize() {
    return size;
  }
};

Obstacle::Obstacle() {
}

Obstacle::~Obstacle() {
}

Obstacle::Obstacle(unsigned int gazebo_id, std::string material, Eigen::Vector3d relative_position, Eigen::Quaterniond relative_orientation,
                   Eigen::Vector3d size) {
  this->gazebo_id            = gazebo_id;
  this->material             = material;
  this->relative_position    = relative_position;
  this->relative_orientation = relative_orientation;
  this->size                 = size;
}

#endif /* RADIATION_UTILS_OBSTACLE_H */
