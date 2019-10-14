#ifndef RADIATION_UTILS_SOURCE_H
#define RADIATION_UTILS_SOURCE_H

#include <eigen3/Eigen/Core>
#include <iostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

/* This library provides wrapper objects for the 3D objects interacting with the simulated radiation */

/* Triplet struct //{ */
struct Triplet
{
  int    side_index;
  double apparent_activity;
  double diagnonal_absorption_prob;

  bool operator>(const Triplet &t) const {
    return side_index > t.side_index;
  }

  bool operator<(const Triplet &t) const {
    return side_index < t.side_index;
  }
  bool operator==(const Triplet &t) const {
    return side_index == t.side_index;
  }
};
//}

/* Source //{ */
class Source {
private:
  unsigned int gazebo_id;
  double       activity;
  double       energy;
  std::string  material;
  double       timepix_photoabsorption_coeff;

  Eigen::Vector3d   relative_position;
  std::set<Triplet> side_properties;
  // int - indices for exposed Timepix sides
  // double - apparent activity for corresponding sides
  // double - timepix_diagonal_absorption_probability (normalization constant)

public:
  Source();
  ~Source();
  Source(unsigned int gazebo_id, std::string material, double activity, Eigen::Vector3d relative_position);

  bool operator==(Source const &s1) {
    return this->gazebo_id == s1.gazebo_id;
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

  double getActivity() {
    return activity;
  }

  double getEnergy() {
    return energy;
  }

  Eigen::Vector3d getRelativePosition() {
    return relative_position;
  }
  std::set<Triplet> getSideProperties() {
    return side_properties;
  }

  void setSideProperties(std::set<Triplet> side_properties) {
    this->side_properties = side_properties;
  }

  double getTimepixPhotoAbsorptionCoeff() {
    return timepix_photoabsorption_coeff;
  }

  void setTimepixPhotoAbsorptionCoeff(double timepix_photoabsorption_coeff) {
    this->timepix_photoabsorption_coeff = timepix_photoabsorption_coeff;
  }

  void setRelativePosition(Eigen::Vector3d relative_position) {
    this->relative_position = relative_position;
  }
};

Source::Source() {
}

Source::~Source() {
}

Source::Source(unsigned int gazebo_id, std::string material, double activity, Eigen::Vector3d relative_position) {
  this->gazebo_id         = gazebo_id;
  this->material          = material;
  this->activity          = activity;
  this->relative_position = relative_position;
}
//}

#endif /* RADIATION_UTILS_SOURCE_H */
