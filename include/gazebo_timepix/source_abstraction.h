#ifndef SOURCE_ABSTRACTION_H
#define SOURCE_ABSTRACTION_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

/* This library provides wrapper objects for the 3D objects interacting with the simulated radiation */

typedef std::pair<int, double> SideProperty;

/* SourceAbstraction //{ */

class SourceAbstraction {
private:
  unsigned int gazebo_id;
  double       activity;
  double       energy;
  double       mass_att_coeff;  // attenuation of emitted particles by the sensor
  std::string  material;
  double       air_mass_att_coeff;  // atttenuation of emitted particles by air

  Eigen::Vector3d           relative_position;
  std::vector<SideProperty> side_properties;
  // int - index of exposed Timepix side
  // double - apparent activity of this source for indexed side

  std::vector<unsigned int> obstacles_in_path;

public:
  SourceAbstraction();
  ~SourceAbstraction();
  SourceAbstraction(unsigned int gazebo_id, std::string material, double activity, double energy, double mass_att_coeff, double air_mass_att_coeff,
                    Eigen::Vector3d relative_position);

  bool operator==(SourceAbstraction const &s1) {
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

  double getMassAttCoeff() {
    return mass_att_coeff;
  }

  Eigen::Vector3d getRelativePosition() {
    return relative_position;
  }

  std::vector<SideProperty> getSideProperties() {
    return side_properties;
  }

  void setSideProperties(std::vector<SideProperty> side_properties) {
    this->side_properties = side_properties;
  }

  void setRelativePosition(Eigen::Vector3d relative_position) {
    this->relative_position = relative_position;
  }

  void addObstacle(unsigned int obstacle_id) {
    obstacles_in_path.push_back(obstacle_id);
  }

  void removeObstacle(unsigned int obstacle_id) {
    obstacles_in_path.erase(std::remove(obstacles_in_path.begin(), obstacles_in_path.end(), obstacle_id), obstacles_in_path.end());
  }

  std::vector<unsigned int> getObstaclesInPath() {
    return obstacles_in_path;
  }

  double getAirMassAttCoeff() {
    return air_mass_att_coeff;
  }
};

SourceAbstraction::SourceAbstraction() {
}

SourceAbstraction::~SourceAbstraction() {
}

SourceAbstraction::SourceAbstraction(unsigned int gazebo_id, std::string material, double activity, double energy, double mass_att_coeff,
                                     double air_mass_att_coeff, Eigen::Vector3d relative_position) {

  this->gazebo_id          = gazebo_id;
  this->material           = material;
  this->activity           = activity;
  this->energy             = energy;
  this->mass_att_coeff     = mass_att_coeff;
  this->air_mass_att_coeff = air_mass_att_coeff;
  this->relative_position  = relative_position;
}
//}

#endif /* SOURCE_ABSTRACTION_H */
