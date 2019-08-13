#ifndef RADIATION_UTILS_SOURCE_H
#define RADIATION_UTILS_SOURCE_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>
#include <gazebo_timepix/geometry_utils.h>

class Source {
public:
  ~Source();
  Source(unsigned int gazebo_id, std::string material, double activity, double energy, Eigen::Vector3d relative_position,
         std::vector<double> apparent_activities, std::vector<int> exposed_sides, double photoabsorption_coeff, double diagnonal_absorption_prob);

  unsigned int     getId();
  std::string      getMaterial();
  double           getActivity();
  double           getEnergy();
  double           getPhotoAbsorptionCoeff();
  double           getDiagonalAbsorptionProb();
  std::vector<int> getExposedSides();

  std::vector<double> getApparentActivities();
  Eigen::Vector3d     getRelativePosition();
  Eigen::Vector3d     getWorldPosition();

  void setApparentActivities(std::vector<double> apparent_activities);
  void setWorldPosition(Eigen::Vector3d world_position);

private:
  unsigned int gazebo_id;
  std::string  material;
  double       activity;
  double       energy;
  double       diagnonal_absorption_prob;
  double       photoabsorption_coeff;

  std::vector<double> apparent_activities;
  std::vector<int>    exposed_sides;
  Eigen::Vector3d     relative_position;
};

Source::~Source() {
}

Source::Source(unsigned int gazebo_id, std::string material, double activity, double energy, Eigen::Vector3d relative_position,
               std::vector<double> apparent_activities, std::vector<int> exposed_sides, double photoabsorption_coeff, double diagnonal_absorption_prob) {
  this->gazebo_id                 = gazebo_id;
  this->material                  = material;
  this->activity                  = activity;
  this->energy                    = energy;
  this->relative_position         = relative_position;
  this->apparent_activities       = apparent_activities;
  this->exposed_sides             = exposed_sides;
  this->photoabsorption_coeff     = photoabsorption_coeff;
  this->diagnonal_absorption_prob = diagnonal_absorption_prob;
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

double Source::getEnergy() {
  return energy;
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

double Source::getPhotoAbsorptionCoeff(){
  return photoabsorption_coeff;
}

double Source::getDiagonalAbsorptionProb() {
  return diagnonal_absorption_prob;
}

void Source::setApparentActivities(std::vector<double> apparent_activities) {
  this->apparent_activities = apparent_activities;
}

#endif /* RADIATION_UTILS_SOURCE_H */
