#include <radiation_utils.h>

Source::Source() {
}
Source::~Source() {
}
Source::Source(std::string material, double activity, Eigen::Vector3d position) {
  this->material = material;
  this->activity = activity;
  this->position = position;
}
