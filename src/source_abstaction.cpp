#include <gazebo_timepix3/source_abstraction.h>

/* Destructor //{ */
SourceAbstraction::~SourceAbstraction() {
}
//}

/* Constructor //{ */
SourceAbstraction::SourceAbstraction() {
}

SourceAbstraction::SourceAbstraction(unsigned int gazebo_id, std::string material, double activity, double energy, double mass_att_coeff,
                                     double air_mass_att_coeff, Eigen::Vector3d relative_position) {

  gazebo_id_          = gazebo_id;
  material_           = material;
  activity_           = activity;
  energy_             = energy;
  mass_att_coeff_     = mass_att_coeff;
  air_mass_att_coeff_ = air_mass_att_coeff;
  relative_position_  = relative_position;
}
//}

/* operator == //{ */
bool SourceAbstraction::operator==(SourceAbstraction const &sa) {
  return gazebo_id_ == sa.gazebo_id_;
}

bool SourceAbstraction::operator==(const unsigned int &i) {
  return gazebo_id_ == i;
}

bool SourceAbstraction::operator==(unsigned int &i) {
  return gazebo_id_ == i;
}
//}

/* getId //{ */
const unsigned int SourceAbstraction::getId() const {
  return gazebo_id_;
}
//}

/* getMaterial //{ */
const std::string SourceAbstraction::getMaterial() const {
  return material_;
}
//}

/* getActivity //{ */
const double SourceAbstraction::getActivity() const {
  return activity_;
}
//}

/* getEnergy //{ */
const double SourceAbstraction::getEnergy() const {
  return energy_;
}
//}

/* getMassAttCoeff //{ */
const double SourceAbstraction::getMassAttCoeff() const {
  return mass_att_coeff_;
}
//}

/* getRelativePosition //{ */
const Eigen::Vector3d SourceAbstraction::getRelativePosition() const {
  return relative_position_;
}
//}

/* getSideProperties //{ */
const std::vector<SideProperty> SourceAbstraction::getSideProperties() const {
  return side_properties_;
}
//}

/* getObstaclesInPath //{ */
const std::vector<unsigned int> SourceAbstraction::getObstaclesInPath() const {
  return obstacles_in_path_;
}
//}

/* getAirMassAttCoeff //{ */
const double SourceAbstraction::getAirMassAttCoeff() const {
  return air_mass_att_coeff_;
}
//}

/* setSideProperties //{ */
void SourceAbstraction::setSideProperties(std::vector<SideProperty> side_properties) {
  side_properties_ = side_properties;
}
//}

/* setRelativePosition //{ */
void SourceAbstraction::setRelativePosition(Eigen::Vector3d relative_position) {
  relative_position_ = relative_position;
}
//}

/* addObstacle //{ */
void SourceAbstraction::addObstacle(unsigned int obstacle_id) {
  obstacles_in_path_.push_back(obstacle_id);
}
//}

/* removeObstacle //{ */
void SourceAbstraction::removeObstacle(unsigned int obstacle_id) {
  obstacles_in_path_.erase(std::remove(obstacles_in_path_.begin(), obstacles_in_path_.end(), obstacle_id), obstacles_in_path_.end());
}
//}
