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
unsigned int SourceAbstraction::getId() {
  return gazebo_id_;
}
//}

/* getMaterial //{ */
std::string SourceAbstraction::getMaterial() {
  return material_;
}
//}

/* getActivity //{ */
double SourceAbstraction::getActivity() {
  return activity_;
}
//}

/* getEnergy //{ */
double SourceAbstraction::getEnergy() {
  return energy_;
}
//}

/* getMassAttCoeff //{ */
double SourceAbstraction::getMassAttCoeff() {
  return mass_att_coeff_;
}
//}


/* getRelativePosition //{ */
Eigen::Vector3d SourceAbstraction::getRelativePosition() {
  return relative_position_;
}
//}

/* getSideProperties //{ */
std::vector<SideProperty> SourceAbstraction::getSideProperties() {
  return side_properties_;
}
//}

/* getObstaclesInPath //{ */
std::vector<unsigned int> SourceAbstraction::getObstaclesInPath() {
  return obstacles_in_path_;
}
//}

/* getAirMassAttCoeff //{ */
double SourceAbstraction::getAirMassAttCoeff() {
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
