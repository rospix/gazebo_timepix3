#include <gazebo_timepix3/obstacle_abstraction.h>

/* Destructor //{ */
ObstacleAbstraction::~ObstacleAbstraction() {
}
//}

/* Constructor //{ */
ObstacleAbstraction::ObstacleAbstraction() {
}

ObstacleAbstraction::ObstacleAbstraction(unsigned int gazebo_id, std::string material, Eigen::Vector3d relative_position,
                                         Eigen::Quaterniond relative_orientation, Eigen::Vector3d size) {
  gazebo_id_            = gazebo_id;
  material_             = material;
  relative_position_    = relative_position;
  relative_orientation_ = relative_orientation;
  size_                 = size;
  density_              = getMaterialDensity(material);
}
//}

/* operator == //{ */
bool ObstacleAbstraction::operator==(const ObstacleAbstraction &o1) {
  return this->gazebo_id_ == o1.gazebo_id_;
}

bool ObstacleAbstraction::operator==(const unsigned int &i) {
  return this->gazebo_id_ == i;
}
//}

/* getId //{ */
unsigned int ObstacleAbstraction::getId() {
  return gazebo_id_;
}
//}

/* getMaterial //{ */
std::string ObstacleAbstraction::getMaterial() {
  return material_;
}
//}

/* getDensity //{ */
double ObstacleAbstraction::getDensity() {
  return density_;
}
//}

/* getSize //{ */
Eigen::Vector3d ObstacleAbstraction::getSize() {
  return size_;
}
//}

/* getRelativePosition //{ */
Eigen::Vector3d ObstacleAbstraction::getRelativePosition() {
  return relative_position_;
}
//}

/* getRelativeOrientation //{ */
Eigen::Quaterniond ObstacleAbstraction::getRelativeOrientation() {
  return relative_orientation_;
}
//}

/* getRelativeCuboid //{ */
mrs_lib::geometry::Cuboid ObstacleAbstraction::getRelativeCuboid() {
  return relative_cuboid_;
}
//}

/* setRelativePosition //{ */
void ObstacleAbstraction::setRelativePosition(Eigen::Vector3d &relative_position) {
  relative_position_ = relative_position;
  updateRelativeCuboid();
}
//}

/* setRelativeOrientation //{ */
void ObstacleAbstraction::setRelativeOrientation(Eigen::Quaterniond &relative_orientation) {
  relative_orientation_ = relative_orientation;
  updateRelativeCuboid();
}
//}

/* addSource //{ */
void ObstacleAbstraction::addSource(SourceAbstraction &sa) {
  for (unsigned int i = 0; i < source_ids_.size(); i++) {
    if (source_ids_[i] == sa.getId()) {
      return;
    }
  }
  source_ids_.push_back(sa.getId());
  source_att_coeffs_.push_back(calculateMassAttCoeff(sa.getEnergy(), sa.getMaterial(), AttenuationType::MASS_ENERGY));
  ROS_INFO("Source%u linked to Obstacle%u", sa.getId(), gazebo_id_);
}
//}

/* removeSource //{ */
void ObstacleAbstraction::removeSource(SourceAbstraction &sa) {
  unsigned int index = -1;
  for (unsigned int i = 0; i < source_ids_.size(); i++) {
    if (source_ids_[i] == sa.getId()) {
      index = i;
      break;
    }
  }
  if (index >= 0) {
    source_ids_.erase(source_ids_.begin() + index);
    source_att_coeffs_.erase(source_att_coeffs_.begin() + index);
  }
}
//}

/* removeSource //{ */
void ObstacleAbstraction::removeSource(const unsigned int source_index) {
  unsigned int index = -1;
  for (unsigned int i = 0; i < source_ids_.size(); i++) {
    if (source_ids_[i] == source_index) {
      index = i;
      break;
    }
  }
  if (index >= 0) {
    source_ids_.erase(source_ids_.begin() + index);
    source_att_coeffs_.erase(source_att_coeffs_.begin() + index);
  }
}
//}

/* updateRelativeCuboid //{ */
void ObstacleAbstraction::updateRelativeCuboid() {
  relative_cuboid_ = mrs_lib::geometry::Cuboid(relative_position_, size_, relative_orientation_);
}
//}

