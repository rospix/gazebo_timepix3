#ifndef OBSTACLE_ABSTRACTION_H
#define OBSTACLE_ABSTRACTION_H

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <ignition/math.hh>
#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <rad_utils/physics.h>
#include <gazebo_timepix/source_abstraction.h>

class ObstacleAbstraction {
private:
  unsigned int gazebo_id;
  std::string  material;

  std::vector<unsigned int> source_ids;
  std::vector<double>       source_att_coeffs;

  Eigen::Vector3d    relative_position;
  Eigen::Quaterniond relative_orientation;
  Eigen::Vector3d    size;
  double             density;

  mrs_lib::geometry::Cuboid relative_cuboid;  // TODO update to allow different shapes of obstacles

public:
  ObstacleAbstraction();
  ~ObstacleAbstraction();
  ObstacleAbstraction(unsigned int gazebo_id, std::string material, Eigen::Vector3d relative_position, Eigen::Quaterniond relative_orientation,
                      Eigen::Vector3d size);

  bool operator==(ObstacleAbstraction const &o1) {
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
    updateRelativeCuboid();
  }

  Eigen::Quaterniond getRelativeOrientation() {
    return relative_orientation;
  }

  void setRelativeOrientation(Eigen::Quaterniond relative_orientation) {
    this->relative_orientation = relative_orientation;
    updateRelativeCuboid();
  }

  void removeSource(SourceAbstraction source) {
    unsigned int index = -1;
    for (unsigned int i = 0; i < source_ids.size(); i++) {
      if (source_ids[i] == source.getId()) {
        index = i;
        break;
      }
    }
    if (index >= 0) {
      source_ids.erase(source_ids.begin() + index);
      source_att_coeffs.erase(source_att_coeffs.begin() + index);
    }
  }

  void removeSource(unsigned int source_index) {
    unsigned int index = -1;
    for (unsigned int i = 0; i < source_ids.size(); i++) {
      if (source_ids[i] == source_index) {
        index = i;
        break;
      }
    }
    if (index >= 0) {
      source_ids.erase(source_ids.begin() + index);
      source_att_coeffs.erase(source_att_coeffs.begin() + index);
    }
  }


  void addSource(SourceAbstraction source) {
    for (unsigned int i = 0; i < source_ids.size(); i++) {
      if (source_ids[i] == source.getId()) {
        return;
      }
    }
    source_ids.push_back(source.getId());
    source_att_coeffs.push_back(calculateMassAttCoeff(source.getEnergy(), source.getMaterial(), AttenuationType::MASS_ENERGY));
    ROS_INFO("Source%u linked to Obstacle%u", source.getId(), gazebo_id);
  }

  Eigen::Vector3d getSize() {
    return size;
  }

  mrs_lib::geometry::Cuboid getRelativeCuboid() {
    return relative_cuboid;
  }

  void updateRelativeCuboid() {
    relative_cuboid = mrs_lib::geometry::Cuboid(relative_position, size, relative_orientation);
  }

  double getDensity() {
    return density;
  }
};

ObstacleAbstraction::ObstacleAbstraction() {
}

ObstacleAbstraction::~ObstacleAbstraction() {
}

ObstacleAbstraction::ObstacleAbstraction(unsigned int gazebo_id, std::string material, Eigen::Vector3d relative_position,
                                         Eigen::Quaterniond relative_orientation, Eigen::Vector3d size) {

  this->gazebo_id            = gazebo_id;
  this->material             = material;
  this->relative_position    = relative_position;
  this->relative_orientation = relative_orientation;
  this->size                 = size;
  this->density              = getMaterialDensity(material);
}

#endif /* OBSTACLE_ABSTRACTION_H */
