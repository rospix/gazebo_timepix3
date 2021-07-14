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
#include <gazebo_timepix3/source_abstraction.h>

class ObstacleAbstraction {
private:
  unsigned int gazebo_id_;
  std::string  material_;

  std::vector<unsigned int> source_ids_;
  std::vector<double>       source_att_coeffs_;

  Eigen::Vector3d    relative_position_;
  Eigen::Quaterniond relative_orientation_;
  Eigen::Vector3d    size_;
  double             density_;

  mrs_lib::geometry::Cuboid relative_cuboid_;  // TODO update to allow different shapes of obstacles

public:
  ObstacleAbstraction();
  ~ObstacleAbstraction();
  ObstacleAbstraction(unsigned int gazebo_id, std::string material, Eigen::Vector3d relative_position, Eigen::Quaterniond relative_orientation,
                      Eigen::Vector3d size);

  bool operator==(const ObstacleAbstraction &o1);
  bool operator==(const unsigned int &i);

  // getters
  unsigned int              getId();
  std::string               getMaterial();
  double                    getDensity();
  Eigen::Vector3d           getSize();
  Eigen::Vector3d           getRelativePosition();
  Eigen::Quaterniond        getRelativeOrientation();
  mrs_lib::geometry::Cuboid getRelativeCuboid();

  void setRelativePosition(const Eigen::Vector3d &relative_position);
  void setRelativeOrientation(Eigen::Quaterniond relative_orientation);

  void addSource(const SourceAbstraction &sa);
  void removeSource(const SourceAbstraction &sa);
  void removeSource(unsigned int source_index);

  void updateRelativeCuboid();
};

#endif /* OBSTACLE_ABSTRACTION_H */
