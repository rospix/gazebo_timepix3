// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_timepix/data_utils.h>
#include <radiation_utils/geometry.h>

// std includes
#include <random>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>
#include <fstream>

#include <boost/thread.hpp>

#include <eigen3/Eigen/Core>

#include <gazebo_rad_msgs/Timepix.h>
#include <gazebo_rad_msgs/TimepixDiagnostics.h>

enum AttenuationType
{
  PHOTOELECTRIC = 1,
  ALL           = 2,
};

/* class definition //{ */
class Timepix {
public:
  Timepix();
  ~Timepix();
  Timepix(Eigen::Vector3d size, std::string material);

  static void calculateMassAttCoeff(double photon_energy, double mac, double density);

private:
  // initialization variables
  Eigen::Vector3d size;
  std::string     material;
  std::string     global_frame;
  std::string     local_frame;

  // TODO change this into cuboid
  Rectangle rect;

  // values from nist tables
  double density;
  double photoelectric_mac;
  double all_mac;


  bool          terminated = false;
  void          SimulateRoutine();
  void          PublishRoutine();
  boost::thread simulate_thread;
  boost::thread publish_thread;

  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher                   sensor_publisher, diagnostics_publisher;

  // random number generation
  std::mt19937                           rand_gen;
  std::uniform_real_distribution<double> rand_dbl;
};
//}

void Timepix::calculateMassAttCoeff(double photon_energy, double mac, double density) {
  Table table = loadNistTable("si");
}

