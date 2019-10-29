#ifndef GAZEBO_TIMEPIX_H
#define GAZEBO_TIMEPIX_H

/* includes //{ */

// std libraries
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <eigen3/Eigen/Core>
#include <fstream>

// ros and gazebo libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <ros/package.h>
#include <sdf/sdf.hh>
#include <tf/transform_broadcaster.h>

// package libraries
#include <gazebo_timepix/source.h>
#include <gazebo_timepix/geometry_utils.h>
#include <gazebo_timepix/visual_utils.h>

// ros and gazebo messages
#include <gazebo_rad_msgs/Timepix.h>
#include <gazebo_rad_msgs/DebugSetActivity.h>
#include <gazebo_rad_msgs/DebugSetMaterial.h>

#include <gazebo_rad_msgs/TimepixDiagnostics.h>
#include <gazebo_rad_msgs/SourceDiagnostics.h>

#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>

//}

typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource> RadiationSourceConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::Termination>     TerminationConstPtr;

enum AttenuationType
{
  PHOTOELECTRIC = 1,
  ALL           = 2
};

namespace gazebo
{
class GAZEBO_VISIBLE Timepix : public ModelPlugin {
public:
  Timepix();
  virtual ~Timepix();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  ignition::math::Vector3d size;
  std::string              material = "si";
  std::stringstream        global_frame;
  std::stringstream        local_frame;

  bool          terminated;
  boost::thread publisher_thread;
  void          PublisherLoop();
  ros::Time     Simulate(ros::Time sim_start);
  void          buildSensorCuboid();
  void          publishDiagnostics();
  void          publishSensorMsg(int particle_count);

  std::set<Triplet> calculateSideProperties(Source s);
  double            getDensity(std::string material);
  double            calculateMassAttCoeff(double photon_energy, AttenuationType a);
  double            photoabsorptionProbability(double material_thickness, double mass_att_coeff, double mat_density);

  double        exposition_seconds = 1.0;
  ros::Duration exposition_duration;
  double        density;
  double        diagonal_length;


  std::vector<Source>    sources;
  std::vector<Rectangle> sides;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;

  transport::SubscriberPtr sources_sub;
  transport::SubscriberPtr termination_sub;
  tf::TransformBroadcaster transform_broadcaster;

  event::ConnectionPtr updateConnection_;

  void sourcesCallback(RadiationSourceConstPtr &msg);
  void terminationCallback(TerminationConstPtr &msg);
  void OnWorldUpdate(const common::UpdateInfo &upd);

  std::unique_ptr<ros::NodeHandle> ros_node;
  ros::Publisher                   ros_publisher, diagnostics_publisher;

  BatchVisualizer bv;

  Eigen::Vector3d sampleRectangle(Rectangle r);

  std::vector<std::vector<std::string>> loadNistTable(std::string material);

  // RNG stuff
  std::mt19937                           rand_gen;
  std::uniform_real_distribution<double> rand_dbl;
};

GZ_REGISTER_MODEL_PLUGIN(Timepix)
Timepix::Timepix() : ModelPlugin() {
}
}  // namespace gazebo

#endif /* GAZEBO_RAD_SOURCE_H */
