#ifndef GAZEBO_TIMEPIX_H
#define GAZEBO_TIMEPIX_H

/* includes //{ */

// std libraries
#include <algorithm>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Core>

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

#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>

//}


typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource> RadiationSourceConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::Termination>     TerminationConstPtr;

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
  std::string              material = "Si";
  std::stringstream        global_frame;
  std::stringstream        local_frame;

  bool          terminated;
  boost::thread publisher_thread;
  void          PublisherLoop();
  void          Simulate();
  void          buildSensorCuboid();

  std::set<Triplet>          calculateSideProperties(Source s);
  double                        exposition_seconds = 1;
  std::chrono::duration<double> exposition_duration;

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
  ros::Publisher                   ros_publisher;

  BatchVisualizer bv;

  Eigen::Vector3d sampleRectangle(Rectangle r);

  // RNG stuff
  std::mt19937                           rand_gen;
  std::uniform_real_distribution<double> rand_dbl;
};

GZ_REGISTER_MODEL_PLUGIN(Timepix)
Timepix::Timepix() : ModelPlugin() {
}
}  // namespace gazebo

#endif /* GAZEBO_RAD_SOURCE_H */
