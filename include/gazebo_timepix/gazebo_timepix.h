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
#include <mutex>

// ros and gazebo libraries
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <ros/package.h>
#include <sdf/sdf.hh>
#include <tf/transform_broadcaster.h>

// mrs libraries
#include <mrs_msgs/Float64Srv.h>
#include <mrs_lib/geometry_utils.h>
#include <mrs_lib/batch_visualizer.h>

// package libraries
#include <gazebo_timepix/source_abstraction.h>
#include <gazebo_timepix/obstacle_abstraction.h>

#include <radiation_utils/physics.h>

// ros and gazebo messages
#include <gazebo_rad_msgs/Timepix.h>
#include <gazebo_rad_msgs/DebugSetActivity.h>
#include <gazebo_rad_msgs/DebugSetMaterial.h>

#include <gazebo_rad_msgs/TimepixDiagnostics.h>
#include <gazebo_rad_msgs/SourceDiagnostics.h>

#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.pb.h>

//}

typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource>   RadiationSourceConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationObstacle> RadiationObstacleConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::Termination>       TerminationConstPtr;

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
  void          publisherLoop();
  ros::Time     simulate();
  void          buildSensorCuboid();
  void          publishDiagnostics();
  void          debugVisualize();
  void          publishSensorMsg(int particle_count);

  std::vector<Triplet> calculateSideProperties(SourceAbstraction s);

  double exposition_seconds = 1.0;
  double density;
  double diagonal_length;

  double air_density;

  std::mutex sources_mutex;
  std::mutex obstacles_mutex;

  std::vector<SourceAbstraction>   sources;
  std::vector<ObstacleAbstraction> obstacles;
  std::vector<mrs_lib::Rectangle>  sides;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;

  transport::SubscriberPtr sources_sub_, obstacles_sub_, termination_sub_;
  tf::TransformBroadcaster transform_broadcaster;

  event::ConnectionPtr updateConnection_;

  void sourcesCallback(RadiationSourceConstPtr &msg);
  void obstaclesCallback(RadiationObstacleConstPtr &msg);
  void terminationCallback(TerminationConstPtr &msg);
  void onWorldUpdate(const common::UpdateInfo &upd);

  // param control
  bool setExpositionCallback(mrs_msgs::Float64SrvRequest &req, mrs_msgs::Float64SrvResponse &res);

  double                    traceEnvironmentAbsorption(SourceAbstraction sa);
  std::vector<unsigned int> traceObstaclesId(SourceAbstraction sa);

  ros::NodeHandle    ros_node;
  ros::Publisher     ros_publisher, diagnostics_publisher;
  ros::ServiceServer set_exposition_server;

  BatchVisualizer debug_visualizer;
  BatchVisualizer bv;

  Eigen::Vector3d sampleRectangle(mrs_lib::Rectangle r);

  // RNG stuff
  std::mt19937                           rand_gen;
  std::uniform_real_distribution<double> rand_dbl;
};

GZ_REGISTER_MODEL_PLUGIN(Timepix)
Timepix::Timepix() : ModelPlugin() {
}
}  // namespace gazebo

/* targetRelativePosition //{ */
Eigen::Vector3d targetRelativePosition(ignition::math::Pose3d my_pose, Eigen::Vector3d target_pos) {

  ignition::math::Vector3d    p = my_pose.Pos();
  ignition::math::Quaterniond q = my_pose.Rot();

  Eigen::Vector3d    my_world_pos(p.X(), p.Y(), p.Z());
  Eigen::Quaterniond local2world = Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z());
  Eigen::Quaterniond world2local = local2world.inverse();

  return world2local * (target_pos - my_world_pos);
}
//}

/* targetRelativePose //{ */
ignition::math::Pose3d targetRelativePose(ignition::math::Pose3d my_pose, ignition::math::Pose3d target_pose) {
  return target_pose - my_pose;
}
//}

#endif /* GAZEBO_RAD_SOURCE_H */
