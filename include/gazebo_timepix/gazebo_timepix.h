#ifndef GAZEBO_TIMEPIX_H
#define GAZEBO_TIMEPIX_H

#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_rad_msgs/Timepix.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.pb.h>
#include <gazebo_rad_msgs/Termination.pb.h>

#include <gazebo_timepix/source.h>
#include <gazebo_timepix/obstacle.h>

typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource>   RadiationSourceConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationObstacle> RadiationObstacleConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::Termination>       TerminationConstPtr;

namespace gazebo
{
class GAZEBO_VISIBLE Timepix : public ModelPlugin {
public:
  Timepix();
  virtual ~Timepix();
  void QueueThread();
  void SimulationThread();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void LateUpdate();

private:
  bool          terminated;
  boost::thread simulation_thread;

  physics::ModelPtr    model_;
  physics::WorldPtr    world_;
  event::ConnectionPtr updateConnection_;

  std::vector<Source>   sources;
  std::vector<Obstacle> obstacles;
  std::mutex            sources_mutex;
  std::mutex            obstacles_mutex;

  double             exposition_time;
  Eigen::Vector3d    world_pos;
  Eigen::Quaterniond world_ori;

  transport::NodePtr       transport_node_;
  transport::SubscriberPtr sources_sub;
  transport::SubscriberPtr obstacles_sub;
  transport::SubscriberPtr termination_sub;

  std::unique_ptr<ros::NodeHandle> ros_node;
  ros::Publisher                   timepix_pub;

  void sourcesCallback(RadiationSourceConstPtr &msg);
  void obstaclesCallback(RadiationObstacleConstPtr &msg);
  void terminationCallback(TerminationConstPtr &msg);
};
GZ_REGISTER_MODEL_PLUGIN(Timepix)
Timepix::Timepix() : ModelPlugin() {
}
}  // namespace gazebo

#endif /* GAZEBO_TIMEPIX_H */
