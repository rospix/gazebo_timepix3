#ifndef GAZEBO_TIMEPIX_H
#define GAZEBO_TIMEPIX_H

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_rad_msgs/Timepix.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.pb.h>

typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource>   RadiationSourceConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationObstacle> RadiationObstacleConstPtr;

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
  virtual void OnUpdate(const common::UpdateInfo &);

private:
  physics::ModelPtr                model_;
  physics::WorldPtr                world_;
  event::ConnectionPtr             updateConnection_;
  std::unique_ptr<ros::NodeHandle> rosNode;

  ros::Publisher timepix_pub;
};
GZ_REGISTER_MODEL_PLUGIN(Timepix)
Timepix::Timepix() : ModelPlugin() {
}
}  // namespace gazebo
#endif /* GAZEBO_TIMEPIX_H */
