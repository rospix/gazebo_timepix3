#ifndef GAZEBO_TIMEPIX_H
#define GAZEBO_TIMEPIX_H

#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo_rad_msgs/Timepix.h>
#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.pb.h>

#include <gazebo_timepix/source.h>
#include <gazebo_timepix/obstacle.h>
#include <gazebo_timepix/geometry_utils.h>
#include <gazebo_timepix/visual_utils.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

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

private:
  std::uniform_real_distribution<double> rand_dbl;
  std::mt19937                           rand_gen;

  bool          terminated;
  boost::thread simulation_thread;
  int           simulate();

  physics::ModelPtr model_;
  physics::WorldPtr world_;

  std::vector<Source>   sources;
  std::vector<Obstacle> obstacles;
  std::mutex            sources_mutex;
  std::mutex            obstacles_mutex;
  std::mutex            transform_mutex;

  double                 sensor_size;
  double                 sensor_thickness;
  double                 diagonal_length;
  double                 exposition_time;
  std::vector<Rectangle> sides;
  Cuboid                 sensor_cuboid;

  std::string              material;
  std::stringstream        frame_name;
  transport::NodePtr       transport_node_;
  transport::SubscriberPtr sources_sub;
  transport::SubscriberPtr obstacles_sub;
  transport::SubscriberPtr termination_sub;
  tf::TransformBroadcaster transform_broadcaster;

  std::unique_ptr<ros::NodeHandle> ros_node;
  ros::Publisher                   timepix_pub;

  void sourcesCallback(RadiationSourceConstPtr &msg);
  void obstaclesCallback(RadiationObstacleConstPtr &msg);
  void terminationCallback(TerminationConstPtr &msg);

  Eigen::Vector3d sampleRectangle(Rectangle r);
  Eigen::Vector3d sampleSide(int index);

  BatchVisualizer bv;
};
GZ_REGISTER_MODEL_PLUGIN(Timepix)
Timepix::Timepix() : ModelPlugin() {
}
}  // namespace gazebo

#endif /* GAZEBO_TIMEPIX_H */
