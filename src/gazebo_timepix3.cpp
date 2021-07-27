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
#include <mrs_lib/geometry/shapes.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/batch_visualizer.h>

// package libraries
#include <gazebo_timepix3/source_abstraction.h>
#include <gazebo_timepix3/obstacle_abstraction.h>

#include <rad_utils/physics.h>

// ros and gazebo messages
#include <gazebo_rad_msgs/DebugSetActivity.h>
#include <gazebo_rad_msgs/DebugSetMaterial.h>

#include <gazebo_rad_msgs/Timepix3Diagnostics.h>
#include <gazebo_rad_msgs/SourceDiagnostics.h>

#include <gazebo_rad_msgs/Termination.pb.h>
#include <gazebo_rad_msgs/RadiationSource.pb.h>
#include <gazebo_rad_msgs/RadiationObstacle.pb.h>

#include <rad_msgs/ClusterList.h>

//}

typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource>   RadiationSourceConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationObstacle> RadiationObstacleConstPtr;
typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::Termination>       TerminationConstPtr;

namespace gazebo
{

/* class Timepix3 //{ */
class GAZEBO_VISIBLE Timepix3 : public ModelPlugin {
public:
  Timepix3();
  virtual ~Timepix3();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:
  ignition::math::Vector3d size_;
  std::string              material_      = "si";
  std::string              sensor_suffix_ = "";
  std::stringstream        global_frame_;
  std::stringstream        local_frame_;

  double density_;
  double air_density_;
  double max_message_window_ = 0.5;

  unsigned long sequence_num_ = 0;

  std::string package_path_;

  bool          terminated_;
  boost::thread publisher_thread_;

  std::mutex sources_mutex_;
  std::mutex obstacles_mutex_;
  std::mutex model_mutex_;

  std::vector<SourceAbstraction>            sources_;
  std::vector<ObstacleAbstraction>          obstacles_;
  std::vector<mrs_lib::geometry::Rectangle> sides_;

  physics::ModelPtr       model_;
  transport::NodePtr      gazebo_node_;
  transport::PublisherPtr gazebo_publisher_;

  transport::SubscriberPtr sources_sub_, obstacles_sub_, termination_sub_;
  tf::TransformBroadcaster transform_broadcaster_;
  ros::Time                last_tf_time_;

  event::ConnectionPtr updateConnection_;

  void publisherLoop();
  void buildSensorCuboid();
  void publishDiagnostics();
  void debugVisualize();
  void publishSensorMsg(const Eigen::Vector2i &pixel_coord, const double photon_energy);
  void publishEmptyMsg();

  ros::Time simulate();

  std::vector<SideProperty> calculateSideProperties(SourceAbstraction &sa);

  void sourcesCallback(RadiationSourceConstPtr &msg);
  void obstaclesCallback(RadiationObstacleConstPtr &msg);
  void terminationCallback(TerminationConstPtr &msg);
  void onWorldLateUpdate();

  // param control
  double                    traceEnvironmentTransmission(SourceAbstraction &sa);
  std::vector<unsigned int> traceObstaclesId(SourceAbstraction &sa);

  ros::NodeHandle ros_node_;
  ros::Publisher  ros_publisher_, diagnostics_publisher_;

  mrs_lib::BatchVisualizer debug_visualizer_;
  mrs_lib::BatchVisualizer bv_;

  std::pair<Eigen::Vector3d, Eigen::Vector2i> sampleRectangle(mrs_lib::geometry::Rectangle &r);

  // RNG stuff
  std::mt19937                           rand_gen_;
  std::uniform_real_distribution<double> rand_dbl_;
};

GZ_REGISTER_MODEL_PLUGIN(Timepix3) Timepix3::Timepix3() : ModelPlugin() {
}
//}

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

// index sides of the detector for convenience
#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define BOTTOM 4
#define TOP 5

#define ORANGE 1.0, 0.7, 0.2, 1.0
#define GREEN 0.3, 1.0, 0.3, 1.0
#define BLUE 0.2, 0.2, 1.0, 1.0
#define BLACK 0.0, 0.0, 0.0, 1.0
#define GRAY 0.7, 0.7, 0.7, 1.0
#define BROWN 0.3, 0.2, 0.0, 1.0

using namespace gazebo;

/* Destructor //{ */
Timepix3::~Timepix3() {

  // shutdown subscribers
  sources_sub_->Unsubscribe();
  obstacles_sub_->Unsubscribe();
  termination_sub_->Unsubscribe();

  // inform other gazebo nodes
  gazebo_rad_msgs::msgs::Termination msg;
  msg.set_id(model_->GetId());


  // terminate
  terminated_ = true;
  publisher_thread_.join();
  ROS_INFO("[Timepix3 #%u]: Plugin terminated", model_->GetId());
}
//}

/* Load //{ */
void Timepix3::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  model_ = _model;

  /* parse sdf params //{ */
  if (_sdf->HasElement("material")) {
    material_ = _sdf->Get<std::string>("material");
  } else {
    std::cout << "[Timepix3 #" << model_->GetId() << "]: parameter 'material' was not specified" << std::endl;
    std::cout << "[Timepix3 #" << model_->GetId() << "]: using default value: '" << material_ << "'" << std::endl;
  }
  if (_sdf->HasElement("size")) {
    size_ = _sdf->Get<ignition::math::Vector3d>("size");
  } else {
    std::cout << "[Timepix3 #" << model_->GetId() << "]: parameter 'size' was not specified" << std::endl;
    return;
  }
  if (_sdf->HasElement("max_message_window")) {
    max_message_window_ = _sdf->Get<double>("max_message_window");
  } else {
    std::cout << "[Timepix3 #" << model_->GetId() << "]: parameter 'max_message_window' was not specified" << std::endl;
    std::cout << "[Timepix3 #" << model_->GetId() << "]: using default value: '" << max_message_window_ << "'" << std::endl;
  }
  if (_sdf->HasElement("sensor_suffix")) {
    sensor_suffix_ = _sdf->Get<double>("sensor_suffix");
  } else {
    std::cout << "[Timepix3 #" << model_->GetId() << "]: parameter 'sensor_suffix' not used" << std::endl;
  }
  //}

  // init local variables
  buildSensorCuboid();
  local_frame_ << model_->GetName().c_str() << "/timepix_origin";
  global_frame_ << model_->GetName().c_str() << "/gps_origin";
  density_     = getMaterialDensity(material_);
  air_density_ = getMaterialDensity("air");

  package_path_ = ros::package::getPath("gazebo_timepix3");

  rand_dbl_     = std::uniform_real_distribution<double>(0, 1);
  last_tf_time_ = ros::Time::now();

  // init gazebo node
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_timepix3", ros::init_options::NoSigintHandler);
  ros_node_ = ros::NodeHandle("~");

  updateConnection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&Timepix3::onWorldLateUpdate, this));

  // gazebo communication
  sources_sub_     = gazebo_node_->Subscribe("/radiation/sources", &Timepix3::sourcesCallback, this, 1);
  obstacles_sub_   = gazebo_node_->Subscribe("/radiation/obstacles", &Timepix3::obstaclesCallback, this, 1);
  termination_sub_ = gazebo_node_->Subscribe("/radiation/termination", &Timepix3::terminationCallback, this, 1);

  // ros communication
  std::stringstream ss;
  ss << "/" << model_->GetName() << "/minipix" << sensor_suffix_.c_str() << "/cluster_list";
  ros_publisher_ = ros_node_.advertise<rad_msgs::ClusterList>(ss.str().c_str(), 1);
  ss.str(std::string());
  ss << "/" << model_->GetName() << "/minipix" << sensor_suffix_.c_str() << "/plugin_diagnostics";
  diagnostics_publisher_ = ros_node_.advertise<gazebo_rad_msgs::Timepix3Diagnostics>(ss.str().c_str(), 1);

  debug_visualizer_ = mrs_lib::BatchVisualizer(ros_node_, "debug_visualizer", local_frame_.str());
  debug_visualizer_.setPointsScale(0.3);

  terminated_       = false;
  publisher_thread_ = boost::thread(boost::bind(&Timepix3::publisherLoop, this));
  ROS_INFO("[Timepix3 #%u]: Plugin initialized", model_->GetId());
}
//}

/* sourcesCallback //{ */
void Timepix3::sourcesCallback(RadiationSourceConstPtr &msg) {

  std::scoped_lock lock(sources_mutex_);

  /* Check whether the source is already registered //{ */
  for (auto source = sources_.begin(); source != sources_.end(); source++) {

    if (source->getId() == msg->id()) {
      Eigen::Vector3d source_world_pos(msg->x(), msg->y(), msg->z());
      source->setRelativePosition(targetRelativePosition(model_->WorldPose(), source_world_pos));
      source->setSideProperties(calculateSideProperties(*source));
      return;
    }
  }
  //}

  /* Handle newly registered source //{ */
  ROS_INFO("[Timepix3 #%u]: Newly registered RadiationSource%u", model_->GetId(), msg->id());
  Eigen::Vector3d source_world_pos(msg->x(), msg->y(), msg->z());

  double            mass_att_coeff     = calculateMassAttCoeff(msg->energy(), material_, AttenuationType::MASS_ENERGY);
  double            air_mass_att_coeff = calculateMassAttCoeff(msg->energy(), "air", AttenuationType::MASS_ENERGY);
  SourceAbstraction s(msg->id(), msg->material(), msg->activity(), msg->energy(), mass_att_coeff, air_mass_att_coeff,
                      targetRelativePosition(model_->WorldPose(), source_world_pos));
  s.setSideProperties(calculateSideProperties(s));
  sources_.push_back(s);
  //}
}
//}

/* obstaclesCallback //{ */
void Timepix3::obstaclesCallback(RadiationObstacleConstPtr &msg) {

  std::scoped_lock lock(obstacles_mutex_);

  /* Check whether the obstacle is already registered //{ */
  for (auto obstacle = obstacles_.begin(); obstacle != obstacles_.end(); obstacle++) {
    if (obstacle->getId() == msg->id()) {
      ignition::math::Pose3d obstacle_pose;
      obstacle_pose.Pos().X() = msg->pos_x();
      obstacle_pose.Pos().Y() = msg->pos_y();
      obstacle_pose.Pos().Z() = msg->pos_z();
      obstacle_pose.Rot().W() = msg->ori_w();
      obstacle_pose.Rot().X() = msg->ori_x();
      obstacle_pose.Rot().Y() = msg->ori_y();
      obstacle_pose.Rot().Z() = msg->ori_z();

      ignition::math::Pose3d relative_pose = obstacle_pose - model_->WorldPose();
      Eigen::Vector3d        relative_pos(relative_pose.Pos().X(), relative_pose.Pos().Y(), relative_pose.Pos().Z());
      Eigen::Quaterniond     relative_rot(relative_pose.Rot().W(), relative_pose.Rot().X(), relative_pose.Rot().Y(), relative_pose.Rot().Z());
      obstacle->setRelativePosition(relative_pos);
      obstacle->setRelativeOrientation(relative_rot);
      return;
    }
  }
  //}

  /* Handle newly registered obstacle //{ */
  ROS_INFO("[Timepix3 #%u]: Newly registered RadiationObstacle%u", model_->GetId(), msg->id());

  ignition::math::Pose3d timepix_world_pose = model_->WorldPose();
  ignition::math::Pose3d obstacle_world_pose;

  obstacle_world_pose.Pos().X() = msg->pos_x();
  obstacle_world_pose.Pos().Y() = msg->pos_y();
  obstacle_world_pose.Pos().Z() = msg->pos_z();
  obstacle_world_pose.Rot().X() = msg->ori_x();
  obstacle_world_pose.Rot().Y() = msg->ori_y();
  obstacle_world_pose.Rot().Z() = msg->ori_z();
  obstacle_world_pose.Rot().W() = msg->ori_w();

  ignition::math::Pose3d relative_pose = obstacle_world_pose - timepix_world_pose;

  Eigen::Vector3d    obstacle_size(msg->size_x(), msg->size_y(), msg->size_z());
  Eigen::Vector3d    relative_pos(relative_pose.Pos().X(), relative_pose.Pos().Y(), relative_pose.Pos().Z());
  Eigen::Quaterniond relative_ori(relative_pose.Rot().W(), relative_pose.Rot().X(), relative_pose.Rot().Y(), relative_pose.Rot().Z());

  ObstacleAbstraction o(msg->id(), msg->material(), relative_pos, relative_ori, obstacle_size);
  obstacles_.push_back(o);
  //}
}
//}

/* terminateCallback //{ */
void Timepix3::terminationCallback(TerminationConstPtr &msg) {

  /* terminate source //{ */
  std::scoped_lock lock_sources(sources_mutex_);
  unsigned int     sources_count = sources_.size();
  sources_.erase(std::remove(sources_.begin(), sources_.end(), msg->id()), sources_.end());
  if (sources_.size() != sources_count) {
    ROS_INFO("[Timepix3 #%u]: No longer tracking RadiationSource%u", model_->GetId(), msg->id());
    for (auto it = obstacles_.begin(); it != obstacles_.end(); it++) {
      it->removeSource(msg->id());
    }
    return;
  }
  //}

  /* terminate obstacle //{ */
  std::scoped_lock lock_obstacles(obstacles_mutex_);
  unsigned int     obstacles_count = obstacles_.size();
  obstacles_.erase(std::remove(obstacles_.begin(), obstacles_.end(), msg->id()), obstacles_.end());
  if (obstacles_count != obstacles_.size()) {
    ROS_INFO("[Timepix3 #%u]: No longer tracking RadiationObstacle%u", model_->GetId(), msg->id());
    return;
  }
  //}
}
//}

/* simulate //{ */
ros::Time Timepix3::simulate() {
  int photons_captured = 0;
  int rays_cast        = 0;

  std::scoped_lock lock(sources_mutex_);

  //{
  for (auto source = sources_.begin(); source != sources_.end(); source++) {
    // get num of photons to be simulated
    // trace obstacles
    Eigen::Vector3d        source_pos               = source->getRelativePosition();
    mrs_lib::geometry::Ray r                        = mrs_lib::geometry::Ray::twopointCast(Eigen::Vector3d::Zero(), source_pos);
    double                 environment_transmission = traceEnvironmentTransmission(*source);

    /* std::cout << "Environment transmission: " << environment_transmission << std::endl; */

    std::vector<SideProperty> side_properties = source->getSideProperties();
    for (auto side = side_properties.begin(); side != side_properties.end(); side++) {
      int num_photons = (int)(side->second * max_message_window_ * environment_transmission);

      // generate N photons
      for (int i = 0; i < num_photons; i++) {

        std::pair<Eigen::Vector3d, Eigen::Vector2i> sample      = sampleRectangle(sides_[side->first]);
        Eigen::Vector3d                             intersect1  = sample.first;
        Eigen::Vector2i                             pixel_coord = sample.second;

        mrs_lib::geometry::Ray r = mrs_lib::geometry::Ray::twopointCast(source->getRelativePosition(), intersect1);
        rays_cast++;
        // for each photon check the collision with other sides of the sensor
        for (int j = 0; j < 6; j++) {
          if (j == side->second) {
            continue;
          }
          auto intersect2 = sides_[j].intersectionRay(r);
          if (intersect2 != boost::none) {
            // ray hit, now calculate detection probability

            double track_length = (*intersect2 - intersect1).norm();
            double pe_prob      = calculateAbsorptionProb(track_length, source->getMassAttCoeff(), density_);
            double coin_flip    = rand_dbl_(rand_gen_);
            if (coin_flip < pe_prob) {
              photons_captured++;
              publishSensorMsg(pixel_coord, source->getEnergy());
            }
          }
        }
      }
    }
  }
  //}
  if (photons_captured < 1) {
    publishEmptyMsg();
  }

  return ros::Time::now();
}
//}

/* publisherLoop //{ */
void Timepix3::publisherLoop() {
  while (!terminated_) {
    auto sim_start = ros::Time::now();
    /* publishDiagnostics(); */
    auto sim_end = simulate();
    /* auto sim_end      = ros::Time::now(); */
    auto sim_duration = sim_end - sim_start;
    (ros::Duration(max_message_window_) - sim_duration).sleep();
  }
}
//}

/* calculateSideProperties //{ */
std::vector<SideProperty> Timepix3::calculateSideProperties(SourceAbstraction &sa) {
  std::vector<SideProperty> ret;
  for (int idx = 0; idx < 6; idx++) {
    Eigen::Vector3d side_normal = (sides_[idx].b() - sides_[idx].a()).cross(sides_[idx].d() - sides_[idx].a());
    if (side_normal.dot(sa.getRelativePosition()) > 0) {
      double       solid_angle       = sides_[idx].solidAngleRelativeTo(sa.getRelativePosition());
      double       apparent_activity = (sa.getActivity() / (4 * M_PI)) * solid_angle;
      SideProperty sp;
      sp.first  = idx;
      sp.second = apparent_activity;
      ret.push_back(sp);
      if (ret.size() > 2) {
        break;
      }
    }
  }
  return ret;
}
//}

/* buildSensorCuboid //{ */
void Timepix3::buildSensorCuboid() {
  for (int i = 0; i < 6; i++) {
    sides_.push_back(mrs_lib::geometry::Rectangle());
  }
  Eigen::Vector3d A(size_[0] / 2.0, -size_[1] / 2.0, -size_[2] / 2.0);
  Eigen::Vector3d B(size_[0] / 2.0, size_[1] / 2.0, -size_[2] / 2.0);
  Eigen::Vector3d C(size_[0] / 2.0, size_[1] / 2.0, size_[2] / 2.0);
  Eigen::Vector3d D(size_[0] / 2.0, -size_[1] / 2.0, size_[2] / 2.0);

  Eigen::Vector3d E(-size_[0] / 2.0, size_[1] / 2.0, -size_[2] / 2.0);
  Eigen::Vector3d F(-size_[0] / 2.0, -size_[1] / 2.0, -size_[2] / 2.0);
  Eigen::Vector3d G(-size_[0] / 2.0, -size_[1] / 2.0, size_[2] / 2.0);
  Eigen::Vector3d H(-size_[0] / 2.0, size_[1] / 2.0, size_[2] / 2.0);

  sides_[FRONT]  = mrs_lib::geometry::Rectangle(A, B, C, D);
  sides_[BACK]   = mrs_lib::geometry::Rectangle(E, F, G, H);
  sides_[LEFT]   = mrs_lib::geometry::Rectangle(B, E, H, C);
  sides_[RIGHT]  = mrs_lib::geometry::Rectangle(F, A, D, G);
  sides_[BOTTOM] = mrs_lib::geometry::Rectangle(F, E, B, A);
  sides_[TOP]    = mrs_lib::geometry::Rectangle(D, C, H, G);
}
//}

/* traceEnvironmentTransmission//{ */
double Timepix3::traceEnvironmentTransmission(SourceAbstraction &sa) {
  Eigen::Vector3d source_position = sa.getRelativePosition();

  mrs_lib::geometry::Ray r = mrs_lib::geometry::Ray::twopointCast(Eigen::Vector3d::Zero(), source_position);

  double transmission              = 1.0;
  double cumulative_obstacle_track = 0.0;
  for (auto o = obstacles_.begin(); o != obstacles_.end(); o++) {
    std::vector<Eigen::Vector3d> intersections = o->getRelativeCuboid().intersectionRay(r);
    if (intersections.size() > 1) {
      if (source_position.norm() > intersections[0].norm() && source_position.norm() > intersections[1].norm()) {
        double obstacle_track  = (intersections[0] - intersections[1]).norm();
        double obstacle_mac    = calculateMassAttCoeff(sa.getEnergy(), o->getMaterial(), AttenuationType::MASS_ENERGY);
        double absorption_prob = calculateAbsorptionProb(obstacle_track, obstacle_mac, o->getDensity());
        transmission *= (1.0 - absorption_prob);
        cumulative_obstacle_track += obstacle_track;
      }
    }
  }
  double air_track       = sa.getRelativePosition().norm() - cumulative_obstacle_track;
  double air_mac         = sa.getAirMassAttCoeff();
  double absorption_prob = calculateAbsorptionProb(air_track, air_mac, air_density_);
  transmission *= (1.0 - absorption_prob);
  return transmission;
}
//}

/* traceObstaclesId //{ */
std::vector<unsigned int> Timepix3::traceObstaclesId(SourceAbstraction &sa) {
  std::vector<unsigned int> ret;

  Eigen::Vector3d source_position = sa.getRelativePosition();

  mrs_lib::geometry::Ray r = mrs_lib::geometry::Ray::twopointCast(Eigen::Vector3d::Zero(), source_position);

  for (auto o = obstacles_.begin(); o != obstacles_.end(); o++) {
    std::vector<Eigen::Vector3d> intersections = o->getRelativeCuboid().intersectionRay(r);
    if (intersections.size() > 1) {
      if (source_position.norm() > intersections[0].norm() && source_position.norm() > intersections[1].norm()) {
        ret.push_back(o->getId());
      }
    }
  }
  return ret;
}
//}

/* onWorldLateUpdate //{ */
void Timepix3::onWorldLateUpdate() {
  ros::Time t_now = ros::Time::now();
  if (t_now > last_tf_time_) {
    tf::Transform  transform;
    tf::Quaternion quat(model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z(), model_->WorldPose().Rot().W());
    tf::Vector3    origin(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
    transform.setOrigin(origin);
    transform.setRotation(quat);
    transform_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame_.str(), local_frame_.str()));
    last_tf_time_ = ros::Time::now();
  }
}
//}

/* sampleRectangle //{ */
std::pair<Eigen::Vector3d, Eigen::Vector2i> Timepix3::sampleRectangle(mrs_lib::geometry::Rectangle &r) {

  double k1 = rand_dbl_(rand_gen_);
  double k2 = rand_dbl_(rand_gen_);

  Eigen::Vector3d world_pos = r.a() + k1 * (r.b() - r.a()) + k2 * (r.d() - r.a());
  Eigen::Vector2i pixel_coord;
  pixel_coord.x() = int(256 * k1);
  pixel_coord.y() = int(256 * k2);

  return {world_pos, pixel_coord};
}
//}

/* publishSensorMsg //{ */
void Timepix3::publishSensorMsg(const Eigen::Vector2i &pixel_coord, const double photon_energy) {
  rad_msgs::ClusterList msg;
  msg.header.stamp = ros::Time::now();
  std::stringstream ss;
  ss << "minipix" << sensor_suffix_.c_str();
  msg.header.frame_id = ss.str();
  msg.header.seq      = sequence_num_;

  rad_msgs::Cluster c;
  c.energy = photon_energy;
  c.height = 1;
  c.x      = pixel_coord.x();
  c.y      = pixel_coord.y();
  c.size   = 1;
  c.stamp  = ros::Time::now();

  msg.clusters.push_back(c);

  ++sequence_num_;
  ros_publisher_.publish(msg);
}
//}

/* publishEmptyMsg //{ */
void Timepix3::publishEmptyMsg() {
  rad_msgs::ClusterList msg;
  msg.header.stamp = ros::Time::now();
  std::stringstream ss;
  ss << "minipix" << sensor_suffix_.c_str();
  msg.header.frame_id = ss.str();
  msg.header.seq      = sequence_num_;

  ++sequence_num_;
  ros_publisher_.publish(msg);
}
//}

/* publishDiagnostics //{ */
void Timepix3::publishDiagnostics() {

  gazebo_rad_msgs::Timepix3Diagnostics msg;
  msg.stamp       = ros::Time::now();
  msg.gazebo_id   = model_->GetId();
  msg.material    = material_;
  msg.size.x      = size_.X();
  msg.size.y      = size_.Y();
  msg.size.z      = size_.Z();
  msg.world_pos.x = model_->WorldPose().Pos().X();
  msg.world_pos.y = model_->WorldPose().Pos().Y();
  msg.world_pos.z = model_->WorldPose().Pos().Z();
  msg.world_rot.w = model_->WorldPose().Rot().W();
  msg.world_rot.x = model_->WorldPose().Rot().X();
  msg.world_rot.y = model_->WorldPose().Rot().Y();
  msg.world_rot.z = model_->WorldPose().Rot().Z();

  /* add sources //{ */
  for (auto s = sources_.begin(); s != sources_.end(); s++) {
    gazebo_rad_msgs::SourceDiagnostics sd;
    sd.id             = s->getId();
    sd.activity       = s->getActivity();
    sd.energy         = s->getEnergy();
    sd.material       = s->getMaterial();
    sd.relative_pos.x = s->getRelativePosition().x();
    sd.relative_pos.y = s->getRelativePosition().y();
    sd.relative_pos.z = s->getRelativePosition().z();

    int  index           = 0;
    auto side_properties = s->getSideProperties();
    for (auto sp = side_properties.begin(); sp != side_properties.end(); sp++, index++) {
      if (index == 0) {
        sd.exposed_sides.x       = sp->first;
        sd.apparent_activities.x = sp->second;
      } else if (index == 1) {
        sd.exposed_sides.y       = sp->first;
        sd.apparent_activities.y = sp->second;
      } else {
        sd.exposed_sides.z       = sp->first;
        sd.apparent_activities.z = sp->second;
      }
    }
    msg.sources.push_back(sd);
  }
  //}

  /* add obstacles //{ */
  for (auto o = obstacles_.begin(); o != obstacles_.end(); o++) {
    gazebo_rad_msgs::ObstacleDiagnostics od;
    od.id             = o->getId();
    od.material       = o->getMaterial();
    od.relative_pos.x = o->getRelativePosition().x();
    od.relative_pos.y = o->getRelativePosition().y();
    od.relative_pos.z = o->getRelativePosition().z();
    od.relative_rot.w = o->getRelativeOrientation().w();
    od.relative_rot.x = o->getRelativeOrientation().x();
    od.relative_rot.y = o->getRelativeOrientation().y();
    od.relative_rot.z = o->getRelativeOrientation().z();
    od.size.x         = o->getSize().x();
    od.size.y         = o->getSize().y();
    od.size.z         = o->getSize().z();
    msg.obstacles.push_back(od);
  }
  //}

  diagnostics_publisher_.publish(msg);
  debugVisualize();
}
//}

/* debugVisualize //{ */
void Timepix3::debugVisualize() {
  debug_visualizer_.clearBuffers();
  debug_visualizer_.clearVisuals();

  // draw sensor
  for (unsigned int i = 0; i < sides_.size(); i++) {
    debug_visualizer_.addRectangle(sides_[i], BLUE, true);
    debug_visualizer_.addRectangle(sides_[i], BLACK, false);

    // draw side normals
    /* mrs_lib::geometry::Ray r = mrs_lib::geometry::Ray::twopointCast(sides[i].center(), sides[i].center() + sides[i].normal()); */
    /* debug_visualizer.addRay(r); */
  }

  // draw registered sources
  for (unsigned int i = 0; i < sources_.size(); i++) {
    debug_visualizer_.addPoint(sources_[i].getRelativePosition(), GREEN);

    // draw participating obstacles
    /* auto participating_obstacles = traceObstaclesId(sources[i]); */
    /* std::cout << "Traced obstacles:\n"; */
    /* for (unsigned int j = 0; j < participating_obstacles.size(); j++) { */
    /* std::cout << "Obstacle" << participating_obstacles[j] << "\n"; */
    /* } */
  }

  for (unsigned int i = 0; i < obstacles_.size(); i++) {
    debug_visualizer_.addCuboid(obstacles_[i].getRelativeCuboid(), ORANGE, true);
    debug_visualizer_.addCuboid(obstacles_[i].getRelativeCuboid(), BLACK, false);
  }


  debug_visualizer_.publish();
}
//}

}  // namespace gazebo
