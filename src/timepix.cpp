#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>

#include <sdf/sdf.hh>
#include <common.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>

#include <std_msgs/Char.h>

#include <mutex>

#include <gazebo_rad_msgs/RadiationSource.pb.h>

#include "source.h"

namespace gazebo
{

  /* class Timepix //{ */

  class GAZEBO_VISIBLE Timepix : public ModelPlugin {
  public:
    Timepix();
    virtual ~Timepix();

    void QueueThread() {
      static const double timeout = 0.01;
      while (this->rosNode->ok()) {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource> RadiationSourceConstPtr;

  protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo &);

  private:
    std::string                            namespace_;
    std::uniform_real_distribution<double> rand_dbl;
    std::mt19937                           rand_gen;

    std::set<unsigned int> source_ids;

    double sensor_size;
    double sensor_thickness;

    physics::EntityPtr modelEntity;

    physics::ModelPtr    model_;
    physics::WorldPtr    world_;
    event::ConnectionPtr updateConnection_;

    transport::NodePtr       node_handle_;
    transport::SubscriberPtr rad_sub;
    void                     radiationCallback(RadiationSourceConstPtr &msg);

    boost::thread callback_queue_thread_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::CallbackQueue               rosQueue;
    std::thread                      rosQueueThread;
    ros::Publisher                   test_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;
  };

  GZ_REGISTER_MODEL_PLUGIN(Timepix)
  Timepix::Timepix() : ModelPlugin() {
  }

  Timepix::~Timepix() {
    updateConnection_->~Connection();
  }

  //}

  /* radiationCallback() //{ */

  void Timepix::radiationCallback(RadiationSourceConstPtr &msg) {

    /* ROS_INFO_STREAM("x: " << msg->x()); */
    /* ROS_INFO_STREAM("y: " << msg->x()); */
    /* ROS_INFO_STREAM("z: " << msg->x()); */
    /* ROS_INFO_STREAM("activity: " << msg->activity()); */
    /* ROS_INFO_STREAM("material: " << msg->material()); */
    /* ROS_INFO_STREAM("ID: " << msg->id()); */

    // insert a newly encountered source into list of source_ids
    if (source_ids.find(msg->id()) == source_ids.end()) {
      source_ids.insert(msg->id());
      ROS_INFO("[Timepix #%u]: Registered source #%u", node_handle_->GetId(), msg->id());
    }
  }

  //}

  /* Load() //{ */

  void Timepix::Load(physics::ModelPtr _model, [[maybe_unused]] sdf::ElementPtr _sdf) {

    int    argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);

    this->rosNode.reset(new ros::NodeHandle("~"));

    // Store the pointer to the model.
    model_ = _model;

    world_ = model_->GetWorld();

    last_time_     = world_->SimTime();
    last_gps_time_ = world_->SimTime();

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init();


    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Timepix::OnUpdate, this, _1));

    this->modelName = model_->GetName();

    this->rad_sub = node_handle_->Subscribe("~/radiation", &Timepix::radiationCallback, this, false);

    this->test_pub = this->rosNode->advertise<std_msgs::Char>("/radiation/test", 10);

    this->rosQueueThread = std::thread(std::bind(&Timepix::QueueThread, this));

    ROS_INFO("[Timepix]: initialized");
  }

  //}

  /* OnUpdate() //{ */

  void Timepix::OnUpdate(const common::UpdateInfo &) {

    common::Time current_time = world_->SimTime();

    ignition::math::Pose3d T_W_I = model_->WorldPose();

    last_time_ = current_time;
  }

  //}

}  // namespace gazebo
