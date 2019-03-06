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
#include <tf/tf.h>

#include <std_msgs/Float64.h>

#include <mutex>

#include <gazebo_rad_msgs/RadiationSource.pb.h>

#include <geometry_utils.h>
#include <radiation_utils.h>
#include <rviz_visualizer.h>

#define UP Eigen::Vector3d(0.0, 0.0, 1.0)  // unit vector pointing up
#define STEP_DURATION 0.01                 // [s]

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

    void raytracerLoop() {
      ros::Duration stepDuration(STEP_DURATION);
      while (this->rosNode->ok()) {
        updatePosition(this->model_->WorldPose());
        simulate();
        stepDuration.sleep();
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

    std::map<unsigned int, Source> sources;

    double                 sensor_size;
    double                 sensor_thickness;
    Rectangle              front, back;
    std::vector<Rectangle> sides;

    void            updatePosition(ignition::math::Pose3d world_pose);
    Eigen::Vector3d sampleRectangle(Rectangle rect);

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
    std::thread                      simulationThread;
    ros::Publisher                   test_pub;

    ros::Publisher sources_visualizer_pub, ray_visualizer_pub, point_visualizer_pub, sensor_front_visualizer_pub, sensor_back_visualizer_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;
    void        simulate();
  };

  GZ_REGISTER_MODEL_PLUGIN(Timepix)
  Timepix::Timepix() : ModelPlugin() {
  }

  Timepix::~Timepix() {
    // end all ROS related stuff
    this->rosNode->shutdown();
    // wait for threads to finish
    this->rosQueueThread.join();
    this->callback_queue_thread_.join();
    this->simulationThread.detach();
    // destroy connection to gazebo
    updateConnection_->~Connection();
  }

  //}

  /* radiationCallback() //{ */

  void Timepix::radiationCallback(RadiationSourceConstPtr &msg) {

    ros::Time curr_time = ros::Time::now();
    // insert a newly encountered source into set of sources
    if (sources.find(msg->id()) == sources.end()) {
      Eigen::Vector3d pos(msg->x(), msg->y(), msg->z());

      Source s(msg->material(), msg->activity(), pos);
      sources.insert(std::pair<unsigned int, Source>(msg->id(), s));

      ROS_INFO("[Timepix #%u]: Registered source #%u", node_handle_->GetId(), msg->id());
    }
    // store last time the source sent a msg
    sources.find(msg->id())->second.last_contact = curr_time;
  }

  Eigen::Vector3d Timepix::sampleRectangle(Rectangle r) {
    Eigen::Vector3d ab = r.points[1] - r.points[0];
    Eigen::Vector3d ad = r.points[3] - r.points[0];

    double k1 = rand_dbl(rand_gen);
    double k2 = rand_dbl(rand_gen);

    return r.points[0] + k1 * ab + k2 * ad;
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

    rand_dbl = std::uniform_real_distribution<double>(0, 1);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Timepix::OnUpdate, this, _1));

    this->modelName = model_->GetName();

    this->sensor_size = 0.01408;
    /* this->sensor_size      = 1.408; */
    this->sensor_thickness = 300e-06;

    this->rad_sub = node_handle_->Subscribe("~/radiation", &Timepix::radiationCallback, this, false);

    this->test_pub                    = this->rosNode->advertise<std_msgs::Float64>("/radiation/test", 100);
    this->sources_visualizer_pub      = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/sources", 100);
    this->ray_visualizer_pub          = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/ray", 100);
    this->point_visualizer_pub        = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/point", 100);
    this->sensor_front_visualizer_pub = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/sensor_front", 100);
    this->sensor_back_visualizer_pub  = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/sensor_back", 100);

    this->rosQueueThread = std::thread(std::bind(&Timepix::QueueThread, this));

    Eigen::Vector3d A = pos3toVector3d(model_->WorldPose());
    Eigen::Vector3d B = pos3toVector3d(model_->WorldPose());
    Eigen::Vector3d C = pos3toVector3d(model_->WorldPose());
    Eigen::Vector3d D = pos3toVector3d(model_->WorldPose());

    this->front = Rectangle(A, B, C, D);
    this->back  = Rectangle(A, B, C, D);

    for (int i = 0; i < 4; i++) {
      this->sides.push_back(Rectangle(A, B, C, D));
    }

    this->simulationThread = std::thread(boost::bind(&Timepix::raytracerLoop, this));
    /* updatePosition(model_->WorldPose()); */

    ROS_INFO("[Timepix]: initialized");
  }

  //}

  /* OnUpdate() //{ */

  void Timepix::OnUpdate(const common::UpdateInfo &) {

    common::Time           current_time = world_->SimTime();
    ignition::math::Pose3d T_W_I        = model_->WorldPose();
  }
  //}

  void Timepix::simulate() {
    if (this->sources.empty()) {
      return;
    }

    ros::Time curr_time = ros::Time::now();
    for (auto source = sources.begin(); source != sources.end();) {
      if (source->second.last_contact.toSec() + STEP_DURATION < curr_time.toSec()) {
        ROS_INFO("[Timepix #%u]: No longer tracking source #%u", node_handle_->GetId(), source->first);
        source = sources.erase(source);
        continue;
      }
      source->second.apparent_activity = (source->second.activity / 4 * M_PI) * rectSolidAngle(this->front, source->second.position);
      ROS_INFO("[Timepix #%u]: Source #%u requires %.0f samples", node_handle_->GetId(), source->first, source->second.apparent_activity * STEP_DURATION);
      double iteration_start = ros::Time::now().toSec();

      // Each source gets 1/n size of step duration
      // TODO equalize the percent slow for all sources
      for (int i = 0; i < source->second.apparent_activity * STEP_DURATION; i++) {
        Eigen::Vector3d front_intersect = sampleRectangle(this->front);

        Ray r = Ray::twopointCast(source->second.position, front_intersect);
        if (ros::Time::now().toSec() > iteration_start + (STEP_DURATION / sources.size())) {
          ROS_WARN("[Timepix #%u]: Source #%u only recieved %d samples (%.3f%% of original)", node_handle_->GetId(), source->first, i,
                   ((100.0 * i) / (source->second.apparent_activity * STEP_DURATION)));
          break;
        }
      }
      source++;
    }
    /* RadiationVisualizer::visualizeSources(sources_visualizer_pub, sources); */

    /*     for (std::map<unsigned int, Source>::iterator it = sources.begin(); it != sources.end(); it++) { */

    /*       it->second.apparent_activity = (it->second.activity / 4 * M_PI) * rectSolidAngle(this->front, it->second.position); */

    /*       if (it->second.last_ray_time.Double() + (1.0 / it->second.apparent_activity) < current_time.Double()) { */
    /*         // cast ray from source to the front rectangle */
    /*         Eigen::Vector3d front_intersect = sampleRectangle(this->front); */
    /*         RadiationVisualizer::visualizePoint(point_visualizer_pub, front_intersect); */
    /*         Ray r = Ray::twopointCast(it->second.position, front_intersect); */
    /*         RadiationVisualizer::visualizeRay(ray_visualizer_pub, r); */
    /*         it->second.last_ray_time = current_time; */

    /*         // check intersection with back side */
    /*         boost::optional<Eigen::Vector3d> back_intersect = this->back.plane.intersectionRay(r); */
    /*         if (!back_intersect) { */
    /*           ROS_WARN("No intersection with sensor back"); */
    /*           // check intersection with sides */
    /*           for (size_t i = 0; i < this->sides.size(); i++) { */
    /*             boost::optional<Eigen::Vector3d> side_intersect = this->sides[i].plane.intersectionRay(r); */
    /*             if (side_intersect) { */
    /*               double intersection_length = (front_intersect - side_intersect.get()).norm(); */
    /*               ROS_INFO("Intersection length: %.7f", intersection_length); */
    /*               break; */
    /*             } */
    /*           } */

    /*         } else { */
    /*           Eigen::Vector3d projection = back.basis.inverse() * (back.points[3] - (*back_intersect)); */
    /*           /1* ROS_INFO("Intersect in basis: %.6f, %.6f, %.6f", projection[0], projection[1], projection[2]); *1/ */
    /*           if (0 <= projection[0] <= 1.0 and 0 <= projection[1] <= 1.0) { */
    /*             double intersection_length = (*back_intersect - front_intersect).norm(); */
    /*             ROS_INFO("Intersection length: %.7f", intersection_length); */
    /*           } */
    /*         } */
    /*       } */
    /*     } */
    /*     if (current_time.Double() > last_time_.Double() + 0.01) { */
    /*       RadiationVisualizer::visualizeRect(sensor_front_visualizer_pub, this->front, 0.0, 0.2, 1.0); */
    /*       RadiationVisualizer::visualizeRect(sensor_back_visualizer_pub, this->back, 0.0, 0.6, 1.0); */
    /*     } */
    /*     //} */
  }  // namespace gazebo
  //}

  /* updatePosition() */  //{
  void Timepix::updatePosition(ignition::math::Pose3d world_pose) {
    Eigen::Vector3d A(sensor_thickness / 2.0, -sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d B(sensor_thickness / 2.0, sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d C(sensor_thickness / 2.0, sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d D(sensor_thickness / 2.0, -sensor_size / 2.0, -sensor_size / 2.0);

    Eigen::Vector3d pos = pos3toVector3d(world_pose);
    Eigen::Affine3d T;
    T = Eigen::Translation3d(pos);

    ignition::math::Quaterniond iq = world_pose.Rot();
    Eigen::Quaterniond          Q(iq.W(), iq.X(), iq.Y(), iq.Z());

    A = T * Q * A;
    B = T * Q * B;
    C = T * Q * C;
    D = T * Q * D;

    Eigen::Vector3d E(-sensor_thickness / 2.0, -sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d F(-sensor_thickness / 2.0, sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d G(-sensor_thickness / 2.0, sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d H(-sensor_thickness / 2.0, -sensor_size / 2.0, -sensor_size / 2.0);

    E = T * Q * E;
    F = T * Q * F;
    G = T * Q * G;
    H = T * Q * H;

    this->front = Rectangle(A, B, C, D);
    this->back  = Rectangle(E, F, G, H);

    sides[0] = Rectangle(A, E, H, D);
    sides[1] = Rectangle(B, F, E, A);
    sides[2] = Rectangle(C, G, F, B);
    sides[3] = Rectangle(D, H, C, G);
  }
  //}

}  // namespace gazebo

