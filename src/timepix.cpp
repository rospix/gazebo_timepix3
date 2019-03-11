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
/* #include <materials.h> */

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
      static const double timeout = STEP_DURATION;
      while (this->rosNode->ok()) {
        this->updatePosition(this->model_->WorldPose());
        this->simulate();

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

    std::map<unsigned int, Source> sources;

    double sensor_size;
    double sensor_thickness;
    /* Material sensor_material; */

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
    ros::Publisher                   test_pub;

    ros::Publisher sources_visualizer_pub, ray_visualizer_pub, point_visualizer_pub, rect1_visualizer_pub, rect2_visualizer_pub, rect3_visualizer_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;

    void                                           simulate();
    std::map<unsigned int, std::vector<Rectangle>> preprocessSources();  // remove inactive sources, map source IDs to rectangles which will be sampled
  };

  GZ_REGISTER_MODEL_PLUGIN(Timepix)
  Timepix::Timepix() : ModelPlugin() {
  }

  Timepix::~Timepix() {
    // end all ROS related stuff
    this->rosNode->shutdown();
    // wait for threads to finish
    this->rosQueueThread.join();
    this->callback_queue_thread_.interrupt();
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

    double k1 = rand_dbl(rand_gen);
    double k2 = rand_dbl(rand_gen);

    Eigen::Vector3d sampled_point = r.points[0] + k1 * r.basis.col(0) + k2 * r.basis.col(1);
    return sampled_point;
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

    /* this->sensor_size      = 0.01408; */
    this->sensor_size = 0.4;
    /* this->sensor_thickness = 300e-06; */
    this->sensor_thickness = 0.1;
    /* this->sensor_material  = Si; */

    this->rad_sub = node_handle_->Subscribe("~/radiation", &Timepix::radiationCallback, this, false);

    this->test_pub               = this->rosNode->advertise<std_msgs::Float64>("/radiation/test", 100);
    this->sources_visualizer_pub = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/sources", 100);
    this->ray_visualizer_pub     = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/ray", 100);
    this->point_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/point", 100);
    this->rect1_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect1", 100);
    this->rect2_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect2", 100);
    this->rect3_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect3", 100);

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

    std::map<unsigned int, std::vector<Rectangle>> exposed_faces = preprocessSources();
    // TODO iterate over the map -> first:=source index, second:=faces to be sampled

    return;
  }
  /*       samples_sum += source->second.apparent_activity * STEP_DURATION; */
  /*       /1* ROS_INFO("[Timepix #%u]: Source #%u requires %.0f samples", node_handle_->GetId(), source->first, source->second.apparent_activity *
   * STEP_DURATION); *1/ */
  /*       source++; */
  /*     } */

  /*     for (auto source = sources.begin(); source != sources.end(); source++) { */
  /*       double iteration_start              = ros::Time::now().toSec(); */
  /*       source->second.time_slot_percentage = (source->second.apparent_activity * STEP_DURATION) / samples_sum; */
  /*       bool front_exposed                  = this->front.normal_vector.dot(source->second.position - this->front.points[0]) < 0; */
  /*       for (int i = 0; i < source->second.apparent_activity * STEP_DURATION; i++) { */
  /*         Eigen::Vector3d intersect1 = sampleRectangle(front_exposed ? front : back); */

  /*         Ray r = Ray::twopointCast(source->second.position, intersect1); */
  /*         if (ros::Time::now().toSec() > iteration_start + source->second.time_slot_percentage * STEP_DURATION) { */
  /*           ROS_WARN("[Timepix #%u]: Source #%u only recieved %d samples (%.3f%% of required samples, using %.3f%% of simulation step)",
   * node_handle_->GetId(), */
  /*                    source->first, i, ((100.0 * i) / (source->second.apparent_activity * STEP_DURATION)), (100.0 * source->second.time_slot_percentage)); */
  /*           break; */
  /*         } */
  /*         boost::optional<Eigen::Vector3d> intersect2 = (front_exposed ? back : front).intersectionRay(r); */

  /*         // calculate second intersection with sensor */
  /*         if (!intersect2) { */
  /*           ROS_WARN("No intersection with sensor back"); */
  /*           for (size_t i = 0; i < this->sides.size(); i++) { */
  /*             intersect2 = this->sides[i].intersectionRay(r); */
  /*             if (intersect2) { */
  /*               break; */
  /*             } */
  /*             ROS_FATAL("No intersect found"); */
  /*             return; */
  /*           } */
  /*         } */
  /*         double track_length = (intersect1 - intersect2.get()).norm(); */

  /*         /1* double cross_section = this->photoelectricCrossSection(this->sensor_material, 0.662); *1/ */
  /*         /1* double cross_section = this->photoelectricCrossSection(this->sensor_material, r.energy); *1/ */
  /*         /1* double capture_prob = std::exp(-sensor_material.element_quantities[0] * sensor_material.molecular_density * cross_section * track_length); *1/
   */
  /*         /1* ROS_INFO("Capture probability: %.4f", (1-capture_prob)); *1/ */

  /*         /1* ROS_INFO("Track length: %.9f", track_length); *1/ */
  /*         RadiationVisualizer::visualizeRay(ray_visualizer_pub, r); */
  /*         RadiationVisualizer::visualizePoint(point_visualizer_pub, intersect2.get()); */
  /*       } */
  /*     } */
  /*   } */
  //}

  /* updatePosition() */  //{
  void Timepix::updatePosition(ignition::math::Pose3d world_pose) {
    Eigen::Vector3d A(sensor_thickness / 2.0, -sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d B(sensor_thickness / 2.0, sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d C(sensor_thickness / 2.0, sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d D(sensor_thickness / 2.0, -sensor_size / 2.0, sensor_size / 2.0);

    Eigen::Vector3d pos = pos3toVector3d(world_pose);
    Eigen::Affine3d T;
    T = Eigen::Translation3d(pos);

    ignition::math::Quaterniond iq = world_pose.Rot();
    Eigen::Quaterniond          Q(iq.W(), iq.X(), iq.Y(), iq.Z());

    A = T * Q * A;
    B = T * Q * B;
    C = T * Q * C;
    D = T * Q * D;

    Eigen::Vector3d E(-sensor_thickness / 2.0, sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d F(-sensor_thickness / 2.0, -sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d G(-sensor_thickness / 2.0, -sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d H(-sensor_thickness / 2.0, sensor_size / 2.0, sensor_size / 2.0);

    E = T * Q * E;
    F = T * Q * F;
    G = T * Q * G;
    H = T * Q * H;

    this->front = Rectangle(A, B, C, D);
    this->back  = Rectangle(E, F, G, H);

    sides[0] = Rectangle(B, E, H, C);
    sides[1] = Rectangle(F, A, D, G);
    sides[2] = Rectangle(F, E, B, A);
    sides[3] = Rectangle(D, C, H, G);
  }
  //}

  /* preprocessSources */  //{
  std::map<unsigned int, std::vector<Rectangle>> Timepix::preprocessSources() {

    std::map<unsigned int, std::vector<Rectangle>> ret;

    ros::Time curr_time = ros::Time::now();
    for (auto source = sources.begin(); source != sources.end();) {
      std::vector<Rectangle> exposed_faces;
      double                 samples_sum = 0;
      // clear inactive sources
      if (source->second.last_contact.toSec() + STEP_DURATION < curr_time.toSec()) {
        ROS_INFO("[Timepix #%u]: No longer tracking source #%u. Reason: Timeout", node_handle_->GetId(), source->first);
        source = sources.erase(source);
        continue;
      }

      // Front/Back face exposure check
      bool front_exposed = front.normal_vector.dot(source->second.position - front.points[0]) > 0;
      exposed_faces.push_back(front_exposed ? front : back);
      source->second.apparent_activity = (source->second.activity / 4 * M_PI) * rectSolidAngle(exposed_faces[0], source->second.position);

      // Side face exposure check
      for (size_t i = 0; i < sides.size(); i++) {
        if (exposed_faces.size() <= 3 && sides[i].normal_vector.dot(source->second.position - sides[i].points[0]) > 0) {
          source->second.apparent_activity += (source->second.activity / 4 * M_PI) * rectSolidAngle(sides[i], source->second.position);
          exposed_faces.push_back(sides[i]);
        }
      }

      if (exposed_faces.size() > 2) {
        RadiationVisualizer::visualizeRect(rect1_visualizer_pub, exposed_faces[0], 0.0, 0.8, 0.8);
        RadiationVisualizer::visualizeRect(rect2_visualizer_pub, exposed_faces[1], 0.8, 0.2, 0.3);
        RadiationVisualizer::visualizeRect(rect3_visualizer_pub, exposed_faces[2]);
      }

      ROS_INFO("Apparent activity of Source #%u: %.3f", source->first, source->second.apparent_activity);
      samples_sum += source->second.apparent_activity * STEP_DURATION;
      ROS_INFO("[Timepix #%u]: Source #%u requires %.0f samples", node_handle_->GetId(), source->first, source->second.apparent_activity * STEP_DURATION);
      ret.insert(std::pair<unsigned int, std::vector<Rectangle>>(source->first, exposed_faces));
      source++;
    }
    return ret;
  }
  //}

}  // namespace gazebo
