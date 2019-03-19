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
#include <materials.h>

#include <chrono>

#define UP Eigen::Vector3d(0.0, 0.0, 1.0)  // unit vector pointing up

#define TIMEOUT 0.5           // [s]
#define SIMULATION_STEP 0.05  // [s]

#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define BOTTOM 4
#define TOP 5

namespace gazebo
{
  /* class Timepix //{ */

  class GAZEBO_VISIBLE Timepix : public ModelPlugin {
  public:
    Timepix();
    virtual ~Timepix();

    void QueueThread() {
      while (this->rosNode->ok()) {
        this->rosQueue.callAvailable(ros::WallDuration(0.1));
      }
    }

    void SimulationThread() {
      while (this->rosNode->ok()) {
        if (sides.size() < 6) {
          ros::Duration(1.0).sleep();
          continue;
        }
        simulate();
        /* ROS_INFO("Timepix running"); */
        ros::Duration(SIMULATION_STEP).sleep();
      }
    }

    typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource> RadiationSourceConstPtr;

  protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo &);

  private:
    std::uniform_real_distribution<double> rand_dbl;
    std::mt19937                           rand_gen;

    std::vector<Source> sources;

    double sensor_size;
    double sensor_thickness;
    double diagonal_length;
    double diagonal_absorption_prob_Cs137;
    double diagonal_absorption_prob_Am241;

    Material sensor_material;
    double   photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density);

    void updatePosition(ignition::math::Pose3d world_pose);
    void radiationCallback(RadiationSourceConstPtr &msg);

    std::vector<Rectangle> sides;
    Eigen::Vector3d        sampleRectangle(Rectangle rect);
    Eigen::Vector3d        sampleSide(int index);

    physics::EntityPtr   modelEntity;
    physics::ModelPtr    model_;
    physics::WorldPtr    world_;
    event::ConnectionPtr updateConnection_;

    Eigen::Quaterniond local2world;
    Eigen::Quaterniond world2local;

    transport::NodePtr       node_handle_;
    transport::SubscriberPtr rad_sub;

    boost::thread callback_queue_thread_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::CallbackQueue               rosQueue;
    std::thread                      rosQueueThread;
    std::thread                      simThread;

    ros::Publisher sources_visualizer_pub, ray_visualizer_pub, track_visualizer_pub, point_visualizer_pub, point1_visualizer_pub, rect1_visualizer_pub,
        rect2_visualizer_pub, rect3_visualizer_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;

    void simulate();  // runs one simulation step for this detector. Return number of captured photons
  };

  GZ_REGISTER_MODEL_PLUGIN(Timepix)
  Timepix::Timepix() : ModelPlugin() {
  }

  Timepix::~Timepix() {
    // end all ROS related stuff
    this->rosNode->shutdown();
    // wait for threads to finish
    this->simThread.join();
    this->rosQueueThread.join();
    this->callback_queue_thread_.join();
    // destroy connection to gazebo
    updateConnection_->~Connection();
  }

  //}

  /* radiationCallback() //{ */

  void Timepix::radiationCallback(RadiationSourceConstPtr &msg) {

    auto timer1 = std::chrono::high_resolution_clock::now();

    /* ROS_INFO("Time: %.3f", curr_time.toSec()); */
    Eigen::Vector3d pos(msg->x(), msg->y(), msg->z());

    for (auto s = sources.begin(); s != sources.end(); s++) {
      // known source -> update its params
      if (s->id == msg->id()) {
        Eigen::Vector3d relative_position = world2local * (pos - pos3toVector3d(model_->WorldPose()));
        s->relative_position              = relative_position;
        s->activity                       = msg->activity();
        std::vector<double> apparent_activities;
        std::vector<int>    exposed_sides;
        for (int i = 0; exposed_sides.size() < 3 && i < 6; i++) {
          if (sides[i].normal_vector.dot(s->relative_position) > 0) {
            double apparent_activity = (s->activity / 4 * M_PI) * rectSolidAngle(sides[i], relative_position);
            apparent_activities.push_back(apparent_activity);
            exposed_sides.push_back(i);
          }
        }
        s->apparent_activities = apparent_activities;
        s->exposed_sides       = exposed_sides;
        s->last_contact        = ros::Time::now();
        auto timer2            = std::chrono::high_resolution_clock::now();
        /* std::cout << "Radiation callback: " << std::chrono::duration_cast<std::chrono::milliseconds>(timer2 - timer1).count() << "ms\n"; */
        /* std::cout << "Sources: " << sources.size() << "\n"; */
        return;
      }
    }
    // unknown source -> calculate params and add to list
    ROS_INFO("[Timepix%u]: Registered source%u", this->model_->GetId(), msg->id());
    Eigen::Vector3d     relative_position = world2local * (pos - pos3toVector3d(model_->WorldPose()));
    std::vector<double> apparent_activities;
    std::vector<int>    exposed_sides;
    for (int i = 0; exposed_sides.size() < 3 && i < 6; i++) {
      if (sides[i].normal_vector.dot(relative_position) > 0) {
        double apparent_activity = (msg->activity() / 4 * M_PI) * rectSolidAngle(sides[i], relative_position);
        apparent_activities.push_back(apparent_activity);
        exposed_sides.push_back(i);
      }
    }
    Source s(msg->id(), msg->material(), msg->activity(), apparent_activities, relative_position);
    s.last_contact  = ros::Time::now();
    s.exposed_sides = exposed_sides;
    sources.push_back(s);
    auto timer2 = std::chrono::high_resolution_clock::now();
    /* std::cout << "Radiation callback: " << std::chrono::duration_cast<std::chrono::milliseconds>(timer2 - timer1).count() << "ms\n"; */
    /* std::cout << "Sources: " << sources.size() << "\n"; */
  }

  Eigen::Vector3d Timepix::sampleRectangle(Rectangle r) {

    double k1 = rand_dbl(rand_gen);
    double k2 = rand_dbl(rand_gen);

    Eigen::Vector3d sampled_point = r.points[0] + k1 * r.basis.col(0) + k2 * r.basis.col(1);
    return sampled_point;
  }

  Eigen::Vector3d Timepix::sampleSide(int index) {

    double k1 = rand_dbl(rand_gen);
    double k2 = rand_dbl(rand_gen);

    Eigen::Vector3d sampled_point = sides[index].points[0] + k1 * sides[index].basis.col(0) + k2 * sides[index].basis.col(1);
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

    this->sensor_size = 0.01408;
    /* this->sensor_size = 0.4; */
    this->sensor_thickness = 300e-06;
    /* this->sensor_thickness = 0.1; */
    this->diagonal_length  = std::sqrt(2 * sensor_size * sensor_size + sensor_thickness * sensor_thickness);
    this->sensor_material  = Si;

    this->diagonal_absorption_prob_Cs137 = photoabsorptionProbability(diagonal_length, sensor_material.pmac_Cs137, sensor_material.density);
    this->diagonal_absorption_prob_Am241 = photoabsorptionProbability(diagonal_length, sensor_material.pmac_Am241, sensor_material.density);

    ROS_INFO("Probability of absorption on body diagonal: Cs137: %.4f, Am241: %.4f", diagonal_absorption_prob_Cs137, diagonal_absorption_prob_Am241);

    this->rad_sub = node_handle_->Subscribe("~/radiation", &Timepix::radiationCallback, this, false);

    this->sources_visualizer_pub = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/sources", 100);
    this->ray_visualizer_pub     = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/ray", 100);
    this->track_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/track", 100);
    this->point_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/point", 100);
    this->point1_visualizer_pub  = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/point1", 100);
    this->rect1_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect1", 100);
    this->rect2_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect2", 100);
    this->rect3_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect3", 100);

    this->rosQueueThread = std::thread(std::bind(&Timepix::QueueThread, this));
    this->simThread      = std::thread(std::bind(&Timepix::SimulationThread, this));

    for (int i = 0; i < 6; i++) {
      sides.push_back(Rectangle());
    }

    Eigen::Vector3d A(sensor_thickness / 2.0, -sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d B(sensor_thickness / 2.0, sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d C(sensor_thickness / 2.0, sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d D(sensor_thickness / 2.0, -sensor_size / 2.0, sensor_size / 2.0);

    Eigen::Vector3d E(-sensor_thickness / 2.0, sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d F(-sensor_thickness / 2.0, -sensor_size / 2.0, -sensor_size / 2.0);
    Eigen::Vector3d G(-sensor_thickness / 2.0, -sensor_size / 2.0, sensor_size / 2.0);
    Eigen::Vector3d H(-sensor_thickness / 2.0, sensor_size / 2.0, sensor_size / 2.0);

    sides[FRONT]  = Rectangle(A, B, C, D);
    sides[BACK]   = Rectangle(E, F, G, H);
    sides[LEFT]   = Rectangle(B, E, H, C);
    sides[RIGHT]  = Rectangle(F, A, D, G);
    sides[BOTTOM] = Rectangle(F, E, B, A);
    sides[TOP]    = Rectangle(D, C, H, G);

    ROS_INFO("[Timepix]: initialized");
  }

  //}

  /* OnUpdate() //{ */

  void Timepix::OnUpdate(const common::UpdateInfo &) {
    ignition::math::Quaterniond q = model_->WorldPose().Rot();
    world2local                   = Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z());
    local2world                   = world2local.inverse();
  }
  //}

  void Timepix::simulate() {
    for (auto s = sources.begin(); s != sources.end();) {
      // Remove inactive sources
      if (s->last_contact.toSec() + TIMEOUT < ros::Time::now().toSec()) {
        ROS_INFO("[Timepix%u]: No longer tracking source%u. Reason: Timeout", this->model_->GetId(), s->id);
        sources.erase(s);
        continue;
      }
      for (size_t i = 0; i < s->exposed_sides.size(); i++) {
        int samples = s->apparent_activities[i] * SIMULATION_STEP;
        ROS_INFO("Generating %d samples for side %d", samples, s->exposed_sides[i]);
        for (int n = 0; n < samples; n++) {
          Eigen::Vector3d                  intersect1 = sampleSide(s->exposed_sides[i]);
          Ray                              r          = Ray::twopointCast(s->relative_position, intersect1);
          boost::optional<Eigen::Vector3d> intersect2;
          for (int j = 0; j < 6; j++) {
            if (j == s->exposed_sides[i]) {
              continue;
            }
            intersect2 = sides[j].intersectionRay(r);
            if (intersect2) {
              ////TODO continue here
              /* ROS_INFO("Intersect with side: %d", j); */
              break;
            }
          }
          /* RadiationVisualizer::visualizePoint(point_visualizer_pub, intersect2.get()); */
        }
      }
      s++;
    }
  }

  double Timepix::photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density) {
    return 1.0 - std::exp(-mass_att_coeff * material_thickness * 100 * material_density);
  }

}  // namespace gazebo
