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

#define UP Eigen::Vector3d(0.0, 0.0, 1.0)  // unit vector pointing up
#define STEP_DURATION 0.05                 // [s]

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
      static const double timeout            = STEP_DURATION;
      int                 photons_per_second = 0;
      int                 counter            = 0;
      while (this->rosNode->ok()) {
        this->updatePosition(this->model_->WorldPose());
        photons_per_second += this->simulate();
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
        counter++;

        if (counter >= (1 / STEP_DURATION)) {
          ROS_INFO("[Timepix #%u]: Intensity %d Bq", this->node_handle_->GetId(), photons_per_second);
          photons_per_second = 0;
          counter            = 0;
        }
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
    double diagonal_length;
    double diagonal_absorption_prob_Cs137;
    double diagonal_absorption_prob_Am241;

    Material sensor_material;
    double   photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density);

    Rectangle sides[6];

    Eigen::Vector3d sampleRectangle(Rectangle rect);

    physics::EntityPtr modelEntity;

    physics::ModelPtr    model_;
    physics::WorldPtr    world_;
    event::ConnectionPtr updateConnection_;

    transport::NodePtr       node_handle_;
    transport::SubscriberPtr rad_sub;

    void updatePosition(ignition::math::Pose3d world_pose);
    void radiationCallback(RadiationSourceConstPtr &msg);

    boost::thread callback_queue_thread_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::CallbackQueue               rosQueue;
    std::thread                      rosQueueThread;
    ros::Publisher                   test_pub;

    ros::Publisher sources_visualizer_pub, ray_visualizer_pub, track_visualizer_pub, point_visualizer_pub, point1_visualizer_pub, rect1_visualizer_pub,
        rect2_visualizer_pub, rect3_visualizer_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;

    int simulate();  // runs one simulation step for this detector. Return number of captured photons

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
    this->callback_queue_thread_.join();
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

    this->sensor_size = 0.01408;
    /* this->sensor_size = 0.4; */
    this->sensor_thickness = 300e-06;
    /* this->sensor_thickness = 0.1; */
    this->diagonal_length = std::sqrt(2 * sensor_size * sensor_size + sensor_thickness * sensor_thickness);
    this->sensor_material = Si;

    this->diagonal_absorption_prob_Cs137 = photoabsorptionProbability(diagonal_length, sensor_material.pmac_Cs137, sensor_material.density);
    this->diagonal_absorption_prob_Am241 = photoabsorptionProbability(diagonal_length, sensor_material.pmac_Am241, sensor_material.density);

    ROS_INFO("Probability of absorption on body diagonal: Cs137: %.4f, Am241: %.4f", diagonal_absorption_prob_Cs137, diagonal_absorption_prob_Am241);

    this->rad_sub = node_handle_->Subscribe("~/radiation", &Timepix::radiationCallback, this, false);

    this->test_pub               = this->rosNode->advertise<std_msgs::Float64>("/radiation/test", 100);
    this->sources_visualizer_pub = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/sources", 100);
    this->ray_visualizer_pub     = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/ray", 100);
    this->track_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/track", 100);
    this->point_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/point", 100);
    this->point1_visualizer_pub  = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/point1", 100);
    this->rect1_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect1", 100);
    this->rect2_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect2", 100);
    this->rect3_visualizer_pub   = this->rosNode->advertise<visualization_msgs::Marker>("/radiation/visualizer/rect3", 100);

    this->rosQueueThread = std::thread(std::bind(&Timepix::QueueThread, this));

    Eigen::Vector3d A = pos3toVector3d(model_->WorldPose());
    Eigen::Vector3d B = pos3toVector3d(model_->WorldPose());
    Eigen::Vector3d C = pos3toVector3d(model_->WorldPose());
    Eigen::Vector3d D = pos3toVector3d(model_->WorldPose());

    for (int i = 0; i < 6; i++) {
      sides[i] = Rectangle(A, B, C, D);
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

  /* simulate() */  //{
  // return number of captured photons
  int Timepix::simulate() {
    if (this->sources.empty()) {
      return 0;
    }

    std::map<unsigned int, std::vector<Rectangle>> exposed_faces = preprocessSources();

    double begin_time   = ros::Time::now().toSec();
    int    samples_sum  = 0;
    int    captured_sum = 0;
    for (auto it = exposed_faces.begin(); it != exposed_faces.end(); it++) {
      std::vector<Rectangle> faces = it->second;

      for (auto rect = faces.begin(); rect != faces.end(); rect++) {
        for (int i = 0; i < rect->samples; i++) {

          Eigen::Vector3d intersect1 = sampleRectangle(*rect);

          Ray r = Ray::twopointCast(sources.find(it->first)->second.position, intersect1);
          if (ros::Time::now().toSec() > begin_time + STEP_DURATION) {
            /* ROS_WARN("Sampling interrupted. Samples generated: %d", samples_sum); */
            return captured_sum;
          }
          boost::optional<Eigen::Vector3d> intersect2;
          for (int j = 0; j < 6; j++) {
            if (rect->basis == sides[j].basis) {
              continue;
            }
            intersect2 = sides[j].intersectionRay(r);
            if (intersect2) {
              RadiationVisualizer::visualizeRay(track_visualizer_pub, Ray::twopointCast(intersect1, *intersect2));
              RadiationVisualizer::visualizePoint(point1_visualizer_pub, intersect1);
              RadiationVisualizer::visualizePoint(point_visualizer_pub, *intersect2);
              double track_length = (intersect1 - intersect2.get()).norm();
              double pe_prob;
              if (sources.find(it->first)->second.material == "Cs137") {
                pe_prob = photoabsorptionProbability(track_length, sensor_material.pmac_Cs137, sensor_material.density) / diagonal_absorption_prob_Cs137;
              } else if (sources.find(it->first)->second.material == "Am241") {
                pe_prob = photoabsorptionProbability(track_length, sensor_material.pmac_Am241, sensor_material.density) / diagonal_absorption_prob_Am241;
              } else {
                pe_prob = 0;
                ROS_FATAL("Unknown radiation source");
              }
              /* ROS_INFO("Track length: %.4f, Probability: %.4f", track_length, pe_prob); */
              double coin_flip = rand_dbl(rand_gen);
              if (coin_flip < pe_prob) {
                captured_sum++;
              }
              break;
            }
          }
          samples_sum++;
        }
      }
    }
    /* ROS_INFO("Realtime simulation. Samples generated: %d", samples_sum); */
    return captured_sum;
  }
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

    sides[FRONT]  = Rectangle(A, B, C, D);
    sides[BACK]   = Rectangle(E, F, G, H);
    sides[LEFT]   = Rectangle(B, E, H, C);
    sides[RIGHT]  = Rectangle(F, A, D, G);
    sides[BOTTOM] = Rectangle(F, E, B, A);
    sides[TOP]    = Rectangle(D, C, H, G);
  }
  //}

  /* preprocessSources */  //{
  std::map<unsigned int, std::vector<Rectangle>> Timepix::preprocessSources() {

    std::map<unsigned int, std::vector<Rectangle>> ret;

    ros::Time curr_time = ros::Time::now();
    for (auto source = sources.begin(); source != sources.end();) {
      std::vector<Rectangle> exposed_faces;
      /* double                 samples_sum = 0; */
      // clear inactive sources
      if (source->second.last_contact.toSec() + STEP_DURATION < curr_time.toSec()) {
        ROS_INFO("[Timepix #%u]: No longer tracking source #%u. Reason: Timeout", node_handle_->GetId(), source->first);
        source = sources.erase(source);
        continue;
      }

      // Side exposure check
      for (int i = 0; i < 6; i++) {
        if (sides[i].normal_vector.dot(source->second.position - sides[i].center) > 0) {
          double apparent_activity = (source->second.activity / 4 * M_PI) * rectSolidAngle(sides[i], source->second.position);
          source->second.apparent_activity += apparent_activity;
          if (source->second.material == "Cs137") {
            sides[i].samples = apparent_activity * STEP_DURATION * diagonal_absorption_prob_Cs137;
          } else {
            sides[i].samples = apparent_activity * STEP_DURATION * diagonal_absorption_prob_Am241;
          }
          exposed_faces.push_back(sides[i]);
          if (exposed_faces.size() > 2) {
            break;
          }
        }
      }

      if (exposed_faces.size() > 2) {
        RadiationVisualizer::visualizeRect(rect1_visualizer_pub, exposed_faces[0], 0.0, 0.8, 0.8);
        RadiationVisualizer::visualizeRect(rect2_visualizer_pub, exposed_faces[1], 0.8, 0.2, 0.3);
        RadiationVisualizer::visualizeRect(rect3_visualizer_pub, exposed_faces[2]);
      }

      /* ROS_INFO("Apparent activity of Source #%u: %.3f", source->first, source->second.apparent_activity); */
      /* samples_sum += source->second.apparent_activity * STEP_DURATION; */
      /* ROS_INFO("[Timepix #%u]: Source #%u requires %.0f samples", node_handle_->GetId(), source->first, source->second.apparent_activity * STEP_DURATION); */
      ret.insert(std::pair<unsigned int, std::vector<Rectangle>>(source->first, exposed_faces));
      source++;
    }
    return ret;
  }
  //}

  double Timepix::photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density) {
    return 1.0 - std::exp(-mass_att_coeff * material_thickness * 100 * material_density);
  }

}  // namespace gazebo
