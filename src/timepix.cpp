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
#include <std_msgs/Int32.h>

#include <mutex>

#include <gazebo_rad_msgs/RadiationSource.pb.h>

#include <geometry_utils.h>
#include <radiation_utils.h>
#include <rviz_visualizer.h>
#include <materials.h>

#include <chrono>

#define UP Eigen::Vector3d(0.0, 0.0, 1.0)  // unit vector pointing up

#define TIMEOUT 2.0          // [s]
#define SIMULATION_STEP 0.1  // [s]

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
        auto sim_start        = std::chrono::high_resolution_clock::now();
        int  captured_photons = simulate(sim_start);
        auto sim_nanoseconds  = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - sim_start).count();
        if (sim_nanoseconds < SIMULATION_STEP) {
          ros::Duration(sim_nanoseconds - SIMULATION_STEP).sleep();
        }
        std_msgs::Int32 msg;
        msg.data = captured_photons;
        /* msg.data = 0; */
        medipix_pub.publish(msg);
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
    std::mutex          sources_mutex;

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

    ros::Publisher medipix_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;

    int  simulate(std::chrono::high_resolution_clock::time_point time_start);
    void oneDebuggingRay();
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

    /* sources_mutex.lock(); */
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
        s->last_contact        = std::chrono::high_resolution_clock::now();
        /* sources_mutex.unlock(); */
        return;
      }
    }
    // new source -> compute params and add to list
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
    s.last_contact  = std::chrono::high_resolution_clock::now();
    s.exposed_sides = exposed_sides;
    sources.push_back(s);
    ROS_INFO("Added to list");
    /* sources_mutex.unlock(); */
  }

  Eigen::Vector3d Timepix::sampleRectangle(Rectangle r) {

    double k1 = rand_dbl(rand_gen);
    double k2 = rand_dbl(rand_gen);

    return r.points[0] + k1 * r.basis.col(0) + k2 * r.basis.col(1);
  }

  Eigen::Vector3d Timepix::sampleSide(int index) {

    double k1 = rand_dbl(rand_gen);
    double k2 = rand_dbl(rand_gen);

    return sides[index].points[0] + k1 * sides[index].basis.col(0) + k2 * sides[index].basis.col(1);
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

    this->sensor_size      = 0.01408;
    this->sensor_thickness = 300e-06;
    /* this->sensor_size      = 0.4; */
    /* this->sensor_thickness = 0.1; */

    this->diagonal_length = std::sqrt(2 * sensor_size * sensor_size + sensor_thickness * sensor_thickness);
    this->sensor_material = Si;

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
    std::stringstream ss;
    ss << "/timepix" << model_->GetId() << "/photon_count";
    this->medipix_pub = this->rosNode->advertise<std_msgs::Int32>(ss.str().c_str(), 100);

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

  int Timepix::simulate(std::chrono::high_resolution_clock::time_point time_start) {
    int photons_captured = 0;
    int photons_total    = 0;
    /* oneDebuggingRay(); */
    /* return; */

    for (auto s = sources.begin(); s != sources.end();) {
      // Remove inactive sources
      if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - s->last_contact).count() > TIMEOUT) {
        ROS_INFO("[Timepix%u]: No longer tracking source%u. Reason: Timeout", this->model_->GetId(), s->id);
        sources.erase(s);
        continue;
      }

      // VISUALIZE
      /* if (s->exposed_sides.size() > 0) { */
      /* RadiationVisualizer::visualizeRect(rect1_visualizer_pub, sides[s->exposed_sides[0]], "/base_link"); */
      /* rect1_visualizer_pub, move(sides[s->exposed_sides[0]], pos3toVector3d(model_->WorldPose()), pos3toQuaterniond(model_->WorldPose())),
                "/base_link"); */
      /* } */
      /* if (s->exposed_sides.size() > 1) { */
      /* RadiationVisualizer::visualizeRect(rect2_visualizer_pub, sides[s->exposed_sides[1]], "/base_link"); */
      /* rect2_visualizer_pub, move(sides[s->exposed_sides[1]], pos3toVector3d(model_->WorldPose()),
                 pos3toQuaterniond(model_->WorldPose())),
                 "/base_link"); */
      /* } */
      /* if (s->exposed_sides.size() > 2) { */
      /* RadiationVisualizer::visualizeRect(rect3_visualizer_pub, sides[s->exposed_sides[2]], "/base_link"); */
      /* rect3_visualizer_pub, move(sides[s->exposed_sides[2]], pos3toVector3d(model_->WorldPose()), pos3toQuaterniond(model_->WorldPose())),
         "/base_link"); */
      /* } */
      /* // VISUALIZE */

      for (size_t i = 0; i < s->exposed_sides.size(); i++) {

        int samples;
        if (s->material == "Cs137") {
          samples = s->apparent_activities[i] * SIMULATION_STEP * diagonal_absorption_prob_Cs137;
        } else if (s->material == "Am241") {
          samples = s->apparent_activities[i] * SIMULATION_STEP * diagonal_absorption_prob_Am241;
        } else {
          return 0;
        }

        /* ROS_INFO("Generating %d samples for side %d", samples, s->exposed_sides[i]); */
        for (int n = 0; n < samples; n++) {
          photons_total++;
          long elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
          if (elapsed_nanoseconds > SIMULATION_STEP * 1E9) {
            /* ROS_INFO("Simulated particles: %d, Captured: %d, Time: %ld ns", photons_total, photons_captured, elapsed_nanoseconds); */
            /* ROS_INFO("Simulation speed: %.3f particles per second", (photons_total * 1E9) / elapsed_nanoseconds); */
            ROS_WARN("Forced interrupt");
            return photons_captured;
          }
          /* for (int n = 0; n < 1; n++) { */
          /* auto            sample_start = std::chrono::high_resolution_clock::now(); */
          Eigen::Vector3d intersect1 = sampleSide(s->exposed_sides[i]);
          /* auto            time1        = std::chrono::high_resolution_clock::now(); */
          /* RadiationVisualizer::visualizePoint(point_visualizer_pub, intersect1, "/base_link"); */
          Ray r = Ray::twopointCast(s->relative_position, intersect1);
          /* auto                                           time2 = std::chrono::high_resolution_clock::now(); */
          /* std::chrono::high_resolution_clock::time_point time3, time4, time5; */

          /* RadiationVisualizer::visualizeRay(ray_visualizer_pub, r, "/base_link"); */
          for (int j = 0; j < 6; j++) {
            if (j == s->exposed_sides[i]) {
              continue;
            }
            boost::optional<Eigen::Vector3d> intersect2 = sides[j].intersectionRay(r);
            if (intersect2) {

              /* time3 = std::chrono::high_resolution_clock::now(); */
              /* RadiationVisualizer::visualizePoint(point1_visualizer_pub, intersect2.get(), "/base_link"); */
              double track_length = (intersect2.get() - intersect1).norm();
              double pe_prob;
              RadiationVisualizer::visualizeRay(track_visualizer_pub, Ray(intersect1, intersect2.get()), "/base_link");
              if (s->material == "Cs137") {
                pe_prob = photoabsorptionProbability(track_length, sensor_material.pmac_Cs137, sensor_material.density) / diagonal_absorption_prob_Cs137;
              } else if (s->material == "Am241") {
                pe_prob = photoabsorptionProbability(track_length, sensor_material.pmac_Am241, sensor_material.density) / diagonal_absorption_prob_Am241;
              }
              /* ROS_INFO("Track length: %.4f um, Probability: %.4f", track_length * 1E6, pe_prob); */
              /* time4            = std::chrono::high_resolution_clock::now(); */
              double coin_flip = rand_dbl(rand_gen);
              if (coin_flip < pe_prob) {
                photons_captured++;
                /* ROS_INFO("Photon captured"); */
                /* RadiationVisualizer::visualizePoint(point_visualizer_pub, intersect2.get(), "/base_link", 0.7, 0.7, 0.0); */
              }
              /* time5 = std::chrono::high_resolution_clock::now(); */
              break;
            }
          }
          /* auto sample_stop = std::chrono::high_resolution_clock::now(); */
          /* std::cout << "Generating intersect1: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time1 - sample_start).count() << "ns\n"; */
          /* std::cout << "Raycast: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time2 - time1).count() << "ns\n"; */
          /* std::cout << "Finding intersect2: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time3 - time2).count() << "ns\n"; */
          /* std::cout << "Probability calculation: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time4 - time3).count() << "ns\n"; */
          /* std::cout << "Coin flip: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time5 - time4).count() << "ns\n"; */
          /* auto   cycle_length = std::chrono::duration_cast<std::chrono::nanoseconds>(sample_stop - sample_start); */
          /* double frequency    = 1 / (cycle_length.count() * 1E-9); */
          /* std::cout << "Sample time: " << std::chrono::duration_cast<std::chrono::nanoseconds>(sample_stop - sample_start).count() << "ns\n"; */
          /* std::cout << "Theoretical maximum: " << frequency << " particles per second\n#######################\n"; */
        }
      }
      s++;
      long elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - time_start).count();
      /* ROS_INFO("Simulated particles: %d, Captured: %d, Time: %ld ns", photons_total, photons_captured, elapsed_nanoseconds); */
      /* ROS_INFO("Simulation speed: %.3f particles per second", (photons_total * 1E9) / elapsed_nanoseconds); */
    }
    return photons_captured;
  }  // namespace gazebo

  double Timepix::photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density) {
    return 1.0 - std::exp(-mass_att_coeff * material_thickness * 100 * material_density);
  }

  void Timepix::oneDebuggingRay() {

    auto                             sample_start = std::chrono::high_resolution_clock::now();
    Eigen::Vector3d                  intersect1   = sampleSide(0);
    auto                             time1        = std::chrono::high_resolution_clock::now();
    Ray                              r            = Ray::twopointCast(sources[0].relative_position, intersect1);
    auto                             time2        = std::chrono::high_resolution_clock::now();
    boost::optional<Eigen::Vector3d> intersect2   = sides[1].intersectionRay(r);
    auto                             time3        = std::chrono::high_resolution_clock::now();
    /* RadiationVisualizer::visualizeRect(rect1_visualizer_pub, sides[0], "/base_link", 0.0, 0.2, 1.0); */
    /* RadiationVisualizer::visualizeRect(rect2_visualizer_pub, sides[1], "/base_link", 1.0, 0.2, 0.0); */
    /* RadiationVisualizer::visualizePoint(point_visualizer_pub, intersect1, "/base_link", 0.0, 0.0, 1.0); */
    /* if (intersect2) */
    /*   RadiationVisualizer::visualizePoint(point1_visualizer_pub, intersect2.get(), "/base_link", 0.5, 1.0, 0.0); */
    /* RadiationVisualizer::visualizeRay(ray_visualizer_pub, r, "/base_link"); */
    auto time4       = std::chrono::high_resolution_clock::now();
    auto sample_stop = std::chrono::high_resolution_clock::now();
    /* std::cout << "Side sampling: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time1 - sample_start).count() << "ns\n"; */
    /* std::cout << "Raycast: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time2 - time1).count() << "ns\n"; */
    /* std::cout << "Second intersect:: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time3 - time2).count() << "ns\n"; */
    /* std::cout << "Visualization: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time4 - time3).count() << "ns\n"; */
    std::cout << "Time per sample: " << std::chrono::duration_cast<std::chrono::nanoseconds>(sample_stop - sample_start).count()
              << "ns\n#############################\n";
  }

}  // namespace gazebo
