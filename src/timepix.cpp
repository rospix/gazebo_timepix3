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
#include <gazebo_rad_msgs/RadiationObstacle.pb.h>

#include <geometry_visual_utils/visual_utils.h>

#include <radiation_utils.h>
#include <materials.h>

#include <tf/transform_broadcaster.h>

#include <chrono>
#include <time.h>

#define UP Eigen::Vector3d(0.0, 0.0, 1.0)  // unit vector pointing up

#define TIMEOUT 1.5           // [s]
#define SIMULATION_STEP 0.01  // [s]
#define EXPOSITION_TIME 1.0   // [s]

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
      while (!terminated) {
        rosQueue.callAvailable(ros::WallDuration(0.1));
      }
    }

    void SimulationThread() {
      while (!terminated) {

        auto step_start = std::chrono::high_resolution_clock::now();

        // VISUALIZE
        bv.clear();
        bv.addCuboid(sensor_cuboid);

        obstacles_mutex.lock();
        for (auto o = obstacles.begin(); o != obstacles.end();) {
          // VISUALIZE
          bv.addCuboid(o->cuboid, 0.4, 0.9, 0.4);
          if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - o->last_contact).count() > TIMEOUT) {
            ROS_INFO("[Timepix%u]: No longer tracking Obstacle%u. Reason: Timeout", model_->GetId(), o->id);
            obstacles.erase(o);
            continue;
          }
          o++;
        }
        obstacles_mutex.unlock();

        // VISUALIZE
        for (Source s : sources) {
          bv.addPoint(s.relative_position);
        }

        auto sim_start = std::chrono::high_resolution_clock::now();
        sources_mutex.lock();
        std::pair<int, std::chrono::high_resolution_clock::time_point> sim_output = simulate(sim_start);

        photon_readout += sim_output.first;
        auto sim_end = sim_output.second;

        sources_mutex.unlock();

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - last_readout).count() > EXPOSITION_TIME) {
          ROS_INFO("[Timepix%u]: Particle flux: %d per second", model_->GetId(), photon_readout);
          std_msgs::Int32 msg;
          msg.data = photon_readout;
          medipix_pub.publish(msg);

          photon_readout = 0;
          last_readout   = sim_end;
        }

        auto   sim_ns      = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - sim_start).count();
        double sim_seconds = (double)(sim_ns / 1E9);
        if (sim_seconds < SIMULATION_STEP) {
          std::this_thread::sleep_for(std::chrono::duration<double>(SIMULATION_STEP - sim_seconds));
        }

        tf::Transform  transform;
        tf::Quaternion quat(model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z(), model_->WorldPose().Rot().W());
        tf::Vector3    origin(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
        transform.setOrigin(origin);
        transform.setRotation(quat);
        transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin", frame_name.str()));

        bv.publish();
        /* auto   step_end    = std::chrono::high_resolution_clock::now(); */
        /* double duration_ms = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(step_end - step_start).count() / 1E6); */
        /* ROS_WARN("Step duration: %.2f ms", duration_ms); */
      }
    }

    typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationSource>   RadiationSourceConstPtr;
    typedef const boost::shared_ptr<const gazebo_rad_msgs::msgs::RadiationObstacle> RadiationObstacleConstPtr;

  protected:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnUpdate(const common::UpdateInfo &);

  private:
    int photon_readout = 0;

    std::chrono::high_resolution_clock::time_point last_readout;
    std::uniform_real_distribution<double>         rand_dbl;
    std::mt19937                                   rand_gen;

    std::vector<Source> sources;
    std::mutex          sources_mutex;

    std::vector<Obstacle> obstacles;
    std::mutex            obstacles_mutex;

    bool terminated = false;

    double sensor_size;
    double sensor_thickness;
    double diagonal_length;
    double diagonal_absorption_prob_Cs137;
    double diagonal_absorption_prob_Am241;

    Material sensor_material;
    double   photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density);

    void updatePosition(ignition::math::Pose3d world_pose);
    void sourcesCallback(RadiationSourceConstPtr &msg);
    void obstaclesCallback(RadiationObstacleConstPtr &msg);

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
    transport::SubscriberPtr sources_sub;
    transport::SubscriberPtr obstacles_sub;

    boost::thread callback_queue_thread_;

    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::CallbackQueue               rosQueue;
    std::thread                      rosQueueThread;
    std::thread                      simThread;

    std::stringstream frame_name;

    ros::Publisher medipix_pub;

    gazebo_rad_msgs::msgs::RadiationSource radiation_msg;

    common::Time last_gps_time_;
    common::Time last_time_;

    std::string modelName;


    std::pair<int, std::chrono::high_resolution_clock::time_point> simulate(std::chrono::high_resolution_clock::time_point time_start);


    void   oneDebuggingRay();
    double obstacleAttenuation(Source s);  // percentage of particles which will get absorbed by obstacles

    BatchVisualizer          bv;
    tf::TransformBroadcaster transform_broadcaster;

    Cuboid sensor_cuboid;
  };

  GZ_REGISTER_MODEL_PLUGIN(Timepix)
  Timepix::Timepix() : ModelPlugin() {
  }

  Timepix::~Timepix() {
    ROS_INFO("[Timepix%u]: Terminating operation...", model_->GetId());
    terminated = true;

    // wait for simulation to finish
    simThread.join();

    // end all ROS related stuff
    rosNode->shutdown();

    rosQueueThread.join();
    callback_queue_thread_.join();

    // destroy connection to gazebo
    updateConnection_->~Connection();

    ROS_INFO("[Timepix%u]: Removed successfully", model_->GetId());
  }

  //}

  /* sourcesCallback() //{ */
  void Timepix::sourcesCallback(RadiationSourceConstPtr &msg) {
    if (msg->material() != "Cs137" && msg->material() != "Am241") {
      ROS_WARN("Unrecognized source material '%s', cannot simulate", msg->material().c_str());
      return;
    }

    Eigen::Vector3d pos(msg->x(), msg->y(), msg->z());

    sources_mutex.lock();
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
        sources_mutex.unlock();
        return;
      }
    }
    // new source -> compute params and add to list
    Eigen::Vector3d relative_position = world2local * (pos - pos3toVector3d(model_->WorldPose()));
    Source          s(msg->id(), msg->material(), msg->activity(), relative_position);
    s.last_contact = std::chrono::high_resolution_clock::now();
    sources.push_back(s);
    sources_mutex.unlock();
    ROS_INFO("[Timepix%u]: RadiationSource%u recognized", model_->GetId(), msg->id());
  }
  //}

  /* obstaclesCallback() //{ */
  void Timepix::obstaclesCallback(RadiationObstacleConstPtr &msg) {

    Eigen::Vector3d    obstacle_pos(msg->pos_x(), msg->pos_y(), msg->pos_z());
    Eigen::Quaterniond obstacle_ori(msg->ori_w(), msg->ori_x(), msg->ori_y(), msg->ori_z());

    double depth  = msg->scale_x();
    double width  = msg->scale_y();
    double height = msg->scale_z();

    obstacles_mutex.lock();
    for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
      // known obstacle -> update its params
      if (o->id == msg->id()) {
        Eigen::Quaterniond sensor_ori(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(),
                                      model_->WorldPose().Rot().Z());
        Eigen::Vector3d    relative_position    = world2local * (obstacle_pos - pos3toVector3d(model_->WorldPose()));
        Eigen::Quaterniond relative_orientation = world2local * obstacle_ori;

        o->center       = relative_position;
        o->orientation  = relative_orientation;
        o->cuboid       = Cuboid(relative_position, relative_orientation, depth, width, height);
        o->last_contact = std::chrono::high_resolution_clock::now();

        obstacles_mutex.unlock();
        return;
      }
    }
    // new obstacle -> compute params and add to list
    Eigen::Vector3d    relative_position = world2local * (obstacle_pos - pos3toVector3d(model_->WorldPose()));
    Eigen::Quaterniond relative_orientation(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(),
                                            model_->WorldPose().Rot().Z());

    Cuboid      c(relative_position, relative_orientation, depth, width, height);
    Material    obstacle_material;
    std::string material_name = msg->material();
    if (material_name == "Si") {
      obstacle_material = Si;
    } else if (material_name == "Cd") {
      obstacle_material = Cd;
    } else if (material_name == "Na") {
      obstacle_material = Na;
    } else if (material_name == "I") {
      obstacle_material = I;
    } else if (material_name == "Te") {
      obstacle_material = Te;
    } else if (material_name == "CdTe") {
      obstacle_material = CdTe;
    } else if (material_name == "concrete") {
      obstacle_material = concrete;
    } else if (material_name == "glass") {
      obstacle_material = glass;
    } else {
      ROS_INFO("[RadiationObstacle]: material '%s' not recognized. Using parameters from obstacle model.sdf", material_name.c_str());
      obstacle_material = Material(material_name, msg->density(), msg->mac60kev(), msg->mac600kev());
    }

    Obstacle o(c, relative_position, relative_orientation, obstacle_material);
    o.id           = msg->id();
    o.last_contact = std::chrono::high_resolution_clock::now();

    obstacles.push_back(o);
    obstacles_mutex.unlock();
    ROS_INFO("[Timepix%u]: Obstacle%u recognized", model_->GetId(), msg->id());
  }
  //}

  /* Sampling //{ */
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
    /* this->sensor_size      = 0.2; */
    /* this->sensor_thickness = 0.05; */

    this->diagonal_length = std::sqrt(2 * sensor_size * sensor_size + sensor_thickness * sensor_thickness);
    this->sensor_material = Si;

    this->diagonal_absorption_prob_Am241 = photoabsorptionProbability(diagonal_length, sensor_material.mac60kev, sensor_material.density);
    this->diagonal_absorption_prob_Cs137 = photoabsorptionProbability(diagonal_length, sensor_material.mac600kev, sensor_material.density);

    ROS_INFO("Probability of absorption on body diagonal: Cs137: %.4f, Am241: %.4f", diagonal_absorption_prob_Cs137, diagonal_absorption_prob_Am241);


    std::stringstream ss;
    ss << "/" << std::getenv("UAV_NAME") << "/timepix/photon_count";
    medipix_pub = rosNode->advertise<std_msgs::Int32>(ss.str().c_str(), 100);

    frame_name << "/fcu_" << std::getenv("UAV_NAME") << "/timepix_origin";
    bv = BatchVisualizer(*this->rosNode.get(), frame_name.str());

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

    sensor_cuboid = Cuboid(A, B, C, D, E, F, G, H);


    this->sources_sub   = node_handle_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
    this->obstacles_sub = node_handle_->Subscribe("~/radiation/obstacles", &Timepix::obstaclesCallback, this, 1);

    last_readout = std::chrono::high_resolution_clock::now();

    this->simThread      = std::thread(std::bind(&Timepix::SimulationThread, this));
    this->rosQueueThread = std::thread(std::bind(&Timepix::QueueThread, this));

    ROS_INFO("[Timepix]: initialized");
  }

  //}

  /* OnUpdate() //{ */

  void Timepix::OnUpdate(const common::UpdateInfo &) {
    ignition::math::Quaterniond q = model_->WorldPose().Rot();
    local2world                   = Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z());
    world2local                   = local2world.inverse();
  }
  //}

  /* simulate() //{ */
  std::pair<int, std::chrono::high_resolution_clock::time_point> Timepix::simulate(std::chrono::high_resolution_clock::time_point time_start) {
    std::pair<int, std::chrono::high_resolution_clock::time_point> ret;
    if (sources.size() < 1) {
      ret.first  = 0;
      ret.second = std::chrono::high_resolution_clock::now();
      return ret;
    }
    int required_photon_count = 0;

    int photons_captured = 0;
    int photons_total    = 0;

    /* auto time1 = std::chrono::high_resolution_clock::now(); */
    for (auto s = sources.begin(); s != sources.end();) {
      // Remove inactive sources
      if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - s->last_contact).count() > TIMEOUT) {
        ROS_INFO("[Timepix%u]: No longer tracking source%u. Reason: Timeout", model_->GetId(), s->id);
        sources.erase(s);
        continue;
      }
      /* auto time2 = std::chrono::high_resolution_clock::now(); */
      /* ROS_INFO_STREAM("Removing inactive sources: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time2 - time1).count() << " ns\n"); */

      double count_multiplier = obstacleAttenuation(*s);

      /* auto time3 = std::chrono::high_resolution_clock::now(); */
      /* ROS_INFO_STREAM("Calculating obstacle attenuation: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time3 - time2).count() << " ns\n"); */

      /* std::cout << "Time used: " << std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() << "\n"; */
      /* ROS_INFO("Count multiplier due to obstacles: %.3f", count_multiplier); */

      // VISUALIZE
      /* bv.addPoint(s->relative_position); */

      for (size_t i = 0; i < s->exposed_sides.size(); i++) {

        // VISUALIZE
        /* bv.addRect(sides[s->exposed_sides[i]]); */

        /* auto time4   = std::chrono::high_resolution_clock::now(); */
        int samples = 0;
        if (s->material == "Cs137") {
          samples = s->apparent_activities[i] * SIMULATION_STEP * diagonal_absorption_prob_Cs137;
        } else if (s->material == "Am241") {
          samples = s->apparent_activities[i] * SIMULATION_STEP * diagonal_absorption_prob_Am241;
        } else {
          ret.first  = 0;
          ret.second = std::chrono::high_resolution_clock::now();
          return ret;
        }
        /* auto time5 = std::chrono::high_resolution_clock::now(); */
        /* ROS_INFO_STREAM("Getting sample count: " << std::chrono::duration_cast<std::chrono::nanoseconds>(time5 - time4).count() << " ns\n"); */


        samples = samples * count_multiplier;
        required_photon_count += samples;


        /* ROS_INFO("Generating %d samples for side %d", samples, s->exposed_sides[i]); */
        for (int n = 0; n < samples; n++) {
          photons_total++;
          auto   time_now            = std::chrono::high_resolution_clock::now();
          auto   elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - time_start).count();
          double elapsed_ns          = (double)(elapsed_nanoseconds);
          if (elapsed_ns > SIMULATION_STEP * 1E9) {
            double elapsed_ns        = (double)(elapsed_nanoseconds);
            double theoretical_speed = (photons_total * 1E9) / elapsed_ns;
            ROS_INFO("Simulated particles: %d/%d, Captured: %d, Time: %.3f ms", photons_total, required_photon_count, photons_captured,
                     elapsed_nanoseconds / 1E6);
            ROS_INFO("Theoretical speed: %.2f particles per second", theoretical_speed);
            ROS_WARN("Forced interrupt");
            ret.first  = photons_captured;
            ret.second = time_now;
            return ret;
          }
          Eigen::Vector3d intersect1 = sampleSide(s->exposed_sides[i]);

          Ray r = Ray::twopointCast(s->relative_position, intersect1);

          for (int j = 0; j < 6; j++) {
            if (j == s->exposed_sides[i]) {
              continue;
            }
            boost::optional<Eigen::Vector3d> intersect2 = sides[j].intersectionRay(r);
            if (intersect2) {

              double track_length = (intersect2.get() - intersect1).norm();
              double pe_prob;
              if (s->material == "Cs137") {
                pe_prob = photoabsorptionProbability(track_length, sensor_material.mac600kev, sensor_material.density) / diagonal_absorption_prob_Cs137;
              } else if (s->material == "Am241") {
                pe_prob = photoabsorptionProbability(track_length, sensor_material.mac60kev, sensor_material.density) / diagonal_absorption_prob_Am241;
              }
              double coin_flip = rand_dbl(rand_gen);
              if (coin_flip < pe_prob) {
                photons_captured++;

                // VISUALIZE
                /* Ray track = Ray::twopointCast(intersect1, intersect2.get()); */
                /* bv.addRay(track, 1.0, 0.5, 0.0); */
                /* if (photons_captured < 5) { */
                bv.addRay(r);
                /* } */
              }
              break;
            }
          }
        }
      }
      s++;
    }
    auto   time_now            = std::chrono::high_resolution_clock::now();
    auto   elapsed_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - time_start).count();
    double elapsed_ns          = (double)(elapsed_nanoseconds);
    double theoretical_speed   = (photons_total * 1E9) / elapsed_ns;
    ROS_INFO("Simulated particles: %d/%d, Captured: %d, Time: %.3f ms", photons_total, required_photon_count, photons_captured, elapsed_nanoseconds / 1E6);
    ROS_INFO("Theoretical speed: %.2f particles per second", theoretical_speed);

    /* VISUALIZE //{ */
    /* bv.publish(); */
    /* //} VISUALIZE */
    ret.first  = photons_captured;
    ret.second = time_now;
    return ret;
  }
  //}

  /* photoabsorptionProbability //{*/
  double Timepix::photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density) {
    return 1.0 - std::exp(-mass_att_coeff * material_thickness * 100 * material_density);  // multiply by 100 to get thickness in cm
  }
  //}

  /* obstacleAttenuation //{ */
  // return percentage of particles which will survive their way through obstacles and air
  double Timepix::obstacleAttenuation(Source s) {
    obstacles_mutex.lock();
    Ray r = Ray::twopointCast(s.relative_position, Eigen::Vector3d(0.0, 0.0, 0.0));
    /* ROS_INFO("[Timepix]: Casting ray"); */
    double ret                  = 1.0;
    double total_obstacle_track = 0.0;
    int    obstacles_in_way     = 0;
    for (Obstacle o : obstacles) {
      std::vector<Eigen::Vector3d> intersections;
      // TODO get intersection with all sides
      for (int i = 0; i < 6; i++) {
        boost::optional<Eigen::Vector3d> intersect = o.cuboid.sides[i].intersectionRay(r);
        if (intersect) {
          intersections.push_back(*intersect);
        }
      }
      if (intersections.size() > 1) {
        double track_length   = (intersections[1] - intersections[0]).norm();
        Ray    obstacle_track = Ray::twopointCast(intersections[1], intersections[0]);
        /* bv.addRay(obstacle_track, 1.0, 1.0, 0.3); */
        if (s.material == "Cs137") {
          ret *= (1 - photoabsorptionProbability(track_length, o.material.mac600kev, o.material.density));
        } else if (s.material == "Am241") {
          ret *= (1 - photoabsorptionProbability(track_length, o.material.mac60kev, o.material.density));
        }
        obstacles_in_way++;
        total_obstacle_track += track_length;
        /* ROS_INFO("Timepix: Obstacle track length: %.3f", track_length); */
      }
    }
    if (s.material == "Cs137") {
      ret *= (1 - photoabsorptionProbability(s.relative_position.norm() - total_obstacle_track, air.mac600kev, air.density));
    } else if (s.material == "Am241") {
      ret *= (1 - photoabsorptionProbability(s.relative_position.norm() - total_obstacle_track, air.mac600kev, air.density));
    }
    /* ROS_INFO("[Timepix%u]: Obstacles in way: %d, Total obstacle track: %.2f, Environ particle loss: %.2f", model_->GetId(), obstacles_in_way, */
    /*          total_obstacle_track, (1 - ret)); */
    obstacles_mutex.unlock();
    return ret;
  }
  //}
}  // namespace gazebo
