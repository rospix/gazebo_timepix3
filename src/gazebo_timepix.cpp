#include <gazebo_timepix/gazebo_timepix.h>
#include <gazebo_timepix/python_wrappers.h>
#include <algorithm>

// index sides of the detector for convenience
#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define BOTTOM 4
#define TOP 5

#define BLUE Eigen::Vector4d(0.2, 0.2, 0.9, 0.7)
#define ORANGE Eigen::Vector4d(0.9, 0.6, 0.2, 0.7)
#define RED Eigen::Vector4d(1.0, 0.2, 0.2, 1.0)
#define GREEN Eigen::Vector4d(0.2, 1.0, 0.2, 1.0)

using namespace gazebo;

/* Destructor //{ */
Timepix::~Timepix() {
  terminated = true;
  simulation_thread.join();
  ROS_INFO("[Timepix%u]: Plugin terminated", model_->GetId());
}
//}

/* Load //{ */
void Timepix::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  /* init local variables //{ */
  rand_dbl = std::uniform_real_distribution<double>(0, 1);
  model_   = _model;
  world_   = _model->GetWorld();
  frame_name << model_->GetName().c_str() << "/timepix_origin";

  transport_node_ = transport::NodePtr(new transport::Node());
  transport_node_->Init();

  // ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);
  this->ros_node.reset(new ros::NodeHandle("~"));

  // init batch visualizer for rviz
  bv = BatchVisualizer(*ros_node.get(), frame_name.str());
  //}

  /* parse sdf params //{ */
  if (_sdf->HasElement("exposition_time")) {
    exposition_time = _sdf->Get<double>("exposition_time");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'exposition_time' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("sensor_size")) {
    sensor_size = _sdf->Get<double>("sensor_size");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'sensor_size' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("sensor_thickness")) {
    sensor_thickness = _sdf->Get<double>("sensor_thickness");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'sensor_thickness' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("material")) {
    material = _sdf->Get<std::string>("material");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'material' was not specified", model_->GetId());
  }
  //}

  diagonal_length = std::sqrt(2 * sensor_size * sensor_size + sensor_thickness * sensor_thickness);

  // init sensor publisher
  std::stringstream ss;
  ss << "/" << model_->GetName().c_str() << "/timepix/photon_count";
  timepix_pub = ros_node->advertise<gazebo_rad_msgs::Timepix>(ss.str().c_str(), 100);

  /* build sensor cuboid //{ */
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
  //}

  // subscribe to gazebo topics
  sources_sub     = transport_node_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
  obstacles_sub   = transport_node_->Subscribe("~/radiation/obstacles", &Timepix::obstaclesCallback, this, 1);
  termination_sub = transport_node_->Subscribe("~/radiation/termination", &Timepix::terminationCallback, this, 1);

  ROS_INFO("[Timepix%u]: Plugin initialized", model_->GetId());
  ROS_INFO("[Timepix%u]: Material: %s", model_->GetId(), material.c_str());

  terminated        = false;
  simulation_thread = boost::thread(boost::bind(&Timepix::SimulationThread, this));
}
//}

/* SimulationThread //{ */
void Timepix::SimulationThread() {
  while (!terminated) {
    auto start_time = std::chrono::high_resolution_clock::now();

    /* Relative geometry handler //{ */
    tf::Transform  transform;
    tf::Vector3    origin(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
    tf::Quaternion quat;
    quat.setW(model_->WorldPose().Rot().W());
    quat.setX(model_->WorldPose().Rot().X());
    quat.setY(model_->WorldPose().Rot().Y());
    quat.setZ(model_->WorldPose().Rot().Z());
    transform.setOrigin(origin);
    transform.setRotation(quat);
    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin", frame_name.str()));
    //}

    bv.clear();
    int photons = simulate();
    /* bv.addCuboid(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d(sensor_thickness, sensor_size, sensor_size), BLUE); */

    /* /1* Environment visualization //{ *1/ */
    /* obstacles_mutex.lock(); */
    /* for (auto o = obstacles.begin(); o != obstacles.end(); o++) { */
    /*   bv.addCuboid(o->getRelativePosition(), o->getRelativeOrientation(), o->getScale(), ORANGE); */
    /* } */
    /* obstacles_mutex.unlock(); */
    /* sources_mutex.lock(); */
    /* for (auto s = sources.begin(); s != sources.end(); s++) { */
    /*   bv.addPoint(s->getRelativePosition(), ORANGE); */
    /* } */
    /* sources_mutex.unlock(); */
    /* bv.publish(); */
    /* //} */

    /* build msg //{ */
    gazebo_rad_msgs::Timepix msg;
    msg.id       = model_->GetId();
    msg.count    = photons;
    msg.exposure = exposition_time;
    msg.size.x   = sensor_thickness;
    msg.size.y   = sensor_size;
    msg.size.z   = sensor_size;
    msg.stamp    = ros::Time::now();
    //}

    timepix_pub.publish(msg);

    auto   end_time   = std::chrono::high_resolution_clock::now();
    double dt         = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1E9;
    double sleep_time = exposition_time - dt;
    std::cout << "Simulation took " << dt << " seconds\n";
    std::cout << "Captured " << photons << " photons (flux: " << photons / exposition_time << " photons/sec)\n";
    if (sleep_time > 0.0) {
      std::cout << "Sleeping for: " << sleep_time << " seconds\n";
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
  }
}
//}

/* simulate //{ */
int Timepix::simulate() {
  int required_photon_count = 0;
  int photons_captured      = 0;
  for (auto s = sources.begin(); s != sources.end(); s++) {
    // TODO obstacle attenuation
    double count_multiplier = obstacleAttenuation(*s);
    /* std::cout << "Environment attenuation: " << 100 * (1 - count_multiplier) << "\%\n"; */
    for (size_t i = 0; i < s->getApparentActivities().size(); i++) {
      int samples = (int)(s->getApparentActivities()[i] * exposition_time * s->getDiagonalAbsorptionProb() * count_multiplier);
      required_photon_count += samples;
      /* std::cout << "For apparent activity " << s->getApparentActivities()[i] << " simulating " << samples << " photons\n"; */
      for (int n = 0; n < samples; n++) {
        Eigen::Vector3d intersect1 = sampleSide(s->getExposedSides()[i]);

        Ray r = Ray::twopointCast(s->getRelativePosition(), intersect1);

        for (int j = 0; j < 6; j++) {
          if (j == s->getExposedSides()[i]) {
            continue;
          }
          boost::optional<Eigen::Vector3d> intersect2 = sides[j].intersectionRay(r);

          double pe_prob = 0.0;
          if (intersect2) {
            double track_length = (intersect2.get() - intersect1).norm();
            pe_prob             = photoabsorptionProbability(track_length, s->getTimepixPhotoAbsorptionCoeff(), s->getTimepixDensity());
            /* std::cout << "Capture probability: " << pe_prob << "\n"; */

            double coin_flip = rand_dbl(rand_gen);
            if (coin_flip < pe_prob) {
              bv.addRay(r, GREEN, 0.0002);
              photons_captured++;
            } else {
              bv.addRay(r, RED, 0.0002);
            }
            break;
          }
        }
      }
    }
  }
  std::cout << "Simulated " << required_photon_count << " photons\n";
  return photons_captured;
}
//}

/* sourcesCallback //{ */
void Timepix::sourcesCallback(RadiationSourceConstPtr &msg) {
  bool new_source = true;
  for (auto s = sources.begin(); s != sources.end(); s++) {
    if (s->getId() == msg->id()) {
      sources_mutex.lock();
      sources.erase(s);
      sources_mutex.unlock();
      new_source = false;
      break;
    }
  }
  if (new_source) {
    ROS_INFO("[Timepix%u]: Registered RadiationSource%u", model_->GetId(), msg->id());
  }
  Eigen::Quaterniond local2world =
      Eigen::Quaterniond(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z());
  Eigen::Quaterniond world2local = local2world.inverse();

  Eigen::Vector3d     relative_position = world2local * (Eigen::Vector3d(msg->x(), msg->y(), msg->z()) - pos3toVector3d(model_->WorldPose()));
  std::vector<double> apparent_activities;
  std::vector<int>    exposed_sides;
  for (size_t i = 0; i < sides.size(); i++) {
    if (sides[i].normal_vector.dot(relative_position) > 0) {
      double apparent_activity = (msg->activity() / 4 * M_PI) * rectSolidAngle(sides[i], relative_position);
      apparent_activities.push_back(apparent_activity);
      exposed_sides.push_back(i);
    }
  }

  // TODO update pres python approximator moc zpomaluje simulaci
  /* std::vector<double> timepix_material_properties   = getMaterialProperties(material, msg->energy()); */
  /* double              timepix_photoabsorption_coeff = timepix_material_properties[0]; */
  /* double              timepix_density               = timepix_material_properties[1]; */

  /* std::vector<double> air_material_properties   = getMaterialProperties("air", msg->energy()); */
  /* double              air_photoabsorption_coeff = air_material_properties[0]; */
  /* double              air_density               = air_material_properties[1]; */
  /* double              diagonal_absorption_prob  = diagonal_length * timepix_photoabsorption_coeff; */

  // DEBUG
  double timepix_photoabsorption_coeff = 1.0;
  double timepix_density               = 3.0;
  double air_photoabsorption_coeff     = 1.0;
  double air_density                   = 0.03;
  double diagonal_absorption_prob      = diagonal_length * timepix_photoabsorption_coeff;
  // DEBUG

  Source s(msg->id(), msg->material(), msg->activity(), msg->energy(), timepix_density, relative_position, apparent_activities, exposed_sides,
           timepix_photoabsorption_coeff, diagonal_absorption_prob, air_photoabsorption_coeff, air_density);

  sources_mutex.lock();
  sources.push_back(s);
  sources_mutex.unlock();
}
//}

/* obstaclesCallback //{ */
void Timepix::obstaclesCallback(RadiationObstacleConstPtr &msg) {
  bool new_obstacle = true;
  for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
    if (o->getId() == msg->id()) {
      obstacles_mutex.lock();
      obstacles.erase(o);
      obstacles_mutex.unlock();
      new_obstacle = false;
      break;
    }
  }
  if (new_obstacle) {
    ROS_INFO("[Timepix%u]: Registered RadiationObstacle%u", model_->GetId(), msg->id());
  }
  // store relative pose of the obstacle
  Eigen::Quaterniond local2world =
      Eigen::Quaterniond(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z());
  Eigen::Quaterniond world2local          = local2world.inverse();
  Eigen::Quaterniond relative_orientation = world2local * Eigen::Quaterniond(msg->ori_w(), msg->ori_x(), msg->ori_y(), msg->ori_z());
  Eigen::Vector3d    relative_position    = world2local * (Eigen::Vector3d(msg->pos_x(), msg->pos_y(), msg->pos_z()) - pos3toVector3d(model_->WorldPose()));
  Eigen::Vector3d    obstacle_scale(msg->scale_x(), msg->scale_y(), msg->scale_z());

  Obstacle o(msg->id(), msg->material(), relative_position, relative_orientation, obstacle_scale);
  obstacles_mutex.lock();
  obstacles.push_back(o);
  obstacles_mutex.unlock();
}
//}

/* terminationCallback //{ */
void Timepix::terminationCallback(TerminationConstPtr &msg) {
  sources_mutex.lock();
  for (auto it = sources.begin(); it != sources.end(); it++) {
    if (it->getId() == msg->id()) {
      sources.erase(it);
      ROS_INFO("[Timepix%u]: No longer tracking RadiationSource%u", model_->GetId(), msg->id());
      sources_mutex.unlock();
      return;
    }
  }
  sources_mutex.unlock();
  obstacles_mutex.lock();
  for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
    if (it->getId() == msg->id()) {
      obstacles.erase(it);
      ROS_INFO("[Timepix%u]: No longer tracking RadiationObstacle%u", model_->GetId(), msg->id());
      obstacles_mutex.unlock();
      return;
    }
  }
  obstacles_mutex.unlock();
}
//}

/* sampleRectangle //{ */
Eigen::Vector3d Timepix::sampleRectangle(Rectangle r) {

  double k1 = rand_dbl(rand_gen);
  double k2 = rand_dbl(rand_gen);

  return r.points[0] + k1 * r.basis.col(0) + k2 * r.basis.col(1);
}
//}

/* sampleSide //{ */
Eigen::Vector3d Timepix::sampleSide(int index) {

  double k1 = rand_dbl(rand_gen);
  double k2 = rand_dbl(rand_gen);

  return sides[index].points[0] + k1 * sides[index].basis.col(0) + k2 * sides[index].basis.col(1);
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
  Ray    r                    = Ray::twopointCast(s.getRelativePosition(), Eigen::Vector3d::Zero());
  double ret                  = 1.0;
  double total_obstacle_track = 0.0;
  int    obstacles_in_way     = 0;
  for (Obstacle o : obstacles) {
    std::vector<Eigen::Vector3d> intersections;
    for (int i = 0; i < 6; i++) {
      boost::optional<Eigen::Vector3d> intersect = o.getSides()[i].intersectionRay(r);
      if (intersect) {
        if ((s.getRelativePosition() - *intersect).norm() < s.getRelativePosition().norm()) {
          intersections.push_back(*intersect);
        } else {
          break;
        }
      }
    }
    if (intersections.size() > 1) {
      double              track_length          = (intersections[1] - intersections[0]).norm();
      Ray                 obstacle_track        = Ray::twopointCast(intersections[1], intersections[0]);
      std::vector<double> material_properties   = getMaterialProperties(material, s.getEnergy());
      double              photoabsorption_coeff = material_properties[0];
      double              density               = material_properties[1];
      ret *= (1 - photoabsorptionProbability(track_length, photoabsorption_coeff, density));
      obstacles_in_way++;
      total_obstacle_track += track_length;
    }
  }
  ret *= (1 - photoabsorptionProbability(s.getRelativePosition().norm() - total_obstacle_track, s.getAirPhotoAbsorptionCoeff(), s.getAirDensity()));
  obstacles_mutex.unlock();
  return ret;
}
//}

//}
