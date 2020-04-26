#include <gazebo_timepix/gazebo_timepix.h>

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
Timepix::~Timepix() {

  // shutdown subscribers
  sources_sub->Unsubscribe();
  obstacles_sub->Unsubscribe();
  termination_sub->Unsubscribe();

  // inform other gazebo nodes
  gazebo_rad_msgs::msgs::Termination msg;
  msg.set_id(model_->GetId());


  // terminate
  terminated = true;
  publisher_thread.join();
  ROS_INFO("[Timepix%u]: Plugin terminated", model_->GetId());
}
//}

/* Load //{ */
void Timepix::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  /* parse sdf params //{ */
  if (_sdf->HasElement("material")) {
    material = _sdf->Get<std::string>("material");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'material' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("size")) {
    size = _sdf->Get<ignition::math::Vector3d>("size");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'size' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("exposition_time")) {
    exposition_seconds = _sdf->Get<double>("exposition_time");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'publish_rate' was not specified", model_->GetId());
  }
  //}

  // init local variables
  model_ = _model;
  buildSensorCuboid();
  local_frame << model_->GetName().c_str() << "/timepix_origin";
  global_frame << "local_origin";

  rand_dbl = std::uniform_real_distribution<double>(0, 1);

  // init gazebo node
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_timepix", ros::init_options::NoSigintHandler);
  ros_node = ros::NodeHandle("~");

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Timepix::onWorldUpdate, this, _1));

  // gazebo communication
  sources_sub     = gazebo_node_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
  obstacles_sub   = gazebo_node_->Subscribe("~/radiation/obstacles", &Timepix::obstaclesCallback, this, 1);
  termination_sub = gazebo_node_->Subscribe("~/radiation/termination", &Timepix::terminationCallback, this, 1);

  // ros communication
  ros_publisher         = ros_node.advertise<gazebo_rad_msgs::Timepix>("/radiation/timepix", 1);
  diagnostics_publisher = ros_node.advertise<gazebo_rad_msgs::TimepixDiagnostics>("/radiation/diagnostics", 1);

  terminated       = false;
  publisher_thread = boost::thread(boost::bind(&Timepix::PublisherLoop, this));
  ROS_INFO("[Timepix%u]: Plugin initialized", model_->GetId());

  /* bv = BatchVisualizer(ros_node, "visualization", global_frame.str()); */
  debug_visualizer = BatchVisualizer(ros_node, "debug_visualizer", global_frame.str());
  debug_visualizer.setPointsScale(0.7);
}
//}

/* sourcesCallback //{ */
void Timepix::sourcesCallback(RadiationSourceConstPtr &msg) {

  sources_mutex.lock();

  /* Check whether the source is already registered //{ */
  for (auto source = sources.begin(); source != sources.end(); source++) {

    if (source->getId() == msg->id()) {
      Eigen::Vector3d source_world_pos(msg->x(), msg->y(), msg->z());
      source->setRelativePosition(targetRelativePosition(model_->WorldPose(), source_world_pos));
      source->setSideProperties(calculateSideProperties(*source));
      std::vector<double> apparent_activities;
      for (auto side_triplet = source->getSideProperties().begin(); side_triplet != source->getSideProperties().end(); side_triplet++) {
        apparent_activities.push_back(side_triplet->apparent_activity);
      }
      /* std::cout << "Source" << msg->id() << ": apparent activities:\n"; */
      /* for (unsigned int i = 0; i < apparent_activities.size(); i++) { */
      /* std::cout << "     " << apparent_activities[i] << "\n"; */
      /* } */
      sources_mutex.unlock();
      return;
    }
  }
  //}

  /* Handle newly registered source //{ */
  ROS_INFO("[Timepix%u]: Newly registered RadiationSource%u", model_->GetId(), msg->id());
  Eigen::Vector3d source_world_pos(msg->x(), msg->y(), msg->z());

  double mass_att_coeff            = calculateMassAttCoeff(msg->energy(), material, AttenuationType::MASS_ENERGY);
  double diagnonal_absorption_prob = calculateAbsorptionProb(diagonal_length, mass_att_coeff, density);

  SourceAbstraction s(msg->id(), msg->material(), msg->activity(), msg->energy(), mass_att_coeff, diagnonal_absorption_prob,
                      targetRelativePosition(model_->WorldPose(), source_world_pos));
  s.setSideProperties(calculateSideProperties(s));
  sources.push_back(s);
  //}

  sources_mutex.unlock();
}
//}

/* obstaclesCallback //{ */
void Timepix::obstaclesCallback(RadiationObstacleConstPtr &msg) {

  obstacles_mutex.lock();

  /* Check whether the obstacle is already registered //{ */
  for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++) {
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
      obstacles_mutex.unlock();
      return;
    }
  }
  //}

  /* Handle newly registered obstacle //{ */
  ROS_INFO("[Timepix%u]: Newly registered RadiationObstacle%u", model_->GetId(), msg->id());

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
  obstacles.push_back(o);
  //}

  obstacles_mutex.unlock();
}
//}

/* terminateCallback //{ */
void Timepix::terminationCallback(TerminationConstPtr &msg) {

  /* terminate source //{ */
  sources_mutex.lock();
  unsigned int sources_count = sources.size();
  sources.erase(std::remove(sources.begin(), sources.end(), msg->id()), sources.end());
  if (sources.size() != sources_count) {
    ROS_INFO("[Timepix%u]: No longer tracking RadiationSource%u", model_->GetId(), msg->id());
    for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
      it->removeSource(msg->id());
    }
    sources_mutex.unlock();
    return;
  }
  sources_mutex.unlock();
  //}

  /* terminate obstacle //{ */
  obstacles_mutex.lock();
  unsigned int obstacles_count = obstacles.size();
  obstacles.erase(std::remove(obstacles.begin(), obstacles.end(), msg->id()), obstacles.end());
  if (obstacles_count != obstacles.size()) {
    ROS_INFO("[Timepix%u]: No longer tracking RadiationObstacle%u", model_->GetId(), msg->id());
    obstacles_mutex.unlock();
    return;
  }
  obstacles_mutex.unlock();
  //}
}
//}

/* Simulate //{ */
ros::Time Timepix::Simulate(ros::Time sim_start) {
  int photons_captured = 0;
  int rays_cast        = 0;

  for (auto source = sources.begin(); source != sources.end(); source++) {
    // get num of photons to be simulated
    int lossles_num_photons = source->getSideProperties()[FRONT].apparent_activity;
    // trace obstacles
    Eigen::Vector3d source_pos               = source->getRelativePosition();
    Ray             r                        = Ray::twopointCast(Eigen::Vector3d::Zero(), source_pos);
    double          environment_transparency = 1.0;

    for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
    }
  }

  /* for (auto o = obstacles.begin(); o != obstacles.end(); o++) { */
  /*   bv.addCuboid(o->getRelativeCuboid(), BROWN, false); */
  /* } */

  /* for (auto s = sources.begin(); s != sources.end(); s++) { */
  /*   /1*   // TODO obstacle attenuation *1/ */

  /*   bv.addPoint(s->getRelativePosition(), GREEN); */
  /*   bv.addRay(Ray::twopointCast(Eigen::Vector3d::Zero(), s->getRelativePosition())); */
  /*   for (auto side_properties = s->getSideProperties().begin(); side_properties != s->getSideProperties().end(); side_properties++) { */

  /*     int side_index = side_properties->side_index; */
  /*     bv.addRectangle(sides[side_index], GRAY, true); */
  /* //{ */

  /* int samples = (int)(side_properties->apparent_activity * exposition_seconds * s->getDiagonalAbsorptionProb()); */

  /* for (int n = 0; n < samples; n++) { */
  /*   ros::Time curr_time = ros::Time::now(); */
  /*   auto      time_diff = (curr_time - sim_start).toSec(); */
  /*   if (time_diff >= exposition_seconds) { */
  /*     ROS_WARN("[Timepix%u]: Time slip detected. Simulation incomplete", model_->GetId()); */
  /*     publishSensorMsg(photons_captured); */
  /*     return ros::Time::now(); */
  /*   } */

  /*   Eigen::Vector3d intersect1 = sampleRectangle(sides[side_index]); */

  /*   Ray r = Ray::twopointCast(s->getRelativePosition(), intersect1); */
  /*   rays_cast++; */
  /*   for (int j = 0; j < 6; j++) { */
  /*     if (j == side_index) { */
  /*       continue; */
  /*     } */
  /*     Triangle t1 = sides[j].triangles()[0]; */
  /*     Triangle t2 = sides[j].triangles()[1]; */

  /*     boost::optional<Eigen::Vector3d> intersect2 = t1.intersectionRay(r); */
  /*     if(intersect2 == boost::none){ */
  /*       intersect2 = t2.intersectionRay(r); */
  /*     } */
  /*     double                           pe_prob; */
  /*     if (intersect2) { */
  /*       double track_length   = (intersect2.get() - intersect1).norm(); */
  /*       double mass_att_coeff = s->getMassAttCoeff(); */
  /*       pe_prob               = calculateAbsorptionProb(track_length, mass_att_coeff, density) / s->getDiagonalAbsorptionProb(); */

  /*       /1* std::cout << "pe_prob: " << (pe_prob * 100) << "\%\n"; *1/ */
  /*       double coin_flip = rand_dbl(rand_gen); */
  /*       if (coin_flip < pe_prob) { */
  /*         bv.addRay(r, GREEN); */
  /*         photons_captured++; */
  /*       } else { */
  /*         bv.addRay(r, ORANGE); */
  /*       } */
  /*       break; */
  /*     } */
  /*   } */
  //}
  /* } */
  /* } */
  /* auto sim_end      = ros::Time::now(); */
  /* auto sim_duration = (sim_end - sim_start).toSec(); */
  /* std::cout << "simulation duration: " << (1000 * sim_duration) << " ms\n"; */
  /* ROS_INFO("[Timepix%u]: Rays cast: %d, Capture percentage: %.3f%, Simulation speed: %.3f rays/sec", model_->GetId(), rays_cast, */
  /* (100.0 * photons_captured) / rays_cast, rays_cast / sim_duration_sec.count()); */
  /* publishSensorMsg(photons_captured); */
  return ros::Time::now();
}
//}

/* PublisherLoop //{ */
void Timepix::PublisherLoop() {
  while (!terminated) {
    auto sim_start = ros::Time::now();
    auto sim_end   = Simulate(sim_start);

    /* RVIZ visualization (ROS message) //{ */
    gazebo_rad_msgs::Timepix debug_msg;
    debug_msg.material = material;
    debug_msg.exposure = exposition_seconds;
    debug_msg.count    = 0;
    debug_msg.id       = model_->GetId();
    debug_msg.size.x   = size[0];
    debug_msg.size.y   = size[1];
    debug_msg.size.z   = size[2];
    debug_msg.stamp    = ros::Time::now();
    ros_publisher.publish(debug_msg);
    //}

    /* diagnostics (ROS message) //{ */
    publishDiagnostics();
    //}

    auto sim_duration = sim_end - sim_start;
    /* ROS_INFO("[Timepix%u]: Simulation took %d nsec", model_->GetId(), sim_duration.nsec); */
    (ros::Duration(exposition_seconds) - sim_duration).sleep();
  }
}
//}

/* calculateSideProperties //{ */
std::vector<Triplet> Timepix::calculateSideProperties(SourceAbstraction s) {
  std::vector<Triplet> ret;
  for (int i = 0; i < 6 && ret.size() <= 3; i++) {
    Eigen::Vector3d side_normal = (sides[i].b() - sides[i].a()).cross(sides[i].d() - sides[i].a());
    if (side_normal.dot(s.getRelativePosition()) > 0) {
      double solid_angle = geometry::rectSolidAngle(sides[i], s.getRelativePosition());
      if (solid_angle == solid_angle) {  // NaN check
        double  apparent_activity = (s.getActivity() / 4 * M_PI) * solid_angle;
        Triplet triplet;
        triplet.side_index                = i;
        triplet.apparent_activity         = apparent_activity;
        triplet.diagnonal_absorption_prob = calculateAbsorptionProb(diagonal_length, s.getMassAttCoeff(), density);
        ret.push_back(triplet);
      }
    }
  }
  return ret;
}
//}

/* buildSensorCuboid //{ */
void Timepix::buildSensorCuboid() {
  for (int i = 0; i < 6; i++) {
    sides.push_back(Rectangle());
  }
  Eigen::Vector3d A(size[0] / 2.0, -size[1] / 2.0, -size[2] / 2.0);
  Eigen::Vector3d B(size[0] / 2.0, size[1] / 2.0, -size[2] / 2.0);
  Eigen::Vector3d C(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0);
  Eigen::Vector3d D(size[0] / 2.0, -size[1] / 2.0, size[2] / 2.0);

  Eigen::Vector3d E(-size[0] / 2.0, size[1] / 2.0, -size[2] / 2.0);
  Eigen::Vector3d F(-size[0] / 2.0, -size[1] / 2.0, -size[2] / 2.0);
  Eigen::Vector3d G(-size[0] / 2.0, -size[1] / 2.0, size[2] / 2.0);
  Eigen::Vector3d H(-size[0] / 2.0, size[1] / 2.0, size[2] / 2.0);

  sides[FRONT]    = Rectangle(A, B, C, D);
  sides[BACK]     = Rectangle(E, F, G, H);
  sides[LEFT]     = Rectangle(B, E, H, C);
  sides[RIGHT]    = Rectangle(F, A, D, G);
  sides[BOTTOM]   = Rectangle(F, E, B, A);
  sides[TOP]      = Rectangle(D, C, H, G);
  diagonal_length = (C - F).norm();
}
//}

/* traceObstaclesAttenuation //{ */
double Timepix::traceObstaclesAttenuation(SourceAbstraction sa) {
  Eigen::Vector3d source_position = sa.getRelativePosition();

  Ray r = Ray::twopointCast(Eigen::Vector3d::Zero(), source_position);

  double transmission = 1.0;
  for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
    std::vector<Eigen::Vector3d> intersections = o->getRelativeCuboid().intersectionRay(r);
    if (intersections.size() > 1) {
      if (source_position.norm() > intersections[0].norm() && source_position.norm() > intersections[1].norm()) {
        transmission *= calculateMassAttCoeff(sa.getEnergy(), material, AttenuationType::MASS_ENERGY);
      }
    }
  }
  return 1.0 - transmission;
}
//}

/* traceObstaclesId //{ */
std::vector<unsigned int> Timepix::traceObstaclesId(SourceAbstraction sa) {
  std::vector<unsigned int> ret;

  Eigen::Vector3d source_position = sa.getRelativePosition();

  Ray r = Ray::twopointCast(Eigen::Vector3d::Zero(), source_position);

  for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
    std::vector<Eigen::Vector3d> intersections = o->getRelativeCuboid().intersectionRay(r);
    if (intersections.size() > 1) {
      if (source_position.norm() > intersections[0].norm() && source_position.norm() > intersections[1].norm()) {
        ret.push_back(o->getId());

        // DEBUGOŇ
        debug_visualizer.addPoint(intersections[0], GREEN);
        debug_visualizer.addPoint(intersections[1], ORANGE);
        debug_visualizer.addRay(r);
      }
    }
  }
  return ret;
}
//}

/* onWorldUpdate //{ */
void Timepix::onWorldUpdate([[maybe_unused]] const common::UpdateInfo &upd) {
  tf::Transform  transform;
  tf::Quaternion quat(model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z(), model_->WorldPose().Rot().W());
  tf::Vector3    origin(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
  transform.setOrigin(origin);
  transform.setRotation(quat);
  transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), global_frame.str(), local_frame.str()));
}
//}

/* sampleRectangle //{ */
Eigen::Vector3d Timepix::sampleRectangle(Rectangle r) {

  double k1 = rand_dbl(rand_gen);
  double k2 = rand_dbl(rand_gen);

  return r.a() + k1 * (r.b() - r.a()) + k2 * (r.c() - r.a());
}
//}

/* publishSensorMsg //{ */
void Timepix::publishSensorMsg(int particle_count) {
  gazebo_rad_msgs::Timepix msg;
  msg.stamp    = ros::Time::now();
  msg.id       = model_->GetId();
  msg.material = material;
  msg.size.x   = size.X();
  msg.size.y   = size.Y();
  msg.size.z   = size.Z();
  msg.exposure = exposition_seconds;
  msg.count    = particle_count;

  ros_publisher.publish(msg);
}
//}

/* publishDiagnostics //{ */
void Timepix::publishDiagnostics() {

  gazebo_rad_msgs::TimepixDiagnostics msg;
  msg.stamp           = ros::Time::now();
  msg.gazebo_id       = model_->GetId();
  msg.exposition_time = exposition_seconds;
  msg.material        = material;
  msg.size.x          = size.X();
  msg.size.y          = size.Y();
  msg.size.z          = size.Z();
  msg.world_pos.x     = model_->WorldPose().Pos().X();
  msg.world_pos.y     = model_->WorldPose().Pos().Y();
  msg.world_pos.z     = model_->WorldPose().Pos().Z();
  msg.world_rot.w     = model_->WorldPose().Rot().W();
  msg.world_rot.x     = model_->WorldPose().Rot().X();
  msg.world_rot.y     = model_->WorldPose().Rot().Y();
  msg.world_rot.z     = model_->WorldPose().Rot().Z();

  /* add sources //{ */
  for (auto s = sources.begin(); s != sources.end(); s++) {
    gazebo_rad_msgs::SourceDiagnostics sd;
    sd.id             = s->getId();
    sd.activity       = s->getActivity();
    sd.energy         = s->getEnergy();
    sd.material       = s->getMaterial();
    sd.relative_pos.x = s->getRelativePosition().x();
    sd.relative_pos.y = s->getRelativePosition().y();
    sd.relative_pos.z = s->getRelativePosition().z();

    int i = 0;
    for (auto sides = s->getSideProperties().begin(); sides != s->getSideProperties().end(); sides++) {
      if (i == 0) {
        sd.exposed_sides.x       = sides->side_index;
        sd.apparent_activities.x = sides->apparent_activity;
      } else if (i == 1) {
        sd.exposed_sides.y       = sides->side_index;
        sd.apparent_activities.y = sides->apparent_activity;
      } else {
        sd.exposed_sides.z       = sides->side_index;
        sd.apparent_activities.z = sides->apparent_activity;
      }
      i++;
    }
    msg.sources.push_back(sd);
  }
  //}

  /* add obstacles //{ */
  for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
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

  diagnostics_publisher.publish(msg);
  debugVisualize();
}
//}

/* debugVisualize //{ */
void Timepix::debugVisualize() {
  debug_visualizer.clearBuffers();
  debug_visualizer.clearVisuals();

  // draw sensor
  for (unsigned int i = 0; i < sides.size(); i++) {
    debug_visualizer.addRectangle(sides[i], BLUE, true);
    debug_visualizer.addRectangle(sides[i], BLACK, false);
  }

  // draw registered sources
  for (unsigned int i = 0; i < sources.size(); i++) {
    debug_visualizer.addPoint(sources[i].getRelativePosition(), GREEN);

    // draw participating obstacles
    auto participating_obstacles = traceObstaclesId(sources[i]);
    std::cout << "Traced obstacles:\n";
    for (unsigned int j = 0; j < participating_obstacles.size(); j++) {
      std::cout << "Obstacle" << participating_obstacles[j] << "\n";
    }
  }

  for (unsigned int i = 0; i < obstacles.size(); i++) {
    debug_visualizer.addCuboid(obstacles[i].getRelativeCuboid(), ORANGE, true);
    /* debug_visualizer.addCuboid(obstacles[i].getRelativeCuboid(), BLACK, false); */
  }


  debug_visualizer.publish();
}
//}
