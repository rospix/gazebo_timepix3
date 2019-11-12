#include <gazebo_timepix/gazebo_timepix.h>

// index sides of the detector for convenience
#define FRONT 0
#define BACK 1
#define LEFT 2
#define RIGHT 3
#define BOTTOM 4
#define TOP 5

#define ORANGE Eigen::Vector4d(1.0, 0.3, 0.2, 1.0)
#define GREEN Eigen::Vector4d(0.3, 1.0, 0.3, 1.0)
#define BLUE Eigen::Vector4d(0.2, 0.2, 1.0, 1.0)
#define GRAY Eigen::Vector4d(0.7, 0.7, 0.7, 1.0)
#define BROWN Eigen::Vector4d(0.3, 0.2, 0.0, 1.0)

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
  std::chrono::duration<double> exposition_duration;
  if (_sdf->HasElement("exposition_time")) {
    exposition_seconds = _sdf->Get<double>("exposition_time");
    /* exposition_duration = ros::Duration(exposition_seconds); */
  } else {
    ROS_WARN("[Timepix%u]: parameter 'publish_rate' was not specified", model_->GetId());
  }
  //}

  // init local variables
  model_ = _model;
  buildSensorCuboid();
  local_frame << model_->GetName().c_str() << "/timepix_origin";
  global_frame << "local_origin";
  density = getDensity(material);

  rand_dbl = std::uniform_real_distribution<double>(0, 1);

  // init gazebo node
  gazebo_node_ = transport::NodePtr(new transport::Node());
  gazebo_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_timepix", ros::init_options::NoSigintHandler);
  ros_node.reset(new ros::NodeHandle("~"));

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Timepix::onWorldUpdate, this, _1));

  // gazebo communication
  sources_sub     = gazebo_node_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
  obstacles_sub   = gazebo_node_->Subscribe("~/radiation/obstacles", &Timepix::obstaclesCallback, this, 1);
  termination_sub = gazebo_node_->Subscribe("~/radiation/termination", &Timepix::terminationCallback, this, 1);

  // ros communication
  ros_publisher         = ros_node->advertise<gazebo_rad_msgs::Timepix>("/radiation/timepix", 1);
  diagnostics_publisher = ros_node->advertise<gazebo_rad_msgs::TimepixDiagnostics>("/radiation/diagnostics", 1);

  terminated       = false;
  publisher_thread = boost::thread(boost::bind(&Timepix::PublisherLoop, this));
  ROS_INFO("[Timepix%u]: Plugin initialized", model_->GetId());

  bv = BatchVisualizer(*ros_node, global_frame.str());
}
//}

/* sourcesCallback //{ */
void Timepix::sourcesCallback(RadiationSourceConstPtr &msg) {
  for (auto it = sources.begin(); it != sources.end(); it++) {
    if (it->getId() == msg->id()) {
      Eigen::Vector3d source_world_pos(msg->x(), msg->y(), msg->z());
      it->setRelativePosition(targetRelativePosition(model_->WorldPose(), source_world_pos));
      it->setSideProperties(calculateSideProperties(*it));
      /* std::cout << "Source" << msg->id() << ": apparent activities: " << it->getApparentActivities().begin() << "\n"; */
      return;
    }
  }
  ROS_INFO("[Timepix%u]: Newly registered RadiationSource%u", model_->GetId(), msg->id());
  Eigen::Vector3d source_world_pos(msg->x(), msg->y(), msg->z());

  double mass_att_coeff            = calculateMassAttCoeff(msg->energy(), PHOTOELECTRIC);
  double diagnonal_absorption_prob = photoabsorptionProbability(diagonal_length, mass_att_coeff, density);
  Source s(msg->id(), msg->material(), msg->activity(), msg->energy(), mass_att_coeff, diagnonal_absorption_prob,
           targetRelativePosition(model_->WorldPose(), source_world_pos));
  s.setSideProperties(calculateSideProperties(s));
  sources.push_back(s);
}
//}

/* obstaclesCallback //{ */
void Timepix::obstaclesCallback(RadiationObstacleConstPtr &msg) {
  for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
    if (it->getId() == msg->id()) {
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
      it->setRelativePosition(relative_pos);
      it->setRelativeOrientation(relative_rot);
      return;
    }
  }
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

  Obstacle o(msg->id(), msg->material(), relative_pos, relative_ori, obstacle_size);
  obstacles.push_back(o);
}

//}

/* terminateCallback //{ */
void Timepix::terminationCallback(TerminationConstPtr &msg) {
  size_t sources_count = sources.size();
  sources.erase(std::remove(sources.begin(), sources.end(), msg->id()), sources.end());
  if (sources.size() != sources_count) {
    ROS_INFO("[Timepix%u]: No longer tracking RadiationSource%u", model_->GetId(), msg->id());
    return;
  }
  size_t obstacles_count = obstacles.size();
  obstacles.erase(std::remove(obstacles.begin(), obstacles.end(), msg->id()), obstacles.end());
  if (obstacles_count != obstacles.size()) {
    ROS_INFO("[Timepix%u]: No longer tracking RadiationObstacle%u", model_->GetId(), msg->id());
    return;
  }
}
//}

/* Simulate //{ */
ros::Time Timepix::Simulate(ros::Time sim_start) {
  int photons_captured = 0;
  int rays_cast        = 0;

  for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
    bv.addCuboid(o->getRelativePosition(), o->getRelativeOrientation(), o->getSize(), BROWN);
  }

  for (auto s = sources.begin(); s != sources.end(); s++) {
    // TODO obstacle attenuation

    bv.addPoint(s->getRelativePosition(), Eigen::Vector4d(0.0, 0.7, 0.0, 1.0));
    bv.addRay(Ray::twopointCast(Eigen::Vector3d::Zero(), s->getRelativePosition()), Eigen::Vector4d(0.3, 0.7, 0.9, 1.0));
    for (auto side_properties = s->getSideProperties().begin(); side_properties != s->getSideProperties().end(); side_properties++) {

      int side_index = side_properties->side_index;

      bv.addRect(sides[side_index], GRAY);
      int samples = (int)(side_properties->apparent_activity * exposition_seconds * s->getDiagonalAbsorptionProb());
      /* //{ */
      for (int n = 0; n < samples; n++) {
        ros::Time curr_time = ros::Time::now();
        auto      time_diff = (curr_time - sim_start).toSec();
        if (time_diff >= exposition_seconds) {
          ROS_WARN("[Timepix%u]: Time slip detected. Simulation incomplete", model_->GetId());
          publishSensorMsg(photons_captured);
          return ros::Time::now();
        }

        Eigen::Vector3d intersect1 = sampleRectangle(sides[side_index]);

        Ray r = Ray::twopointCast(s->getRelativePosition(), intersect1);
        rays_cast++;
        for (int j = 0; j < 6; j++) {
          if (j == side_index) {
            continue;
          }
          boost::optional<Eigen::Vector3d> intersect2 = sides[j].intersectionRay(r);
          double                           pe_prob;
          if (intersect2) {
            double track_length   = (intersect2.get() - intersect1).norm();
            double mass_att_coeff = s->getMassAttCoeff();
            pe_prob               = photoabsorptionProbability(track_length, mass_att_coeff, density) / s->getDiagonalAbsorptionProb();

            /* std::cout << "pe_prob: " << (pe_prob * 100) << "\%\n"; */
            double coin_flip = rand_dbl(rand_gen);
            if (coin_flip < pe_prob) {
              bv.addRay(r, GREEN);
              photons_captured++;
            } else {
              bv.addRay(r, ORANGE);
            }
            break;
          }
        }
      }
      //}
    }
  }
  /* auto sim_end      = ros::Time::now(); */
  /* auto sim_duration = (sim_end - sim_start).toSec(); */
  /* std::cout << "simulation duration: " << (1000 * sim_duration) << " ms\n"; */
  /* ROS_INFO("[Timepix%u]: Rays cast: %d, Capture percentage: %.3f%, Simulation speed: %.3f rays/sec", model_->GetId(), rays_cast, */
  /* (100.0 * photons_captured) / rays_cast, rays_cast / sim_duration_sec.count()); */
  publishSensorMsg(photons_captured);
  return ros::Time::now();
}
//}

/* PublisherLoop //{ */
void Timepix::PublisherLoop() {
  while (!terminated) {
    auto sim_start = ros::Time::now();
    bv.clear();
    auto sim_end = Simulate(sim_start);

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
    bv.publish();
    //}

    /* diagnostics (ROS message) //{ */
    publishDiagnostics();
    //}

    auto sim_duration = sim_end - sim_start;
    (ros::Duration(exposition_seconds) - sim_duration).sleep();
    /* std::this_thread::sleep_for(exposition_duration - sim_duration); */
  }
}
//}

/* calculateSideProperties //{ */
std::set<Triplet> Timepix::calculateSideProperties(Source s) {
  std::set<Triplet> ret;
  for (int i = 0; i < 6 && ret.size() <= 3; i++) {
    if (sides[i].normal_vector.dot(s.getRelativePosition()) > 0) {
      double solid_angle = rectSolidAngle(sides[i], s.getRelativePosition());
      if (solid_angle == solid_angle) {  // NaN check
        double  apparent_activity = (s.getActivity() / 4 * M_PI) * solid_angle;
        Triplet triplet;
        triplet.side_index                = i;
        triplet.apparent_activity         = apparent_activity;
        triplet.diagnonal_absorption_prob = photoabsorptionProbability(diagonal_length, s.getMassAttCoeff(), density);
        ret.insert(triplet);
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

/* onWorldUpdate //{ */
void Timepix::onWorldUpdate(const common::UpdateInfo &upd) {
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

  return r.points[0] + k1 * r.basis.col(0) + k2 * r.basis.col(1);
}
//}

/* photoabsorptionProbability //{*/
double Timepix::photoabsorptionProbability(double material_thickness, double mass_att_coeff, double material_density) {
  return 1.0 - std::exp(-mass_att_coeff * material_thickness * 100.0 * material_density);  // multiply by 100 to get thickness in cm
}
//}

/* getDensity //{ */
double Timepix::getDensity(std::string material) {
  auto nist_table = loadNistTable(material);
  if (nist_table.size() > 0) {
    std::string density_str = nist_table[0][3];
    return std::stod(density_str);
  }
  ROS_WARN("[Timepix%u]: Material properties not found", model_->GetId());
  return -1;
}
//}

/* calculateMassAttCoeff //{ */
double Timepix::calculateMassAttCoeff(double photon_energy, AttenuationType a) {
  auto nist_table = loadNistTable(material);

  double mass_att_coeff = 0.0;
  if (nist_table.size() > 0) {
    for (size_t i = 1; i < nist_table.size(); i++) {

      // lookup the NIST table
      double table_energy = std::stod(nist_table[i][0]);
      if (table_energy >= photon_energy) {
        // perform linear interpolation between two closest table values
        double lower_att_coeff  = std::stod(nist_table[i - 1][a]);
        double higher_att_coeff = std::stod(nist_table[i][a]);
        double lower_energy     = std::stod(nist_table[i - 1][0]);

        double frac    = (photon_energy - lower_energy) / (table_energy - lower_energy);
        mass_att_coeff = lower_att_coeff + (frac * (higher_att_coeff - lower_att_coeff));
        break;
      }
    }
  }
  ROS_INFO("[Timepix%u]: Interpolated mass attenuation coefficient: %.4f", model_->GetId(), mass_att_coeff);
  return mass_att_coeff;
}
//}

/* loadNistTable //{ */
std::vector<std::vector<std::string>> Timepix::loadNistTable(std::string material) {
  std::vector<std::vector<std::string>> table;

  std::stringstream ss;
  auto              curr_path = boost::filesystem::current_path();
  ss << curr_path.c_str() << "/../mrs_workspace/src/radiation_nodes/gazebo_timepix/nist/" << material.c_str() << ".csv";
  std::ifstream nist_file;
  nist_file.open(ss.str().c_str());
  if (!nist_file.is_open()) {
    ROS_WARN("[Timepix%u]: Failed to open \"%s.csv\"", model_->GetId(), material.c_str());
    return table;
  }
  while (true) {
    std::string line;
    getline(nist_file, line);
    std::vector<std::string> line_elems;
    boost::split(line_elems, line, [](char c) { return c == ','; });
    if (line_elems.size() < 3) {
      if (table.size() < 1) {
        ROS_WARN("[Timepix%u]: Material properties not properly formatted. Expected 4 columns of data, got %ld", model_->GetId(), line_elems.size());
      }
      break;
    }
    std::vector<std::string> line_slice(line_elems.begin(), line_elems.end() - 1);
    table.push_back(line_slice);
    /* std::cout << "adding line\n"; */
  }
  ROS_INFO("[Timepix%u]: Loaded NIST table \"%s.csv\"", model_->GetId(), material.c_str());
  /* std::cout << "Table content:" << table.size() << "rows, " << table.begin()->size() << "cols\n"; */
  /* for (auto r = table.begin(); r != table.end(); r++) { */
  /*   for (auto c = r->begin(); c != r->end(); c++) { */
  /*     std::cout << c->c_str() << ", "; */
  /*   } */
  /*   std::cout << std::endl; */
  /* } */
  return table;
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
  msg.id              = model_->GetId();
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
}
//}
