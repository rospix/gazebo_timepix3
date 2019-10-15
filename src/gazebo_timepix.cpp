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

using namespace gazebo;

/* Destructor //{ */
Timepix::~Timepix() {

  // shutdown subscribers
  sources_sub->Unsubscribe();
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
    ROS_WARN("[RadiationSource%u]: parameter 'material' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("size")) {
    size = _sdf->Get<ignition::math::Vector3d>("size");
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'size' was not specified", model_->GetId());
  }
  if (_sdf->HasElement("exposition_time")) {
    exposition_seconds  = _sdf->Get<double>("exposition_time");
    exposition_duration = std::chrono::duration<double>(exposition_seconds);
  } else {
    ROS_WARN("[RadiationSource%u]: parameter 'publish_rate' was not specified", model_->GetId());
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

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Timepix::OnWorldUpdate, this, _1));

  // gazebo communication
  sources_sub     = gazebo_node_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
  termination_sub = gazebo_node_->Subscribe("~/radiation/termination", &Timepix::terminationCallback, this, 1);

  // ros communication
  ros_publisher = ros_node->advertise<gazebo_rad_msgs::Timepix>("/radiation/timepix", 1);

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
  Source          s(msg->id(), msg->material(), msg->activity(), targetRelativePosition(model_->WorldPose(), source_world_pos));
  s.setSideProperties(calculateSideProperties(s));
  sources.push_back(s);
}
//}

/* terminateCallback //{ */
void Timepix::terminationCallback(TerminationConstPtr &msg) {
  ROS_INFO("[Timepix%u]: No longer tracking RadiationSource%u", model_->GetId(), msg->id());
  std::vector<Source>::iterator it = remove(sources.begin(), sources.end(), msg->id());
  sources.erase(it, sources.end());
}
//}

/* Simulate //{ */
void Timepix::Simulate() {
  int required_photon_count = 0;
  int photons_captured      = 0;
  /* std::cout << "-------------------------\n"; */
  for (auto s = sources.begin(); s != sources.end(); s++) {
    // TODO obstacle attenuation

    bv.addPoint(s->getRelativePosition(), Eigen::Vector4d(0.0, 0.7, 0.0, 1.0));
    bv.addRay(Ray::twopointCast(Eigen::Vector3d::Zero(), s->getRelativePosition()), Eigen::Vector4d(0.3, 0.7, 0.9, 1.0));
    for (auto side_properties = s->getSideProperties().begin(); side_properties != s->getSideProperties().end(); side_properties++) {


      std::cout << "Read: " << side_properties->side_index << "\n";
      int side_index = side_properties->side_index;

      bv.addRect(sides[side_index], GRAY);
      int samples = (int)(side_properties->apparent_activity * exposition_seconds);
      /* std::cout << "[Dosimeter]: for side " << side->first << " with app activity " << side->second << " simulated " << samples << " samples\n"; */
      for (int n = 0; n < samples; n++) {
        Eigen::Vector3d intersect1 = sampleRectangle(sides[side_index]);

        Ray r = Ray::twopointCast(s->getRelativePosition(), intersect1);
        for (int j = 0; j < 6; j++) {
          if (j == side_index) {
            continue;
          }
          boost::optional<Eigen::Vector3d> intersect2 = sides[j].intersectionRay(r);
          double                           pe_prob;
          if (intersect2) {
            double track_length   = (intersect2.get() - intersect1).norm();
            double mass_att_coeff = calculateMassAttCoeff(s->getEnergy(), density);
            double diag_abs_prob  = photoabsorptionProbability(diagonal_length, mass_att_coeff, density);
            pe_prob               = photoabsorptionProbability(track_length, mass_att_coeff, density) / diag_abs_prob;

            /* std::cout << "pe_prob: " << (pe_prob * 100) << "\%\n"; */
            double coin_flip = rand_dbl(rand_gen);
            if (coin_flip < pe_prob) {
              bv.addRay(r, GREEN);
            } else {
              bv.addRay(r, ORANGE);
            }
            break;
          }
        }
      }
    }
  }
  /* std::cout << "-------------------------\n"; */
}
//}

/* PublisherLoop //{ */
void Timepix::PublisherLoop() {
  while (!terminated) {
    bv.clear();
    Simulate();

    /* ROS message (debug) //{ */
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

    std::this_thread::sleep_for(exposition_duration);
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
        triplet.diagnonal_absorption_prob = 0;  // TODO
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

/* OnWorldUpdate //{ */
void Timepix::OnWorldUpdate(const common::UpdateInfo &upd) {
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
  return 1.0 - std::exp(-mass_att_coeff * material_thickness * 100 * material_density);  // multiply by 100 to get thickness in cm
}
//}

/* getDensity //{ */
double Timepix::getDensity(std::string material) {
  // TODO
  // lookup nist table for density
  return -1;
}
//}

/* calculateMassAttCoeff //{ */
double Timepix::calculateMassAttCoeff(double photon_energy, double material_density) {
  // TODO
  // lookup nist table
  // interpolate
  return -1;
}
//}
