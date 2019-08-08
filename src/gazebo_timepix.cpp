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
  updateConnection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&Timepix::LateUpdate, this));

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
  //}

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

  terminated        = false;
  simulation_thread = boost::thread(boost::bind(&Timepix::SimulationThread, this));

  // subscribe to gazebo topics
  sources_sub     = transport_node_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
  obstacles_sub   = transport_node_->Subscribe("~/radiation/obstacles", &Timepix::obstaclesCallback, this, 1);
  termination_sub = transport_node_->Subscribe("~/radiation/termination", &Timepix::terminationCallback, this, 1);

  ROS_INFO("[Timepix%u]: Plugin initialized", model_->GetId());
}
//}

/* LateUpdate //{ */
void Timepix::LateUpdate() {
  ignition::math::Quaterniond q = model_->WorldPose().Rot();
  local2world.w()               = q.W();
  local2world.x()               = q.X();
  local2world.y()               = q.Y();
  local2world.z()               = q.Z();
  try {
    world2local = local2world.inverse();
  }
  catch (...) {
    return;
  }
  ROS_INFO_STREAM("Local to world Quaternion: " << world2local.coeffs());

  tf::Transform  transform;
  tf::Quaternion quat(model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z(), model_->WorldPose().Rot().W());
  tf::Vector3    origin(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
  transform.setOrigin(origin);
  transform.setRotation(quat);
  transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin", frame_name.str()));
}
//}

/* SimulationThread //{ */
void Timepix::SimulationThread() {
  while (!terminated) {
    auto start_time = std::chrono::high_resolution_clock::now();

    bv.clear();
    bv.addCuboid(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), Eigen::Vector3d(sensor_thickness, sensor_size, sensor_size),
                 Eigen::Vector4d(0.0, 0.3, 1.0, 0.9));
    for (Obstacle o : obstacles) {
      bv.addCuboid(o.getRelativePosition(), o.getRelativeOrientation(), o.getScale(), Eigen::Vector4d(0.7, 0.4, 0.1, 0.9));
    }
    bv.publish();


    auto   end_time = std::chrono::high_resolution_clock::now();
    double dt       = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1E9;
    ROS_INFO("delta_t: %.8f", dt);
    double sleep_time = std::max(0.0, exposition_time - dt);
    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
  }
}
//}

/* simulate //{ */
void Timepix::simulate() {
}
//}

/* sourcesCallback //{ */
void Timepix::sourcesCallback(RadiationSourceConstPtr &msg) {
  sources_mutex.lock();
  for (Source s : sources) {
    if (s.getId() == msg->id()) {
      s.setWorldPosition(Eigen::Vector3d(msg->x(), msg->y(), msg->z()));
      sources_mutex.unlock();
      return;
    }
  }
  Eigen::Vector3d source_pos(msg->x(), msg->y(), msg->z());
  Source          s(msg->id(), msg->material(), msg->activity(), source_pos);
  sources.push_back(s);
  ROS_INFO("[Timepix%u]: Registered RadiationSource%u", model_->GetId(), msg->id());
  sources_mutex.unlock();
}
//}

/* obstaclesCallback //{ */
void Timepix::obstaclesCallback(RadiationObstacleConstPtr &msg) {
  obstacles_mutex.lock();
  for (Obstacle o : obstacles) {
    // update known obstacle
    if (o.getId() == msg->id()) {
      Eigen::Quaterniond relative_orientation = world2local * Eigen::Quaterniond(msg->ori_w(), msg->ori_x(), msg->ori_y(), msg->ori_z());
      Eigen::Vector3d    relative_position    = world2local * (Eigen::Vector3d(msg->pos_x(), msg->pos_y(), msg->pos_z()) - pos3toVector3d(model_->WorldPose()));
      Eigen::Vector3d    obstacle_scale(msg->scale_x(), msg->scale_y(), msg->scale_z());
      o.updatePose(relative_position, relative_orientation, obstacle_scale);
      obstacles_mutex.unlock();
      return;
    }
  }
  // register new obstacle
  Eigen::Quaterniond relative_orientation = world2local * Eigen::Quaterniond(msg->ori_w(), msg->ori_x(), msg->ori_y(), msg->ori_z());
  Eigen::Vector3d    relative_position    = world2local * (Eigen::Vector3d(msg->pos_x(), msg->pos_y(), msg->pos_z()) - pos3toVector3d(model_->WorldPose()));
  Eigen::Vector3d    obstacle_scale(msg->scale_x(), msg->scale_y(), msg->scale_z());

  Obstacle o(msg->id(), msg->material(), relative_position, relative_orientation, obstacle_scale);
  obstacles.push_back(o);

  ROS_INFO("[Timepix%u]: Registered RadiationObstacle%u", model_->GetId(), msg->id());
  ROS_INFO("[Timepix%u]: Position: %.2f, %.2f, %2.f", model_->GetId(), relative_position[0], relative_position[1], relative_position[2]);
  ROS_INFO("[Timepix%u]: Rotation: %.2f, %.2f, %2.f, %2.f", model_->GetId(), relative_orientation.w(), relative_position.x(), relative_position.y(),
           relative_position.z());
  ROS_INFO("[Timepix%u]: Scale: %.2f, %.2f, %2.f", model_->GetId(), obstacle_scale[0], obstacle_scale[1], obstacle_scale[2]);

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

//}
