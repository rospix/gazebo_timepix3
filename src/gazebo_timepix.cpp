#include <gazebo_timepix/gazebo_timepix.h>
#include <gazebo_timepix/python_wrappers.h>

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

  // init local variables
  model_ = _model;
  world_ = _model->GetWorld();

  transport_node_ = transport::NodePtr(new transport::Node());
  transport_node_->Init();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);
  this->ros_node.reset(new ros::NodeHandle("~"));

  // parse sdf params
  if (_sdf->HasElement("exposition_time")) {
    exposition_time = _sdf->Get<double>("exposition_time");
  } else {
    ROS_WARN("[Timepix%u]: parameter 'exposition_time' was not specified", model_->GetId());
  }

  // subscribe to gazebo topics
  sources_sub     = transport_node_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
  obstacles_sub   = transport_node_->Subscribe("~/radiation/obstacles", &Timepix::obstaclesCallback, this, 1);
  termination_sub = transport_node_->Subscribe("~/radiation/termination", &Timepix::terminationCallback, this, 1);

  // init sensor publisher
  std::stringstream ss;
  ss << "/" << model_->GetName().c_str() << "/timepix/photon_count";
  timepix_pub = ros_node->advertise<gazebo_rad_msgs::Timepix>(ss.str().c_str(), 100);


  updateConnection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&Timepix::LateUpdate, this));

  terminated        = false;
  simulation_thread = boost::thread(boost::bind(&Timepix::SimulationThread, this));

  ROS_INFO("[Timepix%u]: Plugin initialized", model_->GetId());
}
//}

/* LateUpdate //{ */
void Timepix::LateUpdate() {
  world_pos[0] = model_->WorldPose().Pos().X();
  world_pos[1] = model_->WorldPose().Pos().Y();
  world_pos[2] = model_->WorldPose().Pos().Z();
}
//}

/* SimulationThread //{ */
void Timepix::SimulationThread() {
  while (!terminated) {
    std::this_thread::sleep_for(std::chrono::duration<double>(exposition_time));
  }
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
      o.setWorldPosition(Eigen::Vector3d(msg->pos_x(), msg->pos_y(), msg->pos_z()));
      o.setOrientation(Eigen::Quaterniond(msg->ori_w(), msg->ori_x(), msg->ori_y(), msg->ori_z()));
      obstacles_mutex.unlock();
      return;
    }
  }
  // register new obstacle
  Eigen::Vector3d    obstacle_pos(msg->pos_x(), msg->pos_y(), msg->pos_z());
  Eigen::Quaterniond orientation(msg->ori_x(), msg->ori_y(), msg->ori_z(), msg->ori_w());
  Eigen::Vector3d    scale(msg->scale_x(), msg->scale_y(), msg->scale_z());

  Obstacle o(msg->id(), msg->material(), obstacle_pos, orientation, scale);
  obstacles.push_back(o);

  ROS_INFO("[Timepix%u]: Registered RadiationObstacle%u", model_->GetId(), msg->id());
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

//}
