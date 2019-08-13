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

  /* std::cout << "AAAAAA\n"; */

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

  /* std::cout << "BBBBBBB\n"; */
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

  /* std::cout << "CCCCCCC\n"; */
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

  /* std::cout << "DDDDDDD\n"; */

  // subscribe to gazebo topics
  sources_sub     = transport_node_->Subscribe("~/radiation/sources", &Timepix::sourcesCallback, this, 1);
  obstacles_sub   = transport_node_->Subscribe("~/radiation/obstacles", &Timepix::obstaclesCallback, this, 1);
  termination_sub = transport_node_->Subscribe("~/radiation/termination", &Timepix::terminationCallback, this, 1);

  /* std::cout << "EEEEEEE\n"; */

  ROS_INFO("[Timepix%u]: Plugin initialized", model_->GetId());

  terminated        = false;
  simulation_thread = boost::thread(boost::bind(&Timepix::SimulationThread, this));
}
//}

/* SimulationThread //{ */
void Timepix::SimulationThread() {
  /* std::this_thread::sleep_for(std::chrono::duration<double>(1.0)); */
  /* std::cout << "FFFFFFF\n"; */
  while (!terminated) {
    auto start_time = std::chrono::high_resolution_clock::now();
    /* std::cout << "GGGGGG\n"; */
    /* Relative geometry handler //{ */

    /* std::cout << "HHHHHHHHH\n"; */
    tf::Transform  transform;
    tf::Vector3    origin(model_->WorldPose().Pos().X(), model_->WorldPose().Pos().Y(), model_->WorldPose().Pos().Z());
    tf::Quaternion quat;
    quat.setW(model_->WorldPose().Rot().W());
    quat.setX(model_->WorldPose().Rot().X());
    quat.setY(model_->WorldPose().Rot().Y());
    quat.setZ(model_->WorldPose().Rot().Z());
    transform.setOrigin(origin);
    transform.setRotation(quat);
    /* std::cout << "IIIIII\n"; */
    transform_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "local_origin", frame_name.str()));
    //}
    /* std::cout << "JJJJJJJ\n"; */


    bv.clear();
    /* std::cout << "Tick\n"; */
    obstacles_mutex.lock();
    for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
      bv.addCuboid(o->getRelativePosition(), o->getRelativeOrientation(), o->getScale(), ORANGE);
    }
    obstacles_mutex.unlock();
    bv.publish();
    /* std::cout << "KKKKKKK\n"; */

    auto   end_time   = std::chrono::high_resolution_clock::now();
    double dt         = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count() / 1E9;
    double sleep_time = std::max(0.0, exposition_time - dt);
    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
  }
}
//}

/* sourcesCallback //{ */
void Timepix::sourcesCallback(RadiationSourceConstPtr &msg) {
  bool new_source = true;
  sources_mutex.lock();
  for (auto s = sources.begin(); s != sources.end(); s++) {
    if (s->getId() == msg->id()) {
      s->setWorldPosition(Eigen::Vector3d(msg->x(), msg->y(), msg->z()));
      sources_mutex.unlock();
      new_source = false;
      break;
    }
  }
  sources_mutex.unlock();
  if (!new_source) {
    return;
  }
  Eigen::Vector3d source_pos(msg->x(), msg->y(), msg->z());
  Source          s(msg->id(), msg->material(), msg->activity(), source_pos);
  sources_mutex.lock();
  sources.push_back(s);
  sources_mutex.unlock();
  ROS_INFO("[Timepix%u]: Registered RadiationSource%u", model_->GetId(), msg->id());
}
//}

/* obstaclesCallback //{ */
void Timepix::obstaclesCallback(RadiationObstacleConstPtr &msg) {
  std::cout << "111111\n";
  bool new_obstacle = true;
  for (auto o = obstacles.begin(); o != obstacles.end(); o++) {
    /*   // update known obstacle */
    if (o->getId() == msg->id()) {
      std::cout << "22222\n";
      transform_mutex.lock();
      Eigen::Quaterniond local2world =
          Eigen::Quaterniond(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z());
      Eigen::Quaterniond world2local = local2world.inverse();
      std::cout << "MSG ori: " << msg->ori_w() << ", " << msg->ori_x() << ", " << msg->ori_y() << ", " << msg->ori_z() << "\n";
      std::cout << "MSG pos: " << msg->pos_x() << ", " << msg->pos_y() << ", " << msg->pos_z() << "\n";
      Eigen::Quaterniond relative_orientation = world2local * Eigen::Quaterniond(msg->ori_w(), msg->ori_x(), msg->ori_y(), msg->ori_z());
      std::cout << "222AAAA\n";
      Eigen::Vector3d relative_position = world2local * (Eigen::Vector3d(msg->pos_x(), msg->pos_y(), msg->pos_z()) - pos3toVector3d(model_->WorldPose()));
      transform_mutex.unlock();
      std::cout << "222BBBB\n";
      Eigen::Vector3d obstacle_scale(msg->scale_x(), msg->scale_y(), msg->scale_z());
      std::cout << "222CCCC\n";
      obstacles_mutex.lock();
      o->updatePose(relative_position, relative_orientation, obstacle_scale);
      obstacles_mutex.unlock();
      std::cout << "222DDDD\n";
      new_obstacle = false;
      std::cout << "Obstacle position updated: " << o->getRelativePosition() << "\n";
      break;
    }
  }
  std::cout << "3333333\n";
  if (!new_obstacle) {
    return;
  }
  // register new obstacle
  std::cout << "444444\n";
  transform_mutex.lock();
  Eigen::Quaterniond local2world =
      Eigen::Quaterniond(model_->WorldPose().Rot().W(), model_->WorldPose().Rot().X(), model_->WorldPose().Rot().Y(), model_->WorldPose().Rot().Z());
  Eigen::Quaterniond world2local = local2world.inverse();
  std::cout << "5555555\n";
  Eigen::Quaterniond relative_orientation = world2local * Eigen::Quaterniond(msg->ori_w(), msg->ori_x(), msg->ori_y(), msg->ori_z());
  std::cout << "56565656\n";
  Eigen::Vector3d relative_position = world2local * (Eigen::Vector3d(msg->pos_x(), msg->pos_y(), msg->pos_z()) - pos3toVector3d(model_->WorldPose()));
  std::cout << "666666\n";
  transform_mutex.unlock();
  Eigen::Vector3d obstacle_scale(msg->scale_x(), msg->scale_y(), msg->scale_z());
  std::cout << "77777777\n";

  Obstacle o(msg->id(), msg->material(), relative_position, relative_orientation, obstacle_scale);
  std::cout << "888888\n";
  obstacles_mutex.lock();
  obstacles.push_back(o);
  obstacles_mutex.unlock();
  ROS_INFO("[Timepix%u]: Registered RadiationObstacle%u", model_->GetId(), msg->id());
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
