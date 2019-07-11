#include <gazebo_timepix/python_wrappers.h>
#include <gazebo_timepix/gazebo_timepix.h>
#include <boost/thread.hpp>

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

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);
  this->rosNode.reset(new ros::NodeHandle("~"));

  // parse parameters from sdf file
  this->exposition_time = _sdf->Get<double>("exposition_time");

  std::stringstream ss;
  ss << "/" << model_->GetName().c_str() << "/timepix/photon_count";
  timepix_pub = rosNode->advertise<gazebo_rad_msgs::Timepix>(ss.str().c_str(), 100);

  updateConnection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&Timepix::LateUpdate, this));

  terminated        = false;
  simulation_thread = boost::thread(boost::bind(&Timepix::SimulationThread, this));

  ROS_INFO("[Timepix%u]: Plugin initialized", model_->GetId());
}
//}

/* OnUpdate //{ */
void Timepix::LateUpdate() {
  gazebo_rad_msgs::Timepix msg;
  msg.stamp = ros::Time::now();
  msg.count = 69420;
  timepix_pub.publish(msg);
  /* std::cout << "Timepix update\n"; */
}

void Timepix::SimulationThread() {
  while (!terminated) {
    auto step_start = std::chrono::high_resolution_clock::now();
    std::cout << "weeee\n";
    std::this_thread::sleep_for(std::chrono::duration<double>(exposition_time));
    auto step_end = std::chrono::high_resolution_clock::now();
  }
}
//}
