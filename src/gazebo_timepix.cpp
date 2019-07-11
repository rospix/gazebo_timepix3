#include <gazebo_timepix/python_wrappers.h>
#include <gazebo_timepix/gazebo_timepix.h>
#include <gazebo_timepix/source.h>
#include <gazebo_timepix/obstacle.h>

using namespace gazebo;

/* Destructor //{ */
Timepix::~Timepix() {
  std::cout << "Removed Timepix\n";
}
//}

/* Load //{ */
void Timepix::Load(physics::ModelPtr _model, [[maybe_unused]] sdf::ElementPtr _sdf) {
  // init local variables
  model_ = _model;
  world_ = _model->GetWorld();

  // init ros node
  int    argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_ros_radiation", ros::init_options::NoSigintHandler);
  this->rosNode.reset(new ros::NodeHandle("~"));

  std::stringstream ss;
  ss << "/" << model_->GetName().c_str() << "/timepix/photon_count";
  timepix_pub = rosNode->advertise<gazebo_rad_msgs::Timepix>(ss.str().c_str(), 100);

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Timepix::OnUpdate, this, _1));
  std::cout << "Timepix initialized\n";
}
//}

/* OnUpdate //{ */
void Timepix::OnUpdate(const common::UpdateInfo &upd) {
  gazebo_rad_msgs::Timepix msg;
  msg.stamp = ros::Time::now();
  msg.count = 69420;
  timepix_pub.publish(msg);
  /* std::cout << "Timepix update\n"; */
}
//}
