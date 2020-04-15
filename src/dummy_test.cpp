#include <ros/ros.h>
#include <gazebo_timepix/data_utils.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "timepix_test");
  ros::NodeHandle nh = ros::NodeHandle("~");

  try {
    Table t1 = loadNistTable("ag");
    ROS_INFO_STREAM("[Timepix test]: \"ag.csv\" loaded successfully");
  }catch(std::exception &e){
    ROS_ERROR_STREAM("[Timepix test]: " << e.what());
  }
  
  try {
    Table t2 = loadNistTable("si");
    ROS_INFO_STREAM("[Timepix test]: \"si.csv\" loaded successfully");
  }catch(std::exception &e){
    ROS_ERROR_STREAM("[Timepix test]: " << e.what());
  }

  ros::spin();
  return 0;
}
