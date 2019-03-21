#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <geometry_utils.h>
#include <radiation_utils.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class RadiationVisualizer {
public:
  static void visualizeSources(ros::Publisher pub, std::vector<Source> sources, std::string frame);
  static void visualizeRay(ros::Publisher pub, Ray ray, std::string frame);
  static void visualizePoint(ros::Publisher pub, Eigen::Vector3d p, std::string frame, double r = 0.0, double g = 1.0, double b = 0.0);
  static void visualizeRect(ros::Publisher pub, Rectangle rect, std::string frame, double r = 0.0, double g = 0.7, double b = 1.0);
};

