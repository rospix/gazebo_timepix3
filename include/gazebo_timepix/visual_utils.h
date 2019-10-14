#ifndef RADIATION_UTILS_VISUAL_H
#define RADIATION_UTILS_VISUAL_H

#define RAY_THICKNESS 0.005
#define POINT_SIZE 0.05

#include <eigen3/Eigen/Core>
#include <gazebo_timepix/geometry_utils.h>
#include <geometry_msgs/Point.h>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>

/* VisualTools //{ */
class VisualTools {
public:
  static void visualizeRay(ros::Publisher pub, Ray ray, std::string frame);
  static void visualizePoint(ros::Publisher pub, Eigen::Vector3d p, std::string frame, double size, double r = 0.0, double g = 1.0, double b = 0.0);
  static void visualizeRect(ros::Publisher pub, Rectangle rect, std::string frame, double r = 0.0, double g = 0.7, double b = 1.0);
  static void visualizeCuboid(ros::Publisher pub, Cuboid cuboid, std::string frame, double r = 0.0, double g = 0.7, double b = 1.0);
};

/* visualizeRay */  //{
void VisualTools::visualizeRay(ros::Publisher pub, Ray ray, std::string frame) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id    = frame;
  line_strip.header.stamp       = ros::Time::now();
  line_strip.ns                 = "ray";
  line_strip.action             = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id                 = 1;
  line_strip.type               = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.002;

  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p1, p2;

  p1.x = ray.p1[0];
  p1.y = ray.p1[1];
  p1.z = ray.p1[2];

  p2.x = ray.p2[0];
  p2.y = ray.p2[1];
  p2.z = ray.p2[2];

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  pub.publish(line_strip);
}
//}

/* visualizePoint */  //{
void VisualTools::visualizePoint(ros::Publisher pub, Eigen::Vector3d p, std::string frame, double size, double r, double g, double b) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "point";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::POINTS;

  marker.scale.x = size;
  marker.scale.y = size;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::Point gp;
  gp.x = p[0];
  gp.y = p[1];
  gp.z = p[2];

  marker.points.push_back(gp);

  pub.publish(marker);
}
//}

/* visualizeRect */  //{
void VisualTools::visualizeRect(ros::Publisher pub, Rectangle rect, std::string frame, double r, double g, double b) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id    = frame;
  line_strip.header.stamp       = ros::Time::now();
  line_strip.ns                 = "rect";
  line_strip.action             = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id                 = 1;
  line_strip.type               = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.002;

  line_strip.color.r = r;
  line_strip.color.g = g;
  line_strip.color.b = b;
  line_strip.color.a = 1.0;

  geometry_msgs::Point pa, pb, pc, pd;

  pa.x = rect.points[0][0];
  pa.y = rect.points[0][1];
  pa.z = rect.points[0][2];

  pb.x = rect.points[1][0];
  pb.y = rect.points[1][1];
  pb.z = rect.points[1][2];

  pc.x = rect.points[2][0];
  pc.y = rect.points[2][1];
  pc.z = rect.points[2][2];

  pd.x = rect.points[3][0];
  pd.y = rect.points[3][1];
  pd.z = rect.points[3][2];

  line_strip.points.push_back(pa);
  line_strip.points.push_back(pb);
  line_strip.points.push_back(pc);
  line_strip.points.push_back(pd);
  line_strip.points.push_back(pa);

  pub.publish(line_strip);
  ros::spinOnce();
}
//}

//}

/* BatchVisualizer //{ */

/* declaration //{ */
class BatchVisualizer {
public:
  BatchVisualizer();
  ~BatchVisualizer();
  BatchVisualizer(ros::NodeHandle rosNode, std::string frame);

  void addRay(Ray ray, Eigen::Vector4d color);
  void addPoint(Eigen::Vector3d p, Eigen::Vector4d color);
  void addRect(Rectangle rect, Eigen::Vector4d color);
  void addCuboid(Eigen::Vector3d pos, Eigen::Quaterniond ori, Eigen::Vector3d sca, Eigen::Vector4d color);

  void clear();
  void publish();

private:
  std::string                       frame;
  std::vector<geometry_msgs::Point> vertices;
  std::vector<std_msgs::ColorRGBA>  vertex_colors;
  ros::Publisher                    visual_pub;

  int triangle_count = 0;
};
//}

/* constructors */  //{
BatchVisualizer::BatchVisualizer() {
}

BatchVisualizer::~BatchVisualizer() {
}

BatchVisualizer::BatchVisualizer(ros::NodeHandle rosNode, std::string frame) {
  this->frame      = frame;
  this->visual_pub = rosNode.advertise<visualization_msgs::Marker>("/radiation/visualizer", 1);
}
//}

/* addRay */  //{
void BatchVisualizer::addRay(Ray ray, Eigen::Vector4d color) {

  std_msgs::ColorRGBA vertex_color;
  vertex_color.r = color[0];
  vertex_color.g = color[1];
  vertex_color.b = color[2];
  vertex_color.a = color[3];

  geometry_msgs::Point p1, p2, p3, p4;

  p1.x = ray.p1[0] + RAY_THICKNESS / 2;
  p1.y = ray.p1[1] + RAY_THICKNESS / 2;
  p1.z = ray.p1[2];

  p2.x = ray.p1[0] - RAY_THICKNESS / 2;
  p2.y = ray.p1[1] - RAY_THICKNESS / 2;
  p2.z = ray.p1[2];

  p3.x = ray.p2[0] + RAY_THICKNESS / 2;
  p3.y = ray.p2[1] + RAY_THICKNESS / 2;
  p3.z = ray.p2[2];

  p4.x = ray.p2[0] - RAY_THICKNESS / 2;
  p4.y = ray.p2[1] - RAY_THICKNESS / 2;
  p4.z = ray.p2[2];

  vertices.push_back(p1);
  vertices.push_back(p2);
  vertices.push_back(p3);
  vertices.push_back(p4);
  vertices.push_back(p3);
  vertices.push_back(p1);

  triangle_count += 2;

  for (int i = 0; i < 6; i++) {
    vertex_colors.push_back(vertex_color);
  }
}
//}

/* addPoint */  //{
void BatchVisualizer::addPoint(Eigen::Vector3d p, Eigen::Vector4d color) {

  std_msgs::ColorRGBA vertex_color;
  vertex_color.r = color[0];
  vertex_color.g = color[1];
  vertex_color.b = color[2];
  vertex_color.a = color[3];

  geometry_msgs::Point p1, p2, p3, p4;
  p1.x = p[0] + POINT_SIZE / 2;
  p1.y = p[1] + POINT_SIZE / 2;
  p1.z = p[2];

  p2.x = p[0] - POINT_SIZE / 2;
  p2.y = p[1] - POINT_SIZE / 2;
  p2.z = p[2];

  p3.x = p[0] + POINT_SIZE / 2;
  p3.y = p[1] - POINT_SIZE / 2;
  p3.z = p[2];

  p4.x = p[0] - POINT_SIZE / 2;
  p4.y = p[1] + POINT_SIZE / 2;
  p4.z = p[2];


  vertices.push_back(p1);
  vertices.push_back(p3);
  vertices.push_back(p2);
  vertices.push_back(p2);
  vertices.push_back(p1);
  vertices.push_back(p4);

  triangle_count += 2;

  for (int i = 0; i < 6; i++) {
    vertex_colors.push_back(vertex_color);
  }
}
//}

/* addRect */  //{
void BatchVisualizer::addRect(Rectangle rect, Eigen::Vector4d color) {

  std_msgs::ColorRGBA vertex_color;
  vertex_color.r = color[0];
  vertex_color.g = color[1];
  vertex_color.b = color[2];
  vertex_color.a = color[3];

  geometry_msgs::Point pa, pb, pc, pd;

  pa.x = rect.points[0][0];
  pa.y = rect.points[0][1];
  pa.z = rect.points[0][2];

  pb.x = rect.points[1][0];
  pb.y = rect.points[1][1];
  pb.z = rect.points[1][2];

  pc.x = rect.points[2][0];
  pc.y = rect.points[2][1];
  pc.z = rect.points[2][2];

  pd.x = rect.points[3][0];
  pd.y = rect.points[3][1];
  pd.z = rect.points[3][2];

  vertices.push_back(pa);
  vertices.push_back(pb);
  vertices.push_back(pc);
  vertices.push_back(pd);
  vertices.push_back(pa);
  vertices.push_back(pc);

  triangle_count += 2;

  for (int i = 0; i < 6; i++) {
    vertex_colors.push_back(vertex_color);
  }
}
//}

/* addCuboid */  //{
void BatchVisualizer::addCuboid(Eigen::Vector3d pos, Eigen::Quaterniond ori, Eigen::Vector3d sca, Eigen::Vector4d color) {
  std::cerr << "[VISUAL UTILS]: ADD CUBOID is not implemented!\n";
  // TODO
}
//}

/* clear */  //{
void BatchVisualizer::clear() {
  vertices.clear();
  vertex_colors.clear();
  triangle_count = 0;
}
//}

/* publish */  //{
void BatchVisualizer::publish() {
  visualization_msgs::Marker msg;
  msg.header.frame_id    = frame;
  msg.header.stamp       = ros::Time::now();
  msg.ns                 = "current_namespace";
  msg.action             = visualization_msgs::Marker::ADD;
  msg.type               = visualization_msgs::Marker::TRIANGLE_LIST;
  msg.id                 = 666;
  msg.pose.position.x    = 0.0;
  msg.pose.position.y    = 0.0;
  msg.pose.position.z    = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  msg.scale.x            = 1.0;
  msg.scale.y            = 1.0;
  msg.scale.z            = 1.0;
  msg.points             = vertices;
  msg.colors             = vertex_colors;
  /* std::cout << "Visualizing " << triangle_count << " triangles, " << vertices.size() << " points\n"; */

  visual_pub.publish(msg);
}
//}

//}

#endif /* RADIATION_UTILS_VISUAL_H */
