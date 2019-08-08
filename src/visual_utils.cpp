#include <gazebo_timepix/visual_utils.h>

/* constructors */  //{
BatchVisualizer::BatchVisualizer() {
}

BatchVisualizer::~BatchVisualizer() {
}

BatchVisualizer::BatchVisualizer(ros::NodeHandle rosNode, std::string frame) {
  this->frame      = frame;
  this->visual_pub = rosNode.advertise<visualization_msgs::MarkerArray>("/radiation/visualizer", 1);
}
//}

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

/* addRay */  //{
void BatchVisualizer::addRay(Ray ray, double r, double g, double b, double scale) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "ray";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = scale;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::Point p1, p2;

  p1.x = ray.p1[0];
  p1.y = ray.p1[1];
  p1.z = ray.p1[2];

  p2.x = ray.p2[0];
  p2.y = ray.p2[1];
  p2.z = ray.p2[2];

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  msg.markers.push_back(marker);
}
//}

/* addPoint */  //{
void BatchVisualizer::addPoint(Eigen::Vector3d p, double r, double g, double b, double scale) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "point";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::POINTS;

  marker.scale.x = scale;
  marker.scale.y = scale;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::Point gp;
  gp.x = p[0];
  gp.y = p[1];
  gp.z = p[2];

  marker.points.push_back(gp);
  msg.markers.push_back(marker);
}
//}

/* addRect */  //{
void BatchVisualizer::addRect(Rectangle rect, double r, double g, double b, double scale) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "rect";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = scale;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

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

  marker.points.push_back(pa);
  marker.points.push_back(pb);
  marker.points.push_back(pc);
  marker.points.push_back(pd);
  marker.points.push_back(pa);

  msg.markers.push_back(marker);
}
//}

/* addCuboid*/  //{
void BatchVisualizer::addCuboid(Eigen::Vector3d pos, Eigen::Quaterniond ori, Eigen::Vector3d sca, Eigen::Vector4d color) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "cuboid";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.position.x    = pos.x();
  marker.pose.position.y    = pos.y();
  marker.pose.position.z    = pos.z();
  marker.pose.orientation.w = ori.w();
  marker.pose.orientation.x = ori.x();
  marker.pose.orientation.y = ori.y();
  marker.pose.orientation.z = ori.z();
  marker.scale.x            = sca.x();
  marker.scale.y            = sca.y();
  marker.scale.z            = sca.z();
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::CUBE;
  marker.color.r            = color[0];
  marker.color.g            = color[1];
  marker.color.b            = color[2];
  marker.color.a            = color[3];
  msg.markers.push_back(marker);
}
//}

/* clear */  //{
void BatchVisualizer::clear() {
  msg.markers.clear();
  marker_count = 0;
}
//}

/* publish */  //{
void BatchVisualizer::publish() {
  visual_pub.publish(msg);
}
//}
