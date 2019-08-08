#ifndef TIMEPIX_GEOMETRY_UTILS_H
#define TIMEPIX_GEOMETRY_UTILS_H

#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/optional.hpp>
#include <ignition/math.hh>

class Ray {
public:
  Ray();
  ~Ray();
  Ray(Eigen::Vector3d p1, Eigen::Vector3d p2);

  Eigen::Vector3d p1;
  Eigen::Vector3d p2;
  Eigen::Vector3d direction;

  static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo) {
    return Ray(pointFrom, pointTo);
  }
  static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction) {
    return Ray(origin, origin + direction);
  }
};

class Plane {
public:
  Plane();
  ~Plane();
  Plane(Eigen::Vector3d point, Eigen::Vector3d normal);

  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  double          d = 0;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};

class Rectangle {
public:
  Rectangle();
  ~Rectangle();
  Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);

  Eigen::Vector3d normal_vector;

  Eigen::Matrix3d basis;
  Eigen::Matrix3d projector;

  Plane plane;

  std::vector<Eigen::Vector3d> points;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};

class Cuboid {
public:
  Cuboid();
  ~Cuboid();
  Cuboid(Eigen::Vector3d center, Eigen::Quaterniond orientation, Eigen::Vector3d scale);
  Cuboid(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D, Eigen::Vector3d E, Eigen::Vector3d F, Eigen::Vector3d G,
         Eigen::Vector3d H);

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Rectangle>       sides;
};

double haversin(double angle);
double invHaversin(double angle);

double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);
double solidAngle(double a, double b, double c);
double rectSolidAngle(Rectangle r, Eigen::Vector3d center);

double triangleArea(double a, double b, double c);
double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

Eigen::Vector3d    pos3toVector3d(ignition::math::Pose3d gzpos);
Eigen::Quaterniond pos3toQuaterniond(ignition::math::Pose3d gzpos);
Rectangle          move(Rectangle r, Eigen::Vector3d translation, Eigen::Quaterniond rotation);

#endif /* TIMEPIX_GEOMETRY_UTILS_H */

