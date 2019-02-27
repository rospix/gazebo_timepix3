#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <vector>
#include <boost/optional.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Ray {
public:
  Ray();
  ~Ray();
  Ray(Eigen::Vector3d origin, Eigen::Vector3d direction, double energy = 0.0);

  double          energy;
  Eigen::Vector3d origin;
  Eigen::Vector3d direction;

  static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo) {
    Eigen::Vector3d origin    = pointFrom;
    Eigen::Vector3d direction = (pointTo - pointFrom);
    direction.normalize();
    return Ray(origin, direction);
  }
  static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction) {
    return Ray(origin, direction);
  }
};

class Plane {
public:
  Plane(Eigen::Vector3d point, Eigen::Vector3d normal);
  Plane();
  ~Plane();

  Eigen::Vector3d point;
  Eigen::Vector3d normal;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};


class Rectangle {
public:
  Rectangle(std::vector<Eigen::Vector3d> points);
  ~Rectangle();

  Eigen::Vector3d normal_vector;
  Eigen::Vector3d zero_point;

  Eigen::Matrix3d basis;
  Eigen::Matrix3d projector;

  Plane                        plane;
  std::vector<Eigen::Vector3d> points;
};

#endif
