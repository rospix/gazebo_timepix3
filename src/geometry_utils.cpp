#include <geometry_utils.h>
#include <iostream>
#include <ros/ros.h>

Ray::Ray() {
  this->origin    = Eigen::Vector3d(0.0, 0.0, 0.0);
  this->direction = Eigen::Vector3d(0.0, 0.0, 0.0);
  this->energy    = 0.0;
}

Ray::Ray(Eigen::Vector3d origin, Eigen::Vector3d direction, double energy) {
  this->origin    = origin;
  this->direction = direction;
  this->energy    = energy;
}

Ray::~Ray() {
}

Plane::Plane(Eigen::Vector3d point, Eigen::Vector3d normal) {
  this->point  = point;
  this->normal = normal;
}

Plane::~Plane() {
}

boost::optional<Eigen::Vector3d> Plane::intersectionRay(Ray r, double epsilon) {
  double denom = this->normal.dot(r.direction);
  if (abs(denom) < epsilon) {
    return boost::optional<Eigen::Vector3d>{};
  }
  Eigen::Vector3d p0l0 = this->point - r.origin;
  double          t    = this->normal.dot(p0l0) / denom;
  if (t >= 0) {
    return boost::optional<Eigen::Vector3d>{t * r.direction + r.origin};
  } else {
    return boost::optional<Eigen::Vector3d>{};
  }
}

Rectangle::Rectangle(std::vector<Eigen::Vector3d> points) {
  if (points.size() != 4) {
    std::cout << "Rectangle: can only create rectangle with 4 points!\n";
  }

  Eigen::Vector3d v1 = points[1] - points[0];
  Eigen::Vector3d v2 = points[2] - points[0];

  this->zero_point = points[0];

  this->normal_vector = v1.cross(v2);
  this->normal_vector.normalize();

  this->plane = Plane(zero_point, normal_vector);

  Eigen::Vector3d b1;
  if (abs(normal_vector[0]) > 1e-3 or abs(normal_vector[1]) > 1e-3) {
    b1 = Eigen::Vector3d(-normal_vector[1], normal_vector[0], normal_vector[2]);
  } else {
    b1 = Eigen::Vector3d(normal_vector[0], -normal_vector[2], normal_vector[1]);
  }
  Eigen::Vector3d b2 = normal_vector.cross(b1);
  b2.normalize();

  this->basis.col(0) << b1;
  this->basis.col(1) << b2;
  this->basis.row(2) << normal_vector;

  this->projector = basis * basis.transpose();
}
