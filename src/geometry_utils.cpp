#include <geometry_utils.h>
#include <iostream>
#include <ros/ros.h>

Ray::Ray() {
  this->origin    = Eigen::Vector3d(0.0, 0.0, 0.0);
  this->direction = Eigen::Vector3d(0.0, 0.0, 0.0);
  this->energy    = 0.0;
}

Ray::~Ray() {
}

Ray::Ray(Eigen::Vector3d origin, Eigen::Vector3d direction, double energy) {
  this->origin    = origin;
  this->direction = direction;
  this->energy    = energy;
}

Plane::Plane() {
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
    return Eigen::Vector3d(t * r.direction + r.origin);
  } else {
    return boost::optional<Eigen::Vector3d>{};
  }
}

Rectangle::Rectangle() {
}

Rectangle::~Rectangle() {
}

Rectangle::Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D) {

  Eigen::Vector3d v1 = D - A;
  Eigen::Vector3d v2 = D - C;

  this->points.push_back(A);
  this->points.push_back(B);
  this->points.push_back(C);
  this->points.push_back(D);

  this->normal_vector = v1.cross(v2);
  this->normal_vector.normalize();

  if (A == B or A == C or A == D or B == C or B == D or C == D) {
    return;
  }

  this->plane = Plane(A, normal_vector);

  this->basis.col(0) << D - A;
  this->basis.col(1) << D - C;
  this->basis.col(2) << normal_vector;
  /* this->basis.transposeInPlace(); */

  this->projector = basis * basis.transpose();
}

double haversin(double angle) {
  return (1.0 - std::cos(angle)) / 2.0;
}

double invHaversin(double angle) {
  return 2.0 * std::asin(std::sqrt(angle));
}

double triangleArea(double a, double b, double c) {
  double s = (a + b + c) / 2.0;
  return std::sqrt(s * (s - a) * (s - b) * (s - c));
}


double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2) {
  return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

double solidAngle(double a, double b, double c) {
  return invHaversin((haversin(c) - haversin(a - b)) / (std::sin(a) * std::sin(b)));
}

double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) {
  double ab = vectorAngle(a, b);
  double bc = vectorAngle(b, c);
  double ca = vectorAngle(c, a);

  if (ab < 1e-3 and bc < 1e-3 and ca < 1e-3) {
    return triangleArea(ab, bc, ca);
  }

  double A = solidAngle(ca, ab, bc);
  double B = solidAngle(ab, bc, ca);
  double C = solidAngle(bc, ca, ab);

  return A + B + C - M_PI;
}

double rectSolidAngle(Rectangle r, Eigen::Vector3d center) {
  Eigen::Vector3d a = r.points[0] - center;
  Eigen::Vector3d b = r.points[1] - center;
  Eigen::Vector3d c = r.points[2] - center;
  Eigen::Vector3d d = r.points[3] - center;

  a.normalize();
  b.normalize();
  c.normalize();
  d.normalize();

  double t1 = sphericalTriangleArea(a, b, c);
  double t2 = sphericalTriangleArea(c, d, a);

  return t1 + t2;
}

Eigen::Vector3d pos3toVector3d(ignition::math::Pose3d gzpos) {
  Eigen::Vector3d v;
  v[0] = gzpos.Pos().X();
  v[1] = gzpos.Pos().Y();
  v[2] = gzpos.Pos().Z();
  return v;
}
