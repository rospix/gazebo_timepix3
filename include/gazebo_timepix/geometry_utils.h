#ifndef RADIATION_UTILS_GEOMETRY_H
#define RADIATION_UTILS_GEOMETRY_H

#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/optional.hpp>
#include <ignition/math.hh>

/* Ray //{ */
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

Ray::Ray() {
  this->p1 = Eigen::Vector3d(0.0, 0.0, 0.0);
  this->p2 = Eigen::Vector3d(0.0, 0.0, 0.0);
}

Ray::~Ray() {
}

Ray::Ray(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  this->p1        = p1;
  this->p2        = p2;
  this->direction = p2 - p1;
}
//}

/* Plane //{ */
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

Plane::Plane() {
}

Plane::Plane(Eigen::Vector3d point, Eigen::Vector3d normal) {
  this->point  = point;
  this->normal = normal;
  this->d      = -(normal.dot(point));
}

Plane::~Plane() {
}

boost::optional<Eigen::Vector3d> Plane::intersectionRay(Ray r, double epsilon) {
  double denom = this->normal.dot(r.p2 - r.p1);
  if (abs(denom) < epsilon) {
    return boost::optional<Eigen::Vector3d>{};
  }
  double t = this->normal.dot(this->point - r.p1) / denom;
  if (t >= 0) {
    return Eigen::Vector3d(r.p1 + t * r.direction);
  } else {
    return boost::optional<Eigen::Vector3d>{};
  }
}

//}

/* Rectangle //{ */
class Rectangle {
public:
  Rectangle();
  ~Rectangle();
  Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);

  Eigen::Vector3d normal_vector;

  Eigen::Matrix3d basis;
  Eigen::Matrix3d projector;

  double diagonal_length;

  Plane plane;

  std::vector<Eigen::Vector3d> points;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};

Rectangle::Rectangle() {
}

Rectangle::~Rectangle() {
}

Rectangle::Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D) {

  Eigen::Vector3d v1 = B - A;
  Eigen::Vector3d v2 = D - A;

  diagonal_length = (C - A).norm();

  this->points.push_back(A);
  this->points.push_back(B);
  this->points.push_back(C);
  this->points.push_back(D);

  this->normal_vector = v1.cross(v2);
  this->normal_vector.normalize();

  if (A == B || A == C || A == D || B == C || B == D || C == D) {
    return;
  }

  this->plane = Plane(A, normal_vector);

  this->basis.col(0) << B - A;
  this->basis.col(1) << D - A;
  this->basis.col(2) << normal_vector;

  this->projector = basis * basis.transpose();
}

boost::optional<Eigen::Vector3d> Rectangle::intersectionRay(Ray r, double epsilon) {
  boost::optional<Eigen::Vector3d> intersect = this->plane.intersectionRay(r, epsilon);
  if (!intersect) {
    return intersect;
  }
  Eigen::Vector3d projection = basis.inverse() * (intersect.get() - points[0]);
  if (projection[0] >= 0.0 && projection[0] <= 1.0 && projection[1] >= 0.0 && projection[1] <= 1.0) {
    return intersect;
  }
  return boost::optional<Eigen::Vector3d>{};
}
//}

/* Cuboid //{ */
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

Cuboid::Cuboid() {
}

Cuboid::~Cuboid() {
}

Cuboid::Cuboid(Eigen::Vector3d center, Eigen::Quaterniond orientation, Eigen::Vector3d scale) {

  Eigen::Vector3d A = orientation * Eigen::Vector3d(scale[0] / 2.0, -scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d B = orientation * Eigen::Vector3d(scale[0] / 2.0, scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d C = orientation * Eigen::Vector3d(scale[0] / 2.0, scale[1] / 2.0, scale[2] / 2.0) + center;
  Eigen::Vector3d D = orientation * Eigen::Vector3d(scale[0] / 2.0, -scale[1] / 2.0, scale[2] / 2.0) + center;

  Eigen::Vector3d E = orientation * Eigen::Vector3d(-scale[0] / 2.0, scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d F = orientation * Eigen::Vector3d(-scale[0] / 2.0, -scale[1] / 2.0, -scale[2] / 2.0) + center;
  Eigen::Vector3d G = orientation * Eigen::Vector3d(-scale[0] / 2.0, -scale[1] / 2.0, scale[2] / 2.0) + center;
  Eigen::Vector3d H = orientation * Eigen::Vector3d(-scale[0] / 2.0, scale[1] / 2.0, scale[2] / 2.0) + center;

  Rectangle front(A, B, C, D);
  Rectangle back(E, F, G, H);
  Rectangle left(B, E, H, C);
  Rectangle right(F, A, D, G);
  Rectangle bottom(F, E, B, A);
  Rectangle top(D, C, H, G);

  sides.push_back(front);
  sides.push_back(back);
  sides.push_back(left);
  sides.push_back(right);
  sides.push_back(bottom);
  sides.push_back(top);

  this->vertices.clear();
  this->vertices.push_back(A);
  this->vertices.push_back(B);
  this->vertices.push_back(C);
  this->vertices.push_back(D);
  this->vertices.push_back(E);
  this->vertices.push_back(F);
  this->vertices.push_back(G);
  this->vertices.push_back(H);
}

Cuboid::Cuboid(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D, Eigen::Vector3d E, Eigen::Vector3d F, Eigen::Vector3d G,
               Eigen::Vector3d H) {
  this->vertices.clear();
  this->vertices.push_back(A);
  this->vertices.push_back(B);
  this->vertices.push_back(C);
  this->vertices.push_back(D);
  this->vertices.push_back(E);
  this->vertices.push_back(F);
  this->vertices.push_back(G);
  this->vertices.push_back(H);

  Rectangle front(A, B, C, D);
  Rectangle back(E, F, G, H);
  Rectangle left(B, E, H, C);
  Rectangle right(F, A, D, G);
  Rectangle bottom(F, E, B, A);
  Rectangle top(D, C, H, G);

  sides.push_back(front);
  sides.push_back(back);
  sides.push_back(left);
  sides.push_back(right);
  sides.push_back(bottom);
  sides.push_back(top);
}
//}

/* Miscellaneous */  //{
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

Eigen::Quaterniond pos3toQuaterniond(ignition::math::Pose3d gzpos) {
  Eigen::Quaterniond q(gzpos.Rot().W(), gzpos.Rot().X(), gzpos.Rot().Y(), gzpos.Rot().Z());
  return q;
}

Eigen::Vector3d targetRelativePosition(ignition::math::Pose3d my_pose, Eigen::Vector3d target_pos) {

  ignition::math::Vector3d    p = my_pose.Pos();
  ignition::math::Quaterniond q = my_pose.Rot();

  Eigen::Vector3d    my_world_pos(p.X(), p.Y(), p.Z());
  Eigen::Quaterniond local2world = Eigen::Quaterniond(q.W(), q.X(), q.Y(), q.Z());
  Eigen::Quaterniond world2local = local2world.inverse();

  return world2local * (target_pos - my_world_pos);
}

Rectangle move(Rectangle r, Eigen::Vector3d translation, Eigen::Quaterniond rotation) {
  Rectangle ret;
  for (int i = 0; i < 4; i++) {
    ret.points.push_back((rotation * r.points[i]) + translation);
  }
  Eigen::Vector3d v1 = ret.points[1] - ret.points[0];
  Eigen::Vector3d v2 = ret.points[3] - ret.points[0];

  ret.normal_vector = v1.cross(v2);
  ret.normal_vector.normalize();

  ret.plane = Plane(ret.points[0], ret.normal_vector);

  ret.basis.col(0) << ret.points[1] - ret.points[0];
  ret.basis.col(1) << ret.points[3] - ret.points[0];
  ret.basis.col(2) << ret.normal_vector;

  ret.projector = ret.basis * ret.basis.transpose();

  return ret;
}
//}

#endif /* RADIATION_UTILS_GEOMETRY_H */

