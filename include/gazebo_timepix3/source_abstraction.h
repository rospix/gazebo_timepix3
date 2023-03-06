#ifndef SOURCE_ABSTRACTION_H
#define SOURCE_ABSTRACTION_H

#include <string>
#include <vector>
#include <eigen3/Eigen/Core>

/* This library provides wrapper objects for the 3D objects interacting with the simulated radiation */

typedef std::pair<int, double> SideProperty;

/* SourceAbstraction //{ */

class SourceAbstraction {
private:
  unsigned int gazebo_id_;
  double       activity_;
  double       energy_;
  double       mass_att_coeff_;  // attenuation of emitted particles by the sensor
  std::string  material_;
  double       air_mass_att_coeff_;  // atttenuation of emitted particles by air

  Eigen::Vector3d           relative_position_;
  std::vector<SideProperty> side_properties_;
  // int - index of exposed Timepix side
  // double - apparent activity of this source for indexed side

  std::vector<unsigned int> obstacles_in_path_;

public:
  SourceAbstraction();
  ~SourceAbstraction();
  SourceAbstraction(unsigned int gazebo_id, std::string material, double activity, double energy, double mass_att_coeff, double air_mass_att_coeff,
                    Eigen::Vector3d relative_position);

  bool operator==(SourceAbstraction const &s1);
  bool operator==(const unsigned int &i);
  bool operator==(unsigned int &i);

  // getters
  const unsigned int              getId() const;
  const std::string               getMaterial() const;
  const double                    getActivity() const;
  const double                    getEnergy() const;
  const double                    getMassAttCoeff() const;
  const Eigen::Vector3d           getRelativePosition() const;
  const std::vector<SideProperty> getSideProperties() const;
  const std::vector<unsigned int> getObstaclesInPath() const;
  const double                    getAirMassAttCoeff() const;

  void setSideProperties(std::vector<SideProperty> side_properties);
  void setRelativePosition(Eigen::Vector3d relative_position);

  void addObstacle(unsigned int obstacle_id);
  void removeObstacle(unsigned int obstacle_id);
};

#endif /* SOURCE_ABSTRACTION_H */
