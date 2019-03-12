#ifndef RADIATION_MATERIALS_H
#define RADIATION_MATERIALS_H

#include <string>
#include <vector>
#include <math.h>
#include <radiation_constants.h>

class Material {
public:
  Material();
  ~Material();
  Material(std::string name, double density, double pmac_Am241, double pmac_Cs137);

  std::string name       = "";
  double      density    = 0;  // [g/cm^3]
  double      pmac_Am241 = 0;  // [cm^2/g] photoel. mass att. coeff. for Am241 (59.5keV)
  double      pmac_Cs137 = 0;  // [cm^2/g] photoel. mass att. coeff. for Cs137 (662 keV)
};

const static Material Si   = Material("Si", 2.329, 1.3749E-01, 2.951E-02);
const static Material Cd   = Material("Cd", 8.65, 5.9900E+00, 3.339E-02);
const static Material Na   = Material("Na", 0.968, 5.5004E-02, 2.830E-02);
const static Material I    = Material("I", 4.933, 7.6433E+00, 3.598E-02);
const static Material Te   = Material("Te", 6.24, 7.0963E+00, 3.440E-02);
const static Material CdTe = Material("CdTe", 5.85, 4.163E+00, 3.391E-02);

#endif
