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
  Material(std::string name, double density, double mac_60kev, double mac_600kev);

  std::string name      = "";
  double      density   = 0;  // [g/cm^3]
  double      mac60kev  = 0;  // [cm^2/g] mass attenuation coefficient for Am241 (photon energy 59.5 keV)
  double      mac600kev = 0;  // [cm^2/g] mass attenuation coefficient for Cs137 (photon energy 662 keV)
};

/* Material library //{ */

// Elements
const static Material Si   = Material("Si", 2.329, 1.3749E-01, 2.951E-02);
const static Material Cd   = Material("Cd", 8.65, 5.9900E+00, 3.339E-02);
const static Material Na   = Material("Na", 0.968, 5.5004E-02, 2.830E-02);
const static Material I    = Material("I", 4.933, 7.6433E+00, 3.598E-02);
const static Material Te   = Material("Te", 6.24, 7.0963E+00, 3.440E-02);
const static Material CdTe = Material("CdTe", 5.85, 4.163E+00, 3.391E-02);
const static Material C    = Material("C", 1.7, 1.753E-01, 8.058E-02);

// Compounds
const static Material concrete = Material("concrete", 2.3, 2.66E-01, 6.236E-02);
const static Material glass    = Material("glass", 2.23, 2.417E-01, 8.035E-02);
const static Material air      = Material("air", 1.205E-3, 1.875E-01, 8.055E-02);
//}

#endif
