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
const static Material Si   = Material("Si", 2.329, 0.325031909908, 0.0772599124249);
const static Material Cd   = Material("Cd", 8.65, 6.15220721228, 0.0752410682746);
const static Material Te   = Material("Te", 6.24, 7.0963E+00, 3.440E-02);
const static Material CdTe = Material("CdTe", 6.2, 6.66938158906, 0.0747982314383);
const static Material C    = Material("C", 1.7, 0.175791379779, 0.0771243158107);
const static Material Na   = Material("Na", 0.968, 5.5004E-02, 2.830E-02);
const static Material I    = Material("I", 4.933, 7.6433E+00, 3.598E-02);

// Compounds
const static Material concrete = Material("concrete", 2.3, 2.66E-01, 6.236E-02);
const static Material wood     = Material("wood", 0.7, 1.907E-01, 8.070E-02);
const static Material glass    = Material("glass", 2.23, 2.417E-01, 8.035E-02);
const static Material air      = Material("air", 1.205E-3, 0.188309591009, 0.0770994336217);
const static Material water    = Material("water", 1.0, 2.059E-01, 8.956E-02);
//}

#endif
