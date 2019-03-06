#ifndef RADIATION_CONSTANTS_H
#define RADIATION_CONSTANTS_H

#include <math.h>

class Constants {
public:
  constexpr static double h       = 6.6207015e-34;           // Planck constant [J * s]
  constexpr static double hr      = 6.6207015e-34 / (M_PI);  // reduced Planck constant [J * s]
  constexpr static double c       = 299792458.0;             // speed of light in vacuum [m/s]
  constexpr static double m_e     = 9.10938356e-31;          // mass of an electron [kg]
  constexpr static double alpha   = 1.0 / 137.04;            // fine structure constant [-]
  constexpr static double r_e     = 2.8179e-15;              // classical electron radius [m]
  constexpr static double si_atom = 0.222e-9;                // diameter of an silicone atom [m]
  constexpr static double N_a     = 6.02214086e23;           // Avogadro's number [1/mol]
  constexpr static double barn    = 10.0e-28;                // m^2
};
#endif
