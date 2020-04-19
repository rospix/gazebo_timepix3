#ifndef TIMEPIX_DATA_UTILS_H
#define TIMEPIX_DATA_UTILS_H

#include <vector>
#include <string>
#include <sstream>
#include <exception>
#include <cmath>


enum AttenuationType
{
  MASS        = 1,
  MASS_ENERGY = 2,
};

typedef struct std::vector<std::vector<double>> Table;

Table  loadNistTable(std::string material);
double interpolateAttenuation(Table table, AttenuationType type, double photon_energy);

double calculateMassAttCoeff(double photon_energy, std::string material, AttenuationType type);
double calculateAbsorptionProb(double track_length, double attenuation_coeff, double material_density);
#endif
