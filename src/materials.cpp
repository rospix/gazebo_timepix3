#include <materials.h>

Material::Material() {
}

Material::~Material() {
}

Material::Material(std::string name, double density, double mac60kev, double mac600kev) {

  this->name      = name;
  this->density   = density;
  this->mac60kev  = mac60kev; // mass attenuation coeff for Am241 photons
  this->mac600kev = mac600kev; // mass attenuation coeff for Cs137 photons
};
