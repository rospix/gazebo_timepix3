#include <materials.h>

Material::Material() {
}

Material::~Material() {
}

Material::Material(std::string name, double density, double pmac_Am241, double pmac_Cs137) {

  this->name       = name;
  this->density    = density;
  this->pmac_Am241 = pmac_Am241;
  this->pmac_Cs137 = pmac_Cs137;
};
