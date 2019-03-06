#include <materials.h>

Element::Element() {
}

Element::~Element() {
}

Element::Element(std::string name, double density, double atomic_number, double molar_mass) {

  this->name          = name;
  this->density       = density;
  this->atomic_number = atomic_number;
  this->molar_mass    = molar_mass;

  double atoms_in_kg     = Constants::N_a / molar_mass;
  this->electron_density = atoms_in_kg * atomic_number * density;
  this->atomic_density   = atoms_in_kg * density;
}

Material::Material() {
}

Material::~Material() {
}

Material::Material(std::string name, std::vector<Element> elements, std::vector<double> element_quantities, double density, double molar_mass) {

  this->name               = name;
  this->density            = density;
  this->molar_mass         = molar_mass;
  this->elements           = elements;
  this->element_quantities = element_quantities;

  double molecules_in_kg = Constants::N_a / molar_mass;

  double quantity_sum_over_elements = 0;
  for (size_t i = 0; i < this->elements.size(); i++) {
    quantity_sum_over_elements += elements[i].atomic_number * element_quantities[i];
  }

  this->molecular_density = molecules_in_kg * quantity_sum_over_elements * density;
}

