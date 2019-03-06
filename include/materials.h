#ifndef RADIATION_MATERIALS_H
#define RADIATION_MATERIALS_H

#include <string>
#include <vector>
#include <math.h>
#include <radiation_constants.h>

class Element {
public:
  Element();
  ~Element();
  Element(std::string name, double density, double atomic_number, double molar_mass);

  std::string name             = "";
  double      density          = 0;
  double      atomic_number    = 0;
  double      molar_mass       = 0;
  double      electron_density = 0;
  double      atomic_density   = 0;
};

const static Element Si_element = Element("Si", 2320.0, 14.0, 0.02808550);
const static Element Cd_element = Element("Cd", 8650.0, 48.0, 0.112411);
const static Element Na_element = Element("Cd", 968.0, 11.0, 0.022989770);
const static Element I_element  = Element("I", 4933.0, 53.0, 0.253808940);
const static Element Te_element = Element("Te", 5850.0, 52.0, 0.12760);

class Material {
public:
  Material();
  ~Material();
  Material(std::string name, std::vector<Element> elements, std::vector<double> element_quantities, double density, double molar_mass);

  std::string          name = "";
  std::vector<Element> elements;
  std::vector<double>  element_quantities;
  double               density           = 0;
  double               molar_mass        = 0;
  double               electron_density  = 0;
  double               molecular_density = 0;
  double               atomic_number     = 0;
};

const static Material Si   = Material("Si", std::vector<Element>({Si_element}), std::vector<double>({1.0}), 2320.0, 0.02808550);
const static Material Cd   = Material("Cd", std::vector<Element>({Cd_element}), std::vector<double>({1.0}), 8650.0, 0.112411);
const static Material Na   = Material("Na", std::vector<Element>({Na_element}), std::vector<double>({1.0}), 968.0, 0.022989770);
const static Material I    = Material("I", std::vector<Element>({I_element}), std::vector<double>({1.0}), 4933.0, 0.253808940);
const static Material Te   = Material("Te", std::vector<Element>({Te_element}), std::vector<double>({1.0}), 5850.0, 0.12760);
const static Material CdTe = Material("CdTe", std::vector<Element>({Cd_element, Te_element}), std::vector<double>({1.0, 1.0}), 5850.0, 0.2400110);
const static Material NaI  = Material("NaI", std::vector<Element>({Na_element, I_element}), std::vector<double>({1.0, 1.0}), 3670.0, 0.149894239);

#endif
