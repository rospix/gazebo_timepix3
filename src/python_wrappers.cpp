#include <gazebo_timepix/python_wrappers.h>

/* getPhotoAbsorptionCoeff //{ */
double getPhotoAbsorptionCoeff(std::string material, double energy) {

  // initialize embedded Python
  Py_Initialize();  
  
  // this is necessary because embedded python does not know where to find the script
  PyObject *sysmodule = PyImport_ImportModule("sys");
  PyObject *syspath   = PyObject_GetAttrString(sysmodule, "path");
  std::string pkgpath = ros::package::getPath("gazebo_timepix");
  std::stringstream ss;
  ss << pkgpath << "/scripts";
  PyList_Append(syspath, PyString_FromString(ss.str().c_str()));
  Py_DECREF(syspath);
  Py_DECREF(sysmodule);

  // import the interpolation module
  PyObject *myModule = PyImport_ImportModule("material_properties");

  // load the interpolation function
  PyObject *myFunction = PyObject_GetAttrString(myModule, (char *)"photoabsorption_coeff");
  PyObject *args       = PyTuple_Pack(2, PyString_FromString(material.c_str()), PyFloat_FromDouble(energy));
  PyObject *myResult   = PyObject_CallObject(myFunction, args);

  // return the photoabsorption coefficient
  return PyFloat_AsDouble(myResult);
}
//}
