#include <gazebo_timepix/python_wrappers.h>

/* getMaterialProperties //{ */
std::vector<double> getMaterialProperties(std::string material, double energy) {

  // initialize embedded Python
  Py_Initialize();

  // this is necessary because embedded python does not know where to find the script
  PyObject *        sysmodule = PyImport_ImportModule("sys");
  PyObject *        syspath   = PyObject_GetAttrString(sysmodule, "path");
  std::string       pkgpath   = ros::package::getPath("gazebo_timepix");
  std::stringstream ss;
  ss << pkgpath << "/scripts";
  PyList_Append(syspath, PyString_FromString(ss.str().c_str()));
  Py_DECREF(syspath);
  Py_DECREF(sysmodule);

  // import the interpolation module
  PyObject *my_module = PyImport_ImportModule("material_properties");

  // load the interpolation function
  PyObject *my_function = PyObject_GetAttrString(my_module, (char *)"get_material_properties");
  PyObject *args        = PyTuple_Pack(2, PyString_FromString(material.c_str()), PyFloat_FromDouble(energy));
  PyObject *my_result   = PyObject_CallObject(my_function, args);

  // return the photoabsorption coefficient and density
  std::vector<double> ret;
  ret.push_back(PyFloat_AsDouble(PyTuple_GetItem(my_result, 0)));
  ret.push_back(PyFloat_AsDouble(PyTuple_GetItem(my_result, 1)));
  return ret;
}
//}
