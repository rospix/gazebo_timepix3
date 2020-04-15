#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/package.h>
#include <gazebo_timepix/data_utils.h>

Table loadNistTable(std::string material) {
  Table             table;
  std::stringstream ss;
  std::string       package_path = ros::package::getPath("gazebo_timepix");
  ss << package_path.c_str() << "/nist/" << material.c_str() << ".csv";
  std::ifstream nist_file;
  nist_file.open(ss.str().c_str());
  if (!nist_file.is_open()) {
    std::stringstream msg;
    msg << "Table \"" << material << ".csv\" was not found in folder \"" << package_path << "/nist/\" not found!";
    throw std::runtime_error(msg.str());
  }
  int lines = 0;
  while (true) {
    std::string line;
    getline(nist_file, line);
    std::vector<std::string> line_elems;
    boost::split(line_elems, line, [](char c) { return c == ','; });
    if (line_elems.size() != 5) {
      if (line_elems.size() <= 1 && lines > 0) {
        break;
      }

      std::stringstream msg;
      msg << "Table \"" << material << ".csv\" in folder \"" << package_path << "/nist/\" is empty or not formatted properly! Expected 4 columns, got "
          << line_elems.size() - 1;
      throw std::runtime_error(msg.str());
    }
    std::vector<std::string> line_slice(line_elems.begin(), line_elems.end() - 1);
    table.push_back(line_slice);
    lines++;
  }
  return table;
}
