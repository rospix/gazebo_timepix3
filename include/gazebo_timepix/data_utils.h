#ifndef TIMEPIX_DATA_UTILS_H
#define TIMEPIX_DATA_UTILS_H

#include <vector>
#include <string>
#include <sstream>
#include <exception>

typedef struct std::vector<std::vector<std::string>> Table;

Table loadNistTable(std::string material);

#endif
