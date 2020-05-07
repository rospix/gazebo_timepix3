cmake_minimum_required(VERSION 2.8.3)
project(gazebo_timepix)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED
  roscpp
  std_msgs
  gazebo_ros
  radiation_utils
  gazebo_rad_msgs
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS gazebo_ros gazebo_rad_msgs radiation_utils
  CATKIN_DEPENDS roscpp std_msgs
  # LIBRARIES radiation_utils
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_timepix SHARED src/gazebo_timepix.cpp)
target_link_libraries(gazebo_timepix ${catkin_LIBRARIES})
add_dependencies(gazebo_timepix ${catkin_EXPORTED_TARGETS})
