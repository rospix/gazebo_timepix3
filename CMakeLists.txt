cmake_minimum_required(VERSION 2.8.3)
project(gazebo_timepix)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color")

find_package(catkin REQUIRED
  roscpp
  roslib
  std_msgs
  gazebo_ros
  gazebo_rad_msgs
)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(gazebo REQUIRED)

# include Python
find_package(PythonLibs REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS gazebo_ros roslib gazebo_rad_msgs
  CATKIN_DEPENDS roscpp std_msgs 
)

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/python_wrappers.cpp
  )

target_link_libraries(
  ${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
  )

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  )
