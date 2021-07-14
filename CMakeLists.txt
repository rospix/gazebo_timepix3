cmake_minimum_required(VERSION 3.1.2)
project(gazebo_timepix3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  gazebo_ros
  rad_utils
  mrs_msgs
  mrs_lib
  gazebo_rad_msgs
  )

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp std_msgs mrs_msgs mrs_lib gazebo_ros gazebo_rad_msgs rad_utils
  LIBRARIES SourceAbstraction ObstacleAbstraction
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(SourceAbstraction SHARED src/source_abstaction.cpp)
target_link_libraries(SourceAbstraction ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(SourceAbstraction ${catkin_EXPORTED_TARGETS})

add_library(ObstacleAbstraction SHARED src/obstacle_abstraction.cpp)
target_link_libraries(ObstacleAbstraction ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(ObstacleAbstraction ${catkin_EXPORTED_TARGETS})

add_library(gazebo_timepix3 SHARED src/gazebo_timepix3.cpp)
target_link_libraries(gazebo_timepix3 SourceAbstraction ObstacleAbstraction ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(gazebo_timepix3 ${catkin_EXPORTED_TARGETS})
