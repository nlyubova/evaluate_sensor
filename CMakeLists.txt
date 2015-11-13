cmake_minimum_required(VERSION 2.8.3)
SET(PROJECTNAME evaluate_sensor)

project(${PROJECTNAME})

find_package(Boost QUIET COMPONENTS program_options signals)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  cv_bridge
  image_transport
  geometry_msgs
  tf
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport geometry_msgs
)
# pcl_conversions pcl_ros
###########
## Build ##
###########

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

SET(SRC
  src/main.cpp
  src/evaluate.cpp
  src/evaluate_pc.cpp
  include/evaluate_sensor/evaluate.hpp
)

add_executable(${PROJECTNAME} ${SRC})

 target_link_libraries(${PROJECTNAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${PCL_LIBRARIES}
 )
