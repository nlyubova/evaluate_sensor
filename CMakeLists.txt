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
  #opencv_candidate
)

find_package(Eigen REQUIRED)

find_package(OpenCV 3.0.0 REQUIRED)
#find_package(aruco REQUIRED )
#message(STATUS "OpenCV version: ${OpenCV_VERSION}")
#message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
#message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport geometry_msgs #opencv_candidate
  DEPENDS OpenCV
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
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

SET(SRC
  src/main.cpp
  src/evaluate.cpp
  src/evaluate_pc.cpp
  src/evaluate_depth.cpp
  src/ardetection.cpp
  src/plane_fitting.cpp
  src/arprocess.cpp
  include/evaluate_sensor/evaluate.hpp
  include/evaluate_sensor/evaluate_depth.hpp
  include/evaluate_sensor/evaluate_pc.hpp
  include/evaluate_sensor/ardetection.hpp
  include/evaluate_sensor/plane_fitting.hpp
  include/evaluate_sensor/arprocess.hpp
)

add_executable(${PROJECTNAME} ${SRC})

target_link_libraries(${PROJECTNAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
   #${aruco_LIBS}
 )
