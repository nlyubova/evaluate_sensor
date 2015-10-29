cmake_minimum_required(VERSION 2.8.3)
SET(PROJECTNAME evaluate_sensor)

project(${PROJECTNAME})

find_package(Boost QUIET COMPONENTS program_options signals)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

SET(SRC
  src/main.cpp
  src/evaluate.cpp
  include/evaluate_sensor/evaluate.hpp
)

add_executable(${PROJECTNAME} ${SRC})

 target_link_libraries(${PROJECTNAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )