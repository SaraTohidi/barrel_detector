cmake_minimum_required(VERSION 2.8.3)
project(barrel_detector)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure
  nav_msgs
  roscpp
  std_msgs
)


find_package(OpenCV)
generate_dynamic_reconfigure_options(
  cfg/BarrelDetector.cfg
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES OpenCV
  CATKIN_DEPENDS nav_msgs roscpp std_msgs
dynamic_reconfigure
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(barrel_detector_node src/barrel_detector.cpp
 include/barrel_detector/barrel_detector.h)
add_dependencies(barrel_detector_node barrel_detector_gencfg)

target_link_libraries(barrel_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )



