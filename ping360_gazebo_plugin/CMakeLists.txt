cmake_minimum_required(VERSION 3.0.2)
project(ping360_gazebo_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  ping360_sonar
  roscpp
  sensor_msgs
  std_msgs
)

find_package(GAZEBO REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
)


include_directories(
  ${PROJECT_SOURCE_DIR}/ping360_model
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_ODE_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(ROSLINT_CPP_OPTS "--extensions=hpp,cpp,c,hh,cc,h")

set(PING360_SRCS
  ping360_model/ping360_plugin.cc)

roslint_cpp(${PING360_SRCS})

add_library(ping_plugin ping360_model/ping360_plugin.cc)
target_link_libraries(ping_plugin ${catkin_LIBRARIES})
add_dependencies(ping_plugin ${catkin_EXPORTED_TARGETS})

install(TARGETS ping_plugin
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)