cmake_minimum_required(VERSION 2.8.3)
project(trajectory_follower)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  #autogo_msgs
  tf
  tf_conversions
)


find_package(Eigen3 REQUIRED)


## Generate added messages and services with any dependencies listed here


 generate_dynamic_reconfigure_options(
 	cfg/TrajectoryFollowerReconfigure.cfg
       #cfg/MkzController.cfg
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_follower 
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime dynamic_reconfigure autogo_msgs tf tf_conversions
)


SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")


include_directories(
  include
  /usr/local/include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


 
add_library(trajectory_follower 
    src/mpc/mpc_mkz.cpp
    src/mpc/mpc_components.cpp
    src/mpc/mpc_viz.cpp
    src/trajectory_follower_config.cpp
)

add_dependencies(trajectory_follower 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

add_executable(mpc_node 
src/mpc/mpc_core_mkz.cpp 
src/mpc/mpc_mkz.cpp
src/mpc/mpc_components.cpp
)

target_link_libraries(mpc_node
  trajectory_follower
  ${catkin_LIBRARIES}
  ipopt
)

add_dependencies(mpc_node
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)



