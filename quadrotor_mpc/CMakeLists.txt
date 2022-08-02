cmake_minimum_required(VERSION 3.0.2)
project(uav_mpc)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  Eigen3
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uav_mpc
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_library(mpc_solver
  externals/qpoases/SRC/Bounds.cpp
  externals/qpoases/SRC/Constraints.cpp
  externals/qpoases/SRC/CyclingManager.cpp
  externals/qpoases/SRC/Indexlist.cpp
  externals/qpoases/SRC/MessageHandling.cpp
  externals/qpoases/SRC/QProblem.cpp
  externals/qpoases/SRC/QProblemB.cpp
  externals/qpoases/SRC/SubjectTo.cpp
  externals/qpoases/SRC/Utils.cpp
  externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  model/quadrotor_mpc_codegen/acado_qpoases_interface.cpp
  model/quadrotor_mpc_codegen/acado_integrator.c
  model/quadrotor_mpc_codegen/acado_solver.c
  model/quadrotor_mpc_codegen/acado_auxiliary_functions.c)
  
target_include_directories(mpc_solver PUBLIC
  model/quadrotor_mpc_codegen/
  externals/qpoases
  externals/qpoases/INCLUDE
  externals/qpoases/SRC)
  
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/model/quadrotor_mpc_codegen
  ${PROJECT_SOURCE_DIR}/externals/qpoases
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(mpc_node
  src/mpc_node.cpp
  src/mpc_wrapper.cpp
  src/ros_mission.cpp)
  
target_link_libraries(mpc_node
  mpc_solver
  ${catkin_LIBRARIES})
