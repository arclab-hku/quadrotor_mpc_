cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_msgs)


find_package(catkin REQUIRED
  COMPONENTS
  geometry_msgs
  message_generation)

add_message_files(
  FILES
  mpc_ref_point.msg
  mpc_ref_traj.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)

