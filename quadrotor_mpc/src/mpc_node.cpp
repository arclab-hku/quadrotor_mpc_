#include "ros/ros.h"

#include <stdio.h>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

#include "uav_mpc/ros_mission.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "rpg_mpc");
  ros::NodeHandle nh("~");

  MPCRos mpcros(nh);
  mpcros.ExectControl();
  
  ros::spin();

  return 0;
}
