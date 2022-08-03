#include "uav_mpc/ros_mission.h"

Eigen::Vector3f goal;
Eigen::Quaternionf Q;
Eigen::Vector3f eulerAngle;

Eigen::MatrixXd reference(Nreference, Ksample + 1);

MPCRos::MPCRos(ros::NodeHandle &nh):nh(nh)
{
  nh.getParam("/mass", mass);
  nh.getParam("/hover_thrust", hover_thrust);
  nh.getParam("/takeoff_height", takeoff_height);
  nh.getParam("/ctrl_hz", ctrl_hz);
  nh.getParam("/odomTopicName", odomTopicName);

  mpc_mode = AUTO_TAKEOFF;
  reference = Eigen::MatrixXd::Zero(Nreference, Ksample + 1);
}

MPCRos::~MPCRos()
{
}

void MPCRos::ExectControl()
{
  arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("/mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
  odom_sub = nh.subscribe<nav_msgs::Odometry> (odomTopicName, 1, &MPCRos::odom_Callback, this);
  state_sub = nh.subscribe<mavros_msgs::State> ("/mavros/state", 10, &MPCRos::state_Callback, this);
  traj_sub = nh.subscribe<quadrotor_msgs::mpc_ref_traj> ("/mpc_ref_traj", 1, &MPCRos::traj_Callback, this);
  cmd_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/mavros/setpoint_raw/attitude", 1);

  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;

  ros::Rate rate(ctrl_hz);

  wrapper = new MPCWrapper(nh);
  last_request = ros::Time::now();

  while(ros::ok())
  {
    FSMProcess();
    ros::spinOnce();
    rate.sleep();
  }
}

void MPCRos::FSMProcess()
{
  if(odom_flag)
  {
    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0)))
    {
      if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        ROS_INFO("Offboard enabled");
      last_request = ros::Time::now();
    } 
    else 
    {
      if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
      {
        if(arming_client.call(arm_cmd) && arm_cmd.response.success)
          ROS_INFO("Vehicle armed");
        last_request = ros::Time::now();
      }
    }

    if(!mpc_init)
    {
      mpc_init = wrapper->initSolver(start_odom);
      ROS_INFO("MPC_INIT");
    }

    if(mpc_init)
    {
      reference = Eigen::MatrixXd::Zero(Nreference, Ksample + 1);
      switch(mpc_mode)
      {
        case AUTO_HOVER:
          if(fsm_switch) 
          {
            ROS_INFO("AUTO_HOVER");
            fsm_switch = 0;
          }
          Q = Eigen::Quaternionf (hover_odom.pose.pose.orientation.w,
                                  hover_odom.pose.pose.orientation.x,
                                  hover_odom.pose.pose.orientation.y,
                                  hover_odom.pose.pose.orientation.z);  
          eulerAngle = Q.matrix().eulerAngles(2,1,0);
          if(eulerAngle(0)>1.5707963)
            eulerAngle(0) = eulerAngle(0) - 3.1415926;
          for(int i = 0; i < Ksample + 1; ++i)
          {
            reference.col(i) << hover_odom.pose.pose.position.x, hover_odom.pose.pose.position.y, hover_odom.pose.pose.position.z,
                                cos(eulerAngle(0)/2), 0, 0, sin(eulerAngle(0)/2),
                                0, 0, 0,
                                9.8066, 0, 0, 0;
          }
          wrapper->gerReference(reference);
          if(wrapper->getSolution(current_odom, control))
            publishcontrol();
          else
            exit(0);
          break;

        case AUTO_TRACKING:
          if(fsm_switch) 
          {
            ROS_INFO("AUTO_TRACKING");
            fsm_switch = 0;
          }
          getTrajRef();
          if(reachgoal(current_odom, goal))
          {
            mpc_mode = AUTO_HOVER;
            fsm_switch = 1;
          }
          wrapper->gerReference(reference);
          if(wrapper->getSolution(current_odom, control))
            publishcontrol();
          else
          {
            mpc_mode = AUTO_HOVER;
            fsm_switch = 1;
            ROS_ERROR("NO_Solution. Turn to HOVER Mode");
          }
          break;

        case AUTO_TAKEOFF:
          if(fsm_switch) 
          {
            ROS_INFO("AUTO_TAKEOFF");
            fsm_switch = 0;
          }
          for(int i = 0; i < Ksample + 1; ++i)
          {
            reference.col(i) << start_odom.pose.pose.position.x, start_odom.pose.pose.position.y, takeoff_height,
                                start_odom.pose.pose.orientation.w, start_odom.pose.pose.orientation.x, start_odom.pose.pose.orientation.y, start_odom.pose.pose.orientation.z,
                                0, 0, 0,
                                9.8066, 0, 0, 0;
          }
          Eigen::Vector3f hover_point(start_odom.pose.pose.position.x, start_odom.pose.pose.position.y, takeoff_height);
          if(reachgoal(current_odom, hover_point))
          {
            mpc_mode = AUTO_HOVER;
            fsm_switch = 1;
          }
          wrapper->gerReference(reference);
          if(wrapper->getSolution(current_odom, control))
            publishcontrol();
          // if(control[0] > 0.45 * 9.8066 / hover_thrust)
          //   control[0] = 0.45 * 9.8066 / hover_thrust;
          break;
      }
    }
  }
  else
  {
    ROS_WARN("No Odom");
  }
 
}

void MPCRos::odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(!odom_flag)
  {
    start_odom = *msg;
    ROS_INFO("Odom Recieved");
  }

  odom_flag = 1;
  current_odom = *msg;
}

void MPCRos::state_Callback(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}


void MPCRos::traj_Callback(const quadrotor_msgs::mpc_ref_traj::ConstPtr& msg)
{
  ROS_INFO("Trajectory Recieved");
  traj_msg = *msg;

  goal << msg->goal.x, msg->goal.y, msg->goal.z;
  
  if(mpc_mode == AUTO_HOVER && !(reachgoal(current_odom, goal)))
  {
    mpc_mode = AUTO_TRACKING;
    fsm_switch = 1;
  }
}

void MPCRos::getTrajRef()
{
  double last_px = current_odom.pose.pose.position.x;
  double last_py = current_odom.pose.pose.position.y;
  double next_px, next_py;
  double yaw;
  double last_yaw = 0;
  Eigen::Vector4d quat;
  Eigen::Vector3d acc;
  int k = 0;
                        
  for(auto point:traj_msg.mpc_ref_points)
  {
    // calculate orientation according to yaw and acc
    next_px = point.position.x;
    next_py = point.position.y;
    if(next_px == last_px && next_py == last_py)
      yaw = last_yaw;
    else
      yaw = acos((next_px - last_px)/sqrt((next_px - last_px)*(next_px - last_px) + (next_py - last_py)*(next_py - last_py)));
    if((next_px - last_px)<0)
      yaw = -yaw;
    last_yaw = yaw;
    last_px = next_px;
    last_py = next_py;
    acc << point.acceleration.x, point.acceleration.y, point.acceleration.z - 9.8066;
    acc2quaternion(acc, yaw, quat);
    // ref
    reference.col(k).setZero();
    reference.col(k) << point.position.x, point.position.y, point.position.z,
                        quat[0], quat[1], quat[2], quat[3],
                        point.velocity.x, point.velocity.y, point.velocity.z,
                        0, 0, 0, 0;
    k++;
  }
}

void MPCRos::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw, Eigen::Vector4d &quat) 
{
  Eigen::Vector3d zb_des, yb_des, xb_des, yc;
  Eigen::Matrix3d R;

  yc = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*Eigen::Vector3d::UnitY();
  zb_des = vector_acc / vector_acc.norm();
  xb_des = yc.cross(zb_des) / ( yc.cross(zb_des) ).norm();
  yb_des = zb_des.cross(xb_des) / (zb_des.cross(xb_des)).norm();

  R << xb_des(0), yb_des(0), zb_des(0), 
       xb_des(1), yb_des(1), zb_des(1), 
       xb_des(2), yb_des(2), zb_des(2);

  double tr = R.trace();
  if (tr > 0.0) 
  {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat[0] = 0.25 * S;
    quat[1] = (R(2, 1) - R(1, 2)) / S;
    quat[2] = (R(0, 2) - R(2, 0)) / S;
    quat[3] = (R(1, 0) - R(0, 1)) / S;
  } 
  else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) 
  {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat[0] = (R(2, 1) - R(1, 2)) / S;
    quat[1] = 0.25 * S;
    quat[2] = (R(0, 1) + R(1, 0)) / S;
    quat[3] = (R(0, 2) + R(2, 0)) / S;
  } 
  else if (R(1, 1) > R(2, 2)) 
  {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat[0] = (R(0, 2) - R(2, 0)) / S;
    quat[1] = (R(0, 1) + R(1, 0)) / S;
    quat[2] = 0.25 * S;
    quat[3] = (R(1, 2) + R(2, 1)) / S;
  } 
  else 
  {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat[0] = (R(1, 0) - R(0, 1)) / S;
    quat[1] = (R(0, 2) + R(2, 0)) / S;
    quat[2] = (R(1, 2) + R(2, 1)) / S;
    quat[3] = 0.25 * S;
  }
}

bool MPCRos::reachgoal(nav_msgs::Odometry& msg, Eigen::Vector3f& goal)
{
  double distance;
  distance = (msg.pose.pose.position.x - goal[0]) * (msg.pose.pose.position.x - goal[0]) + 
             (msg.pose.pose.position.y - goal[1]) * (msg.pose.pose.position.y - goal[1]) + 
             (msg.pose.pose.position.z - goal[2]) * (msg.pose.pose.position.z - goal[2]);

  if(distance < 0.08) 
  {
    hover_odom = current_odom;
    return true;
  }
  else
    return false;
}

void MPCRos::publishcontrol()
{
  double thrust = control[0] * hover_thrust / 9.8066;

  mavros_msgs::AttitudeTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = std::string("FCU");
  cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  cmd.body_rate.x = control[1];
  cmd.body_rate.y = control[2];
  cmd.body_rate.z = control[3];
  cmd.thrust = thrust;

  // std::cout << "Input:\n" << thrust << std::endl 
  //                         << control[1] << std::endl 
  //                         << control[2] << std::endl 
  //                         << control[3] << std::endl;
  cmd_pub.publish(cmd);
}