#ifndef ROSMission_H
#define ROSMission_H

#include <math.h>
// Ros
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <quadrotor_msgs/mpc_ref_point.h>
#include <quadrotor_msgs/mpc_ref_traj.h>
// Eigen
#include <Eigen/Eigen>
// MPC Solver
#include "uav_mpc/mpc_wrapper.h"

#define Ksample    20
#define Nreference 14

enum MPCMode
{
  AUTO_TAKEOFF,
  AUTO_HOVER,
  AUTO_TRACKING
};

class MPCRos
{
  // Parameters
  private:
    double mass = 1.5;
    double hover_thrust = 0.4;
    double takeoff_height = 1.3;
    double ctrl_hz = 100;
    std::string odomTopicName;

  private:
    ros::NodeHandle &nh;
    
    bool odom_flag = 0; 
    bool fsm_switch = 1;
    bool mpc_init = 0;  
    bool reached = 0;
    bool has_solution;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::State current_state;
    nav_msgs::Odometry current_odom, start_odom, hover_odom;
    geometry_msgs::PoseStamped nav_goal;
    quadrotor_msgs::mpc_ref_traj traj_msg;
    MPCMode mpc_mode;
    Eigen::Vector4f control;

    ros::Time last_request;
    ros::ServiceClient arming_client, set_mode_client;  
    ros::Subscriber state_sub, odom_sub, goal_sub, traj_sub;  
    ros::Publisher cmd_pub;
    
    MPCWrapper *wrapper;

    void state_Callback(const mavros_msgs::State::ConstPtr& msg);
    void odom_Callback(const nav_msgs::Odometry::ConstPtr& msg);
    void traj_Callback(const quadrotor_msgs::mpc_ref_traj::ConstPtr& msg);

    void FSMProcess();
    void getTrajRef();
    void acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw, Eigen::Vector4d &quat);
    bool reachgoal(nav_msgs::Odometry& msg, Eigen::Vector3f& goal);
    void publishcontrol();

  public:
    MPCRos(ros::NodeHandle &nh);
    ~MPCRos();
    void ExectControl();
    
};

#endif

