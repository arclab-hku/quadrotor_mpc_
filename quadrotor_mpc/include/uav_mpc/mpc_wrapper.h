#ifndef MPCWrapper_H
#define MPCWrapper_H

#include <thread>
// Eigen
#include <Eigen/Eigen>
// Ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
// ACADO
#include "acado_common.h"
#include "acado_auxiliary_functions.h"


/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

class MPCWrapper
{
  // MPC Para
  private:
    double cost_px, cost_py, cost_pz;
    double cost_qw, cost_qx, cost_qy, cost_qz;
    double cost_vx, cost_vy, cost_vz;
    double cost_thrust, cost_wx, cost_wy, cost_wz;
    double T_max, T_min, wx_max, wx_min, wy_max, wy_min, wz_max, wz_min;

  private:
    ros::NodeHandle &nh;
    int status;
    acado_timer t;

    void updateState(nav_msgs::Odometry& msg);

  public:
    MPCWrapper(ros::NodeHandle &nh);
    ~MPCWrapper();
    bool initSolver(nav_msgs::Odometry& msg);
    void gerReference(const Eigen::MatrixXd& ref);
    bool getSolution(nav_msgs::Odometry& msg, Eigen::Vector4f& control);

};


#endif