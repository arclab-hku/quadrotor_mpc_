#include "traj_min_jerk.hpp"
#include "traj_min_snap.hpp"

#include <chrono>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <random>

#include <ros/ros.h>

#include <quadrotor_msgs/mpc_ref_point.h>
#include <quadrotor_msgs/mpc_ref_traj.h>

using namespace std;
using namespace ros;
using namespace Eigen;

#define Ksample 20
#define t_step 0.1

class RandomRouteGenerator
{
public:
    RandomRouteGenerator(Array3d l, Array3d u)
        : lBound(l), uBound(u), uniformReal(0.0, 1.0) {}

    inline MatrixXd generate(int N)
    {
        MatrixXd route(3, N + 1);
        Array3d temp;
        route.col(0).setZero();
        route.col(0) << 0, 0, 2;
        for (int i = 0; i < N; i++)
        {
            temp << uniformReal(gen), uniformReal(gen), uniformReal(gen);
            temp = (uBound - lBound) * temp + lBound;
            route.col(i + 1) << temp;
        }
        std::cout << "route" << route << std::endl;
        return route;
    }

private:
    Array3d lBound;
    Array3d uBound;
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> uniformReal;
};

VectorXd allocateTime(const MatrixXd &wayPs,
                      double vel,
                      double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_node");
    ros::NodeHandle nh_("~");
    ros::Publisher mpc_ref_pub;

    int opt_method = 0;  // 0 for minimun jerk, 1 for minimun snap
    opt_method = nh_.param("opt_method", 0);
    std::cout << opt_method << std::endl;
    mpc_ref_pub = nh_.advertise<quadrotor_msgs::mpc_ref_traj> ("/mpc_ref_traj", 1);

    // RandomRouteGenerator routeGen(Array3d(-16, -16, -16), Array3d(16, 16, 16));

    min_jerk::JerkOpt jerkOpt;
    min_jerk::Trajectory minJerkTraj;

    min_snap::SnapOpt snapOpt;
    min_snap::Trajectory minSnapTraj;

    MatrixXd route;
    VectorXd ts;
    Matrix3d iS, fS;
    Eigen::Matrix<double, 3, 4> iSS, fSS;
    iS.setZero();
    fS.setZero();
    Vector3d zeroVec(0.0, 0.0, 0.0);
    Rate lp(100);

    std::chrono::high_resolution_clock::time_point tc0, tc1, tc2;
    double d0, d1;

    d0 = d1 = 0.0;
    // route = routeGen.generate(3);
    route = Eigen::MatrixXd::Zero(3, 4);
    route << 0, 3, 6, 9,
             0, 0, 0, 0,
             2, 2, 2, 2;

    iS.col(0) << route.leftCols<1>();
    fS.col(0) << route.rightCols<1>();
    ts = allocateTime(route, 3.0, 3.0);

    iSS << iS, Eigen::MatrixXd::Zero(3, 1);
    fSS << fS, Eigen::MatrixXd::Zero(3, 1);

    tc0 = std::chrono::high_resolution_clock::now();
    jerkOpt.reset(iS, fS, route.cols() - 1);
    jerkOpt.generate(route.block(0, 1, 3, 3 - 1), ts);
    jerkOpt.getTraj(minJerkTraj);
    tc1 = std::chrono::high_resolution_clock::now();

    d0 += std::chrono::duration_cast<std::chrono::duration<double>>(tc1 - tc0).count();

    tc1 = std::chrono::high_resolution_clock::now();
    snapOpt.reset(iSS, fSS, route.cols() - 1);
    snapOpt.generate(route.block(0, 1, 3, 3 - 1), ts);
    snapOpt.getTraj(minSnapTraj);
    tc2 = std::chrono::high_resolution_clock::now();

    d1 += std::chrono::duration_cast<std::chrono::duration<double>>(tc2 - tc1).count();
        
    std::cout << "Piece Number: " << 2
              << " MinJerk Comp. Time: " << d0  << " s"
              << " MinSnap Comp. Time: " << d1  << " s" << std::endl;

    double t = 0;
    double t_duration = minJerkTraj.getTotalDuration();
    double t_max = 0;
    ros::Time time_start;
    int start = 0;

    while(ros::ok())
    {
      if(start < 2)
      {
        time_start = ros::Time::now();
        start++;
      }
      ros::Time time_now = ros::Time::now();
      t = time_now.toSec() - time_start.toSec() + 0.01;
      quadrotor_msgs::mpc_ref_traj mpc_traj;

      for(int i = 0; i < Ksample + 1; ++i)
      {
        double rt = t + t_step * i;
        if(rt > t_duration)
          rt = t_max;
        else
          t_max = rt;
        Eigen::Vector3d p;
        p = minJerkTraj.getPos(rt);
        Eigen::Vector3d v;
        v = minJerkTraj.getVel(rt);
        Eigen::Vector3d a;
        a = minJerkTraj.getAcc(rt);
        quadrotor_msgs::mpc_ref_point mpc_point;
        mpc_point.position.x = p[0];
        mpc_point.position.y = p[1];
        mpc_point.position.z = p[2];
        mpc_point.velocity.x = v[0];
        mpc_point.velocity.y = v[1];
        mpc_point.velocity.z = v[2];
        mpc_point.acceleration.x = a[0];
        mpc_point.acceleration.y = a[1];
        mpc_point.acceleration.z = a[2];
        mpc_traj.mpc_ref_points.push_back(mpc_point);
      }

      mpc_ref_pub.publish(mpc_traj);

      ros::spinOnce();
      lp.sleep();
    }
    
    return 0;
}
