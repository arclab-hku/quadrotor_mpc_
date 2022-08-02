#include "uav_mpc/mpc_wrapper.h"

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MPCWrapper::MPCWrapper(ros::NodeHandle &nh):nh(nh)
{
  nh.getParam("/cost/cost_px", cost_px); 
  nh.getParam("/cost/cost_py", cost_py); 
  nh.getParam("/cost/cost_pz", cost_pz); 
  nh.getParam("/cost/cost_qw", cost_qw); 
  nh.getParam("/cost/cost_qx", cost_qx); 
  nh.getParam("/cost/cost_qy", cost_qy); 
  nh.getParam("/cost/cost_qz", cost_qz); 
  nh.getParam("/cost/cost_vx", cost_vx); 
  nh.getParam("/cost/cost_vy", cost_vy); 
  nh.getParam("/cost/cost_vz", cost_vz); 
  nh.getParam("/cost/cost_thrust", cost_thrust); 
  nh.getParam("/cost/cost_wx", cost_wx); 
  nh.getParam("/cost/cost_wy", cost_wy); 
  nh.getParam("/cost/cost_wz", cost_wz); 
  nh.getParam("/boudings/T_max", T_max); 
  nh.getParam("/boudings/T_min", T_min); 
  nh.getParam("/boudings/wx_max", wx_max); 
  nh.getParam("/boudings/wx_min", wx_min); 
  nh.getParam("/boudings/wy_max", wy_max); 
  nh.getParam("/boudings/wy_min", wy_min); 
  nh.getParam("/boudings/wz_max", wz_max); 
  nh.getParam("/boudings/wz_min", wz_min); 
}

MPCWrapper::~MPCWrapper()
{
}

bool MPCWrapper::initSolver(nav_msgs::Odometry& msg)
{
  /* Clear solver memory. */
  memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  memset(&acadoVariables, 0, sizeof( acadoVariables ));

  /* Initialize the solver. */
  acado_initializeSolver();

  /* Initialize the states and controls. */
  for (int i = 0; i < N + 1; ++i)
  {
	  acadoVariables.x[i * NX + 0] = msg.pose.pose.position.x;
	  acadoVariables.x[i * NX + 1] = msg.pose.pose.position.y;
	  acadoVariables.x[i * NX + 2] = msg.pose.pose.position.z;
	  acadoVariables.x[i * NX + 3] = msg.pose.pose.orientation.w;
	  acadoVariables.x[i * NX + 4] = msg.pose.pose.orientation.x;
	  acadoVariables.x[i * NX + 5] = msg.pose.pose.orientation.y;
	  acadoVariables.x[i * NX + 6] = msg.pose.pose.orientation.z;
	  acadoVariables.x[i * NX + 7] = msg.twist.twist.linear.x;
	  acadoVariables.x[i * NX + 8] = msg.twist.twist.linear.y;
	  acadoVariables.x[i * NX + 9] = msg.twist.twist.linear.z;
  }

  for (int i = 0; i < NX; ++i)
	  acadoVariables.x0[ i ] = acadoVariables.x[ i ];

  /* Initialize the Cost Matrix. */
  for(int i = 0; i < N; ++i)
  {
	  acadoVariables.W[ i * (NY * NY) + 0 * NY + 0] = cost_px;
	  acadoVariables.W[ i * (NY * NY) + 1 * NY + 1] = cost_py;
    acadoVariables.W[ i * (NY * NY) + 2 * NY + 2] = cost_pz;
	  acadoVariables.W[ i * (NY * NY) + 3 * NY + 3] = cost_qw;
	  acadoVariables.W[ i * (NY * NY) + 4 * NY + 4] = cost_qx;
	  acadoVariables.W[ i * (NY * NY) + 5 * NY + 5] = cost_qy;
	  acadoVariables.W[ i * (NY * NY) + 6 * NY + 6] = cost_qz;
	  acadoVariables.W[ i * (NY * NY) + 7 * NY + 7] = cost_vx;
	  acadoVariables.W[ i * (NY * NY) + 8 * NY + 8] = cost_vy;
	  acadoVariables.W[ i * (NY * NY) + 9 * NY + 9] = cost_vz;
	  acadoVariables.W[ i * (NY * NY) + 10 * NY + 10] = cost_thrust;
	  acadoVariables.W[ i * (NY * NY) + 11 * NY + 11] = cost_wx;
	  acadoVariables.W[ i * (NY * NY) + 12 * NY + 12] = cost_wy;
	  acadoVariables.W[ i * (NY * NY) + 13 * NY + 13] = cost_wz;
  }

  acadoVariables.WN[0 * NX + 0] = cost_px;
  acadoVariables.WN[1 * NX + 1] = cost_py;
  acadoVariables.WN[2 * NX + 2] = cost_pz;
  acadoVariables.WN[3 * NX + 3] = cost_qw;
  acadoVariables.WN[4 * NX + 4] = cost_qx;
  acadoVariables.WN[5 * NX + 5] = cost_qy;
  acadoVariables.WN[6 * NX + 6] = cost_qz;
  acadoVariables.WN[7 * NX + 7] = cost_vx;
  acadoVariables.WN[8 * NX + 8] = cost_vy;
  acadoVariables.WN[9 * NX + 9] = cost_vz;


  /* Initialize the Boundings. */
  for(int i = 0; i < N; ++i)
  {
    // std::cout << "max" << acadoVariables.ubValues[i] << std::endl;
    // std::cout << "min" << acadoVariables.lbValues[i] << std::endl;
	  acadoVariables.ubValues[i * NU + 0] = 12;
    acadoVariables.lbValues[i * NU + 0] = 2;
    acadoVariables.ubValues[i * NU + 1] = 1.5;
    acadoVariables.lbValues[i * NU + 1] = -1.5;
    acadoVariables.ubValues[i * NU + 2] = 1.5;
    acadoVariables.lbValues[i * NU + 2] = -1.5;
    acadoVariables.ubValues[i * NU + 3] = 1;
    acadoVariables.lbValues[i * NU + 3] = -1;
  }

  /* Prepare first step */
  acado_preparationStep();

  return true;

}

void MPCWrapper::gerReference(const Eigen::MatrixXd& ref)
{
  /* Initialize the measurements/reference. */
  for (int i = 0; i < N; ++i)
  {
    acadoVariables.y[i * NY + 0] = ref.col(i)[0];      // px
	  acadoVariables.y[i * NY + 1] = ref.col(i)[1];      // py
	  acadoVariables.y[i * NY + 2] = ref.col(i)[2];      // pz
	  acadoVariables.y[i * NY + 3] = ref.col(i)[3];      // qw
	  acadoVariables.y[i * NY + 4] = ref.col(i)[4];      // qx
	  acadoVariables.y[i * NY + 5] = ref.col(i)[5];      // qy
	  acadoVariables.y[i * NY + 6] = ref.col(i)[6];      // qz
	  acadoVariables.y[i * NY + 7] = ref.col(i)[7];      // vx
	  acadoVariables.y[i * NY + 8] = ref.col(i)[8];      // vy
	  acadoVariables.y[i * NY + 9] = ref.col(i)[9];      // vz
	  acadoVariables.y[i * NY + 10] = ref.col(i)[10];    // thrust
	  acadoVariables.y[i * NY + 11] = ref.col(i)[11];    // wx
	  acadoVariables.y[i * NY + 12] = ref.col(i)[12];    // wy
	  acadoVariables.y[i * NY + 13] = ref.col(i)[13];    // wz
  }  

  for (int i = 0; i < NYN; ++i)
  {
    acadoVariables.yN[ i ] = ref.col(N+1)[i];
  }  
}

bool MPCWrapper::getSolution(nav_msgs::Odometry& msg, Eigen::Vector4f& control)
{
  acado_tic( &t );
  
  /* The Real-time Intreration. */
  status = acado_feedbackStep( );

  if ( status )
  {
	  return 0;
  }

	// acado_printDifferentialVariables();
	// acado_printControlVariables();

  /* Return Control Input Value */
  real_t *U = acado_getVariablesU();
  control[0] = U[0];
  control[1] = U[1];
  control[2] = U[2];
  control[3] = U[3];

  /* Update the State */
  updateState(msg);

  /* Optional: shift the initialization (look at acado_common.h). */
  // acado_shiftStates(2, 0, 0); 
  // acado_shiftControls( 0 );

  /* Prepare for the next step. */
  acado_preparationStep();

  /* Read the elapsed time. */
  real_t te = acado_toc( &t );

  /* Eye-candy. */
  if( !VERBOSE )
	   printf("\n\n Average time of one real-time iteration:   %f milliseconds\n\n", 1e3 * te );
  
  return true;
}

void MPCWrapper::updateState(nav_msgs::Odometry& msg)
{
  acadoVariables.x0[0] = msg.pose.pose.position.x;
  acadoVariables.x0[1] = msg.pose.pose.position.y;
  acadoVariables.x0[2] = msg.pose.pose.position.z;
  acadoVariables.x0[3] = msg.pose.pose.orientation.w;
  acadoVariables.x0[4] = msg.pose.pose.orientation.x;
  acadoVariables.x0[5] = msg.pose.pose.orientation.y;
  acadoVariables.x0[6] = msg.pose.pose.orientation.z;
  acadoVariables.x0[7] = msg.twist.twist.linear.x;
  acadoVariables.x0[8] = msg.twist.twist.linear.y;
  acadoVariables.x0[9] = msg.twist.twist.linear.z;
}
