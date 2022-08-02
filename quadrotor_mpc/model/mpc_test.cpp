/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <math.h>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <memory>
#include <acado_optimal_control.hpp>

USING_NAMESPACE_ACADO

int main(int argc, char * const argv[ ])
{

  const bool CODE_GEN = true; 
	//
	// Variables
	//

	DifferentialState     p_x, p_y, p_z;
  DifferentialState     q_w, q_x, q_y, q_z;
  DifferentialState     v_x, v_y, v_z;
  Control               T, w_x, w_y, w_z;
  DifferentialEquation  f;
  Function              h, hN;

	// Parameters with exemplary values. These are set/overwritten at runtime.
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 2.0;       // Time horizon [s]
  const double dt = 0.1;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes
  const double g_z = 9.8066;      // Gravity is everywhere [m/s^2]
  const double w_max_yaw = 1;     // Maximal yaw rate [rad/s]
  const double w_max_xy = 3;      // Maximal pitch and roll rate [rad/s]
  const double T_min = 2;         // Minimal thrust [N]
  const double T_max = 20;        // Maximal thrust [N]
  const double mass = 1.0;        // Mass 1.5 [kg]

  // Bias to prevent division by zero.
  const double epsilon = 0.1;     // Camera projection recover bias [m]

  // System Dynamics
  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;
  f << dot(q_w) ==  0.5 * ( - w_x * q_x - w_y * q_y - w_z * q_z);
  f << dot(q_x) ==  0.5 * ( w_x * q_w + w_z * q_y - w_y * q_z);
  f << dot(q_y) ==  0.5 * ( w_y * q_w - w_z * q_x + w_x * q_z);
  f << dot(q_z) ==  0.5 * ( w_z * q_w + w_y * q_x - w_x * q_y);
  f << dot(v_x) ==  2 * ( q_w * q_y + q_x * q_z ) * T / mass;
  f << dot(v_y) ==  2 * ( q_y * q_z - q_w * q_x ) * T / mass;
  f << dot(v_z) ==  ( 1 - 2 * q_x * q_x - 2 * q_y * q_y ) * T / mass - g_z;

	// Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z
    << q_w << q_x << q_y << q_z
    << v_x << v_y << v_z
    << T << w_x << w_y << w_z;

  // End cost vector consists of all states (no inputs at last state).
  hN << p_x << p_y << p_z
     << q_w << q_x << q_y << q_z
   	 << v_x << v_y << v_z;

  
  OCP ocp( t_start, t_end, N );
  ocp.subjectTo( f );
	ocp.subjectTo(-w_max_xy <= w_x <= w_max_xy);
  ocp.subjectTo(-w_max_xy <= w_y <= w_max_xy);
  ocp.subjectTo(-w_max_yaw <= w_z <= w_max_yaw);
  ocp.subjectTo( T_min <= T <= T_max);

  ocp.setNOD(10);
  //	ocp.subjectTo( -5 <= x <= 5 );

  if(CODE_GEN)
  {
    // For code generation, references are set during run time.
    BMatrix Q_sparse = eye<bool>( h.getDim() );
	  BMatrix QN_sparse = eye<bool>( hN.getDim() );
	  ocp.minimizeLSQ(Q_sparse, h);
	  ocp.minimizeLSQEndTerm(QN_sparse, hN);
  }
  else
  {
    DMatrix Q = eye<double>( h.getDim() );
	  Q(0,0) = 100;   // x
    Q(1,1) = 100;   // y
    Q(2,2) = 100;   // z
    Q(3,3) = 100;   // qw
    Q(4,4) = 100;   // qx
    Q(5,5) = 100;   // qy
    Q(6,6) = 100;   // qz
    Q(7,7) = 10;   // vx
    Q(8,8) = 10;   // vy
    Q(9,9) = 10;   // vz
    Q(10,10) = 0.6;   // T
    Q(11,11) = 0.6;   // wx
    Q(12,12) = 0.6;   // wy
    Q(13,13) = 0.6;   // wz

	  DMatrix QN = eye<double>( hN.getDim() );
	  QN(0,0) = Q(0,0);   // x
    QN(1,1) = Q(1,1);   // y
    QN(2,2) = Q(2,2);   // z
    QN(3,3) = Q(3,3);   // qw
    QN(4,4) = Q(4,4);   // qx
    QN(5,5) = Q(5,5);   // qy
    QN(6,6) = Q(6,6);   // qz
    QN(7,7) = Q(7,7);   // vx
    QN(8,8) = Q(8,8);   // vy
    QN(9,9) = Q(9,9);   // vz

    // Reference is at x = 2.0m in hover (qw = 1).
    DVector r(h.getDim());    // Running cost reference
    r.setZero();
    r(0) = 2.0;
    r(3) = 1.0;

    DVector rN(hN.getDim());   // End cost reference
    rN.setZero();
    rN(0) = r(0);
    rN(3) = r(3);

    ocp.minimizeLSQ(Q, h, r);
	  ocp.minimizeLSQEndTerm(QN, hN, rN);
  }

  if(CODE_GEN)
  {
    //
	  // Export the code:
	  //
	  OCPexport mpc( ocp );

	  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
	  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

	  mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
	  mpc.set(NUM_INTEGRATOR_STEPS, N);

	  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    //	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
	  mpc.set(QP_SOLVER, QP_QPOASES);
    //	mpc.set(MAX_NUM_QP_ITERATIONS, 20);
	  mpc.set(HOTSTART_QP, YES);

    //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
    //	mpc.set(QP_SOLVER, QP_FORCES);

    //	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);    // set on run time 
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);   // set on run time
	  mpc.set(GENERATE_TEST_FILE, NO);
	  mpc.set(GENERATE_MAKE_FILE, YES);
	  mpc.set(GENERATE_MATLAB_INTERFACE, YES);

    //	mpc.set(USE_SINGLE_PRECISION, YES);
    //	mpc.set(CG_USE_OPENMP, YES);

	  if (mpc.exportCode( "quadrotor_mpc_codegen" ) != SUCCESSFUL_RETURN)
		  exit( EXIT_FAILURE );

	  mpc.printDimensionsQP( );
  }
  else
  {
    ocp.subjectTo( AT_START, p_x ==  0.0 );
    ocp.subjectTo( AT_START, p_y ==  0.0 );
    ocp.subjectTo( AT_START, p_z ==  0.0 );
    ocp.subjectTo( AT_START, q_w ==  1.0 );
    ocp.subjectTo( AT_START, q_x ==  0.0 );
    ocp.subjectTo( AT_START, q_y ==  0.0 );
    ocp.subjectTo( AT_START, q_z ==  0.0 );
    ocp.subjectTo( AT_START, v_x ==  0.0 );
    ocp.subjectTo( AT_START, v_y ==  0.0 );
    ocp.subjectTo( AT_START, v_z ==  0.0 );
    ocp.subjectTo( AT_START, w_x ==  0.0 );
    ocp.subjectTo( AT_START, w_y ==  0.0 );
    ocp.subjectTo( AT_START, w_z ==  0.0 );

    // Setup some visualization
    GnuplotWindow window1( PLOT_AT_EACH_ITERATION );
    window1.addSubplot( p_x,"position x" );
    window1.addSubplot( p_y,"position y" );
    window1.addSubplot( p_z,"position z" );
    window1.addSubplot( v_x,"verlocity x" );
    window1.addSubplot( v_y,"verlocity y" );
    window1.addSubplot( v_z,"verlocity z" );

    GnuplotWindow window3( PLOT_AT_EACH_ITERATION );
    window3.addSubplot( w_x,"rotation-acc x" );
    window3.addSubplot( w_y,"rotation-acc y" );
    window3.addSubplot( w_z,"rotation-acc z" ); 
    window3.addSubplot( T,"Thrust" );

    // Define an algorithm to solve it.
    OptimizationAlgorithm algorithm(ocp);
    algorithm.set( INTEGRATOR_TOLERANCE, 1e-6 );
    algorithm.set( KKT_TOLERANCE, 1e-3 );
    algorithm << window1;
    algorithm << window3;
    algorithm.solve();
  }

  return EXIT_SUCCESS;
}
