/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */



/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   20        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/* A template for testing of the solver. */
int main( )
{
  /* Some temporary variables. */
  int    i, j, iter;
  int status;
  acado_timer t;
  /* Clear solver memory. */
  memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  memset(&acadoVariables, 0, sizeof( acadoVariables ));

  /* Initialize the solver. */
  acado_initializeSolver();

  /* Initialize the states and controls. */
  for (i = 0; i < N + 1; ++i)
  {
	  acadoVariables.x[i * NX + 0] = 0;
	  acadoVariables.x[i * NX + 1] = 0;
	  acadoVariables.x[i * NX + 2] = 0;
	  acadoVariables.x[i * NX + 3] = 1;
	  acadoVariables.x[i * NX + 4] = 0;
	  acadoVariables.x[i * NX + 5] = 0;
	  acadoVariables.x[i * NX + 6] = 0;
	  acadoVariables.x[i * NX + 7] = 0;
	  acadoVariables.x[i * NX + 8] = 0;
	  acadoVariables.x[i * NX + 9] = 0;
  }

  /* Initialize the feedback states. */
  for (i = 0; i < NX; ++i)
	  acadoVariables.x0[ i ] = acadoVariables.x[ i ];
	
  /* Initialize the measurements/reference. */
  for (i = 0; i < N; ++i)
  {
    acadoVariables.y[i * NY + 0] = 0; 
	  acadoVariables.y[i * NY + 1] = 0; 
	  acadoVariables.y[i * NY + 2] = 2.0; 
	  acadoVariables.y[i * NY + 3] = 1.0; 
	  acadoVariables.y[i * NY + 4] = 0; 
	  acadoVariables.y[i * NY + 5] = 0; 
	  acadoVariables.y[i * NY + 6] = 0; 
	  acadoVariables.y[i * NY + 7] = 0; 
	  acadoVariables.y[i * NY + 8] = 0; 
	  acadoVariables.y[i * NY + 9] = 0; 
	  acadoVariables.y[i * NY + 10] = 0; 
	  acadoVariables.y[i * NY + 11] = 0; 
	  acadoVariables.y[i * NY + 12] = 0;
	  acadoVariables.y[i * NY + 13] = 0;  
  }  

  for (i = 0; i < NYN; ++i)
  {
    acadoVariables.yN[ i ] = acadoVariables.y[ i ];
  }  

  /* Initialize the Cost Matrix. */
  for(i = 0; i < N; ++i)
  {
	  acadoVariables.W[ i * (NY * NY) + 0 * NY + 0] = 100;
	  acadoVariables.W[ i * (NY * NY) + 1 * NY + 1] = 100;
    acadoVariables.W[ i * (NY * NY) + 2 * NY + 2] = 100;
	  acadoVariables.W[ i * (NY * NY) + 3 * NY + 3] = 100;
	  acadoVariables.W[ i * (NY * NY) + 4 * NY + 4] = 100;
	  acadoVariables.W[ i * (NY * NY) + 5 * NY + 5] = 100;
	  acadoVariables.W[ i * (NY * NY) + 6 * NY + 6] = 100;
	  acadoVariables.W[ i * (NY * NY) + 7 * NY + 7] = 10;
	  acadoVariables.W[ i * (NY * NY) + 8 * NY + 8] = 10;
	  acadoVariables.W[ i * (NY * NY) + 9 * NY + 9] = 10;
	  acadoVariables.W[ i * (NY * NY) + 10 * NY + 10] = 0.6;
	  acadoVariables.W[ i * (NY * NY) + 11 * NY + 11] = 0.6;
	  acadoVariables.W[ i * (NY * NY) + 12 * NY + 12] = 0.6;
	  acadoVariables.W[ i * (NY * NY) + 13 * NY + 13] = 0.6;
  }

  acadoVariables.WN[0 * NX + 0] = 100;
  acadoVariables.WN[1 * NX + 1] = 100;
  acadoVariables.WN[2 * NX + 2] = 100;
  acadoVariables.WN[3 * NX + 3] = 100;
  acadoVariables.WN[4 * NX + 4] = 100;
  acadoVariables.WN[5 * NX + 5] = 100;
  acadoVariables.WN[6 * NX + 6] = 100;
  acadoVariables.WN[7 * NX + 7] = 10;
  acadoVariables.WN[8 * NX + 8] = 10;
  acadoVariables.WN[9 * NX + 9] = 10;

  /* Prepare first step */
  if( VERBOSE ) acado_printHeader();
  acado_preparationStep();
  acado_tic( &t );

  for(i = 0; i < 80; ++i)
    printf("\tbound%f: ", acadoVariables.ubValues[i]);

  /* The "real-time iterations" loop. */
  for(iter = 0; iter < NUM_STEPS; ++iter)
  {
    /* Perform the feedback step. */
	  status = acado_feedbackStep( );

	  if ( status )
	  {
	    printf("status %d", status);
	    break;
	  }

	  /* Apply the new control immediately to the process, first NU components. */

	  if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
	  acado_printDifferentialVariables();
	  acado_printControlVariables();

	  for (i = 0; i < NX; ++i) 
	    acadoVariables.x0[ i ] = acadoVariables.x[ NX + i ];

	  /* Optional: shift the initialization (look at acado_common.h). */
    acado_shiftStates(2, 0, 0); 
	  acado_shiftControls( 0 );

	  /* Prepare for the next step. */
	  acado_preparationStep();
  }

  /* Read the elapsed time. */
  real_t te = acado_toc( &t );
  if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

  /* Eye-candy. */
  if( !VERBOSE )
    printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);
  return 0;

}
