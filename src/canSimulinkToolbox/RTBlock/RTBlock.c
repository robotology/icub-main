/*
 * File : RTBlock.c
 */

#define S_FUNCTION_NAME  RTBlock
#define S_FUNCTION_LEVEL 2

#include <stdio.h>
#include "simstruc.h"

#define NUMBER_OF_ARGS	3
#define STEP   		(int)(*mxGetPr(ssGetSFcnParam(S,0)))
#define PRIORITY		(int)(*mxGetPr(ssGetSFcnParam(S,1)))
#define THPRIORITY	(int)(*mxGetPr(ssGetSFcnParam(S,2)))

// Lib functions definition
void mdlInitializeSizes_Fun(SimStruct *S);
void mdlStart_Fun(SimStruct *S, int priority, int thpriority);
void mdlUpdate_Fun(SimStruct *S, int_T tid, int step, double time);
void mdlTerminate_Fun(SimStruct *S);

static char_T msg[256];

/*****************************************************************************
+FUNCTION: mdlInitializeSizes

+DESCRIPTION: Setup sizes of the various vectors.

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
void mdlInitializeSizes(SimStruct *S)
{

#ifdef MATLAB_MEX_FILE
	ssSetNumSFcnParams(S, NUMBER_OF_ARGS);
	if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) 
	{
		sprintf(msg, "Error in RTBlock: \n");
		sprintf(&msg[strlen(msg)],
				 "Wrong number of input arguments passed.\n%d arguments are expected\n",
				 NUMBER_OF_ARGS);
		ssSetErrorStatus(S,msg);
		return;
	}
#endif

	if (!ssSetNumOutputPorts(S, 1)) return;
	if (!ssSetNumInputPorts(S, 0)) return;
	ssSetOutputPortWidth(S, 0, 2); //Width of output port one (index 0) 

	mdlInitializeSizes_Fun(S);
}



/*****************************************************************************
+FUNCTION: mdlInitializeSampleTimes

+DESCRIPTION: Specifiy that we inherit our sample time from the driving block.

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

/*****************************************************************************
+FUNCTION: mdlStart

+DESCRIPTION: Routine used to initialize data

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
#define MDL_START  /* Change to #undef to remove function */
void mdlStart(SimStruct *S)
{
	int priority = PRIORITY; // In milliseconds
	int thpriority = THPRIORITY; // In milliseconds
	mdlStart_Fun(S, priority, thpriority);
}

/*****************************************************************************
+FUNCTION: mdlOutputs

+DESCRIPTION: 

+PARAMETERS: 
SimStruct *S
int_T tid

+RETURN: static void 
*******************************************************************************/
void mdlOutputs(SimStruct *S, int_T tid)
{
}

/*****************************************************************************
+FUNCTION: mdlOutputs

+DESCRIPTION: 

+PARAMETERS: 
SimStruct *S
int_T tid

+RETURN: static void 
*******************************************************************************/
#define MDL_UPDATE
void mdlUpdate(SimStruct *S, int_T tid)
{
	int step = STEP; // In milliseconds
	double time = ssGetT(S); 
	mdlUpdate_Fun(S, tid, step, time);
}

/*****************************************************************************
+FUNCTION: mdlTerminate

+DESCRIPTION: No termination needed, but we are required to have this routine.

+PARAMETERS: 
SimStruct *S

+RETURN: static void 
*******************************************************************************/
void mdlTerminate(SimStruct *S)
{
	mdlTerminate_Fun(S);
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
