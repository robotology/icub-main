
#define S_FUNCTION_NAME ReceiveCanESD_V2
#define S_FUNCTION_LEVEL 2



#define SAMPLE_TIME_0        INHERITED_SAMPLE_TIME
#define NUM_DISC_STATES      0
#define DISC_STATES_IC       [0]
#define NUM_CONT_STATES      0
#define CONT_STATES_IC       [0]

#define SFUNWIZ_GENERATE_TLC 1
#define SOURCEFILES "__SFB__"
#define PANELINDEX           6
#define USE_SIMSTRUCT        0
#define SHOW_COMPILE_STEPS   0                   
#define CREATE_DEBUG_MEXFILE 0
#define SAVE_CODE_ONLY       0
#define SFUNWIZ_REVISION     3.0
/* %%%-SFUNWIZ_defines_Changes_END --- EDIT HERE TO _BEGIN */
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
#include "simstruc.h"
#include "ntcan.h"

#define         NUMBER_OF_ARGS      (2)
#define         CAN_BAUDRATE        ssGetSFcnParam(S,0)
#define         CAN_ID_ARG          ssGetSFcnParam(S,1)
#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

 HANDLE m_h0=NULL;
 CMSG rx_message[20];
 long ret;  
 int x=0;
/*====================*
 * S-function methods *
 *====================*/
#define MDL_CHECK_PARAMETERS
 #if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)  //
   /* Function: mdlCheckParameters =============================================
     * Abstract:
     *    Validate our parameters to verify they are okay.
     */
    static void mdlCheckParameters(SimStruct *S)
    {
     #define PrmNumPos 46
     int paramIndex = 0;
     bool validParam = false;
     char paramVector[] ={'1','2'};
     static char parameterErrorMsg[] ="The data type and/or complexity of parameter    does not match the information "
     "specified in the S-function Builder dialog. For non-double parameters you will need to cast them using int8, int16,"
     "int32, uint8, uint16, uint32 or boolean."; 

     /* All parameters must match the S-function Builder Dialog */
     

	 {
	  const mxArray *pVal0 = ssGetSFcnParam(S,0);
	  if (!IS_PARAM_DOUBLE(pVal0)) {
	    validParam = true;
	    paramIndex = 0;
	    goto EXIT_POINT;
	  }
	 }

	 {
	  const mxArray *pVal1 = ssGetSFcnParam(S,1);
	  if (!IS_PARAM_DOUBLE(pVal1)) {
	    validParam = true;
	    paramIndex = 1;
	    goto EXIT_POINT;
	  }
	 }

	 {
	  const mxArray *pVal2 = ssGetSFcnParam(S,2);
	  if (!IS_PARAM_DOUBLE(pVal2)) {
	    validParam = true;
	    paramIndex = 2;
	    goto EXIT_POINT;
	  }
	 }
     EXIT_POINT:
      if (validParam) {
	  parameterErrorMsg[PrmNumPos] = paramVector[paramIndex];
	  ssSetErrorStatus(S,parameterErrorMsg);
      }
	return;
    }
 #endif /* MDL_CHECK_PARAMETERS */
/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
     int_T m_net=0;
    long m_txbufsize =1;
    long m_rxbufsize =10;
    long m_txtout    =10;
    long m_rxtout    =10;
    unsigned long m_mode=0;
    int m_baudrate;
    const int16_T  *baudrate  = mxGetData(CAN_BAUDRATE); 
    int_T i,j=0;
  //  DECL_AND_INIT_DIMSINFO(inputDimsInfo);
  //  DECL_AND_INIT_DIMSINFO(outputDimsInfo);
    ssSetNumSFcnParams(S, NUMBER_OF_ARGS);  /* Number of expected parameters */
      #if defined(MATLAB_MEX_FILE)
	if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) 
    {
     //   mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) 
        {
            return;
        }
     } 
     else 
     {
        return; /* Parameter mismatch will be reported by Simulink */
	 }
      #endif
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumInputPorts(S, 0);
    ssSetNumOutputPorts(S, mxGetN(CAN_ID_ARG));
//        ssSetNumOutputPorts(S, 2);
    for (i=0;i<mxGetN(CAN_ID_ARG);i++) 
    {
     ssSetOutputPortWidth(S, i, 8);
     ssSetOutputPortDataType(S, i, SS_DOUBLE);
    }
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
   
    for (i=0;i<NUMBER_OF_ARGS;i++) 
    {
       ssSetSFcnParamNotTunable(S,i);
    }
    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE |
                     SS_OPTION_USE_TLC_WITH_ACCELERATOR |
		     SS_OPTION_WORKS_WITH_CODE_REUSE));
    ret = canOpen(m_net, m_mode, m_txbufsize, m_rxbufsize, m_txtout, m_rxtout, &m_h0);
    if(ret == NTCAN_SUCCESS)
    {
     x=1;
    }
    else
    {
        x=-1;
        canClose(m_h0);
        ssSetErrorStatus(S,"CanOpen failed");
        return;
    }
   m_baudrate=(int) *baudrate;
    ret = canSetBaudrate(m_h0,m_baudrate);//canSetBaudrate(m_h0, z-1);
    if(ret == NTCAN_SUCCESS)
    {
     x=1;
    }
    else
    {
        x=-1;
        canClose(m_h0);
        ssSetErrorStatus(S,"CANSetBaudrate failed");
        return;
    }
     for (i=0;i<(long) mxGetN(CAN_ID_ARG);i++) 
     {
        ret = canIdAdd(m_h0,  (long) mxGetPr(CAN_ID_ARG)[i]);
        
        if(ret == NTCAN_SUCCESS)
        {
            x=1;
        }
        else
        {
            x=-1;
            canClose(m_h0);
            ssSetErrorStatus(S,"CANIdAdd failed");
            return;   
        }
     }
 
/*   for (i =0; i<2047; i++)
  {
    ret = canIdAdd(m_h0, i);
    if(ret == NTCAN_SUCCESS)
    {
    }
    else
    {
	 if (m_h0!=0) canClose(m_h0);
     return;   
    }
  }*/
  if (ret == NTCAN_INSUFFICIENT_RESOURCES)
  {
     canClose(m_h0);
  }
}

#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S, 
                                         int_T            port,
                                         const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S, 
                                          int_T            port, 
                                          const DimsInfo_T *dimsInfo)
{
 if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy  the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLE_TIME_0);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct *S, int port, DTypeId dType)
{
    ssSetInputPortDataType( S, 0, dType);
    
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct *S)
{
  ssSetInputPortDataType( S, 0, SS_DOUBLE);
}
/* Function: mdlOutputs =======================================================
 *
*/
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T        *y[8];
    int_T index;
    int i,j=0;
    int d=0;
    long len=mxGetN(CAN_ID_ARG);
    
    for (i=0;i<mxGetN(CAN_ID_ARG);i++) 
    {
        index=(int_T)mxGetPr(CAN_ID_ARG)[i];
        y[i]=ssGetOutputPortRealSignal(S,i);
    }
    ret =canTake(m_h0, rx_message, &len,NULL);
    if(ret == NTCAN_SUCCESS)
    {
        for (i=0;i<len;i++)
        {
            if (&rx_message[i]!=NULL) 
            {
                for(j=0;j<mxGetN(CAN_ID_ARG);j++)
                {
                    if (rx_message[i].id==(int) mxGetPr(CAN_ID_ARG)[j])
                    {
                        for (d=0;d<8;d++)
                        {
                            y[j][d]=(int_T) rx_message[i].data[d]; // j is the index of the output port
                        }
                    }
              
                } // j is the index of the output port      
            }
        }
    }
    
    
    

}



/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    canClose(m_h0);
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


