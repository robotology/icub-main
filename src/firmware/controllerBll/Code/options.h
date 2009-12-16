#ifndef __options_h__
#define __options_h__

//#define VERSION 0x0150   				/* first two joint of the shoulder */
//#define VERSION 0x0151   				/* standard/basic implementation */
//#define VERSION 0x0152				/* waist deifferential decoupling */
//#define VERSION 0x0153			   	/* decouples the third joint of the shoulder */
//#define VERSION 0x0154		    	/* only one joint */
//#define VERSION 0x0155		    	/* */
//#define VERSION 0x0156		    	/* low level current control */
//#define VERSION 0x0170		    	/* Listens for CAN messages from strain boards and computes force control */
//#define VERSION 0x0171		    	/* Use the internal ADC to measure the strain gauges computes force control */
//#define VERSION 0x0172		    	/* like version 0x0170, with optical encoder*/


//#define SERIAL_MENU_OPTION    1       /* if on, enables serial menu interface */

//#define DEBUG_CAN_MSG 			1		/* conditional compile for printing can info */
//#define DEBUG_CONTROL_RATE		1 		/* for debugging control cycle rate */
//#define DEBUG_TRAJECTORY 			1		/* print encoder/joint position */
//#define DEBUG_SERIAL				1		/* for debugging through terminal */
//#define DEBUG_CURRENT				1		/* for debugging current through terminal */
//#define DEBUG_CALIBRATION			1		/* for calibration debugging through terminal */
//#define DEBUG_CPL_BOARD	    	1		/* for coupled board debug */
//#define EMERGENCY_DISABLED		1		/* emergency fault signal disabled */
//#define SMOOTH_PID_CTRL			1		/* for debugging current spikes */
//#define TEMPERATURE_SENSOR    	1
//#define ENCODER_SHIFT_11BITS  	1
//#define DEBUG_ABS_SENSOR_STATUS 	1
#define DEBUG_CONTROL_MODE    	1


#endif
 