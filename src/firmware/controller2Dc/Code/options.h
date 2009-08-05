#ifndef __options_h__
#define __options_h__
               
//#define VERSION 0x0111   				/* standard/basic implementation */
//#define VERSION 0x0112    			    /*decouples shoulder first two joints */
//#define VERSION 0x0113			   	/* decouples the third joint of the shoulder */
//#define VERSION 0x0114		    	/* feedback from the AD */
//#define VERSION 0x0115		    	/* coordinated control of the eyes */
//#define VERSION 0x0116		    	/* low level current control */
 
#define SERIAL_MENU_OPTION    1       /* if on, enables serial menu interface */
  
#define DEBUG_CAN_MSG 		1		/* conditional compile for printing can info */
//#define DEBUG_CONTROL_RATE	1 		/* for debugging control cycle rate */
//#define DEBUG_TRAJECTORY 		1		/* print encoder/joint position */
//#define DEBUG_DESIRED 		1		/* print desired position */
//#define DEBUG_IDENTIFICATION	1		/* print voltage-current-position */
#define DEBUG_SERIAL			1		/* for debugging through terminal */
//#define DEBUG_CURRENT			1		/* for debugging current through terminal */
#define DEBUG_CALIBRATION		1		/* for calibration debugging through terminal */
//#define EMERGENCY_DISABLED	1		/* emergency fault signal disabled */
//#define SMOOTH_PID_CTRL		1		/* for debugging current spikes */


#endif
