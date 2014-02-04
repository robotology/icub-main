// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


//
// $Id: messages.h,v 1.5 2009/07/13 10:07:47 randaz Exp $
//
//

#ifndef __canbusmessagesh__
#define __canbusmessagesh__

#define CAN_PROTOCOL_MAJOR          1
#define CAN_PROTOCOL_MINOR          1
#define LAST_BLL_BUILD             71
#define LAST_MC4_BUILD             71

#define MODE_IDLE					0
#define MODE_POSITION 				1
#define MODE_VELOCITY				2
#define MODE_TORQUE					3
#define MODE_IMPEDANCE_POS			4
#define MODE_IMPEDANCE_VEL			5
#define MODE_CALIB_ABS_POS_SENS		0x10
#define MODE_CALIB_HARD_STOPS		0x20
#define MODE_HANDLE_HARD_STOPS		0x30
#define MODE_MARGIN_REACHED    		0x40

#define MODE_OPENLOOP               0x50

/*
 * this is 8 bits long, MSB is the channel (0 or 1). 
 */
#define CAN_NO_MESSAGE				0
#define CAN_CONTROLLER_RUN		 	1
#define CAN_CONTROLLER_IDLE			2
#define CAN_TOGGLE_VERBOSE			3
#define CAN_CALIBRATE_ENCODER		4
#define CAN_ENABLE_PWM_PAD			5
#define CAN_DISABLE_PWM_PAD			6
#define CAN_GET_CONTROL_MODE		7
#define CAN_MOTION_DONE				8
#define CAN_SET_CONTROL_MODE		9

#define CAN_WRITE_FLASH_MEM			10
#define CAN_READ_FLASH_MEM			11
#define CAN_GET_ADDITIONAL_INFO		12
#define CAN_SET_ADDITIONAL_INFO		13
#define CAN_SET_SPEED_ESTIM_SHIFT	14 
#define CAN_SET_DEBUG_PARAM  		18
#define CAN_GET_DEBUG_PARAM  		19

#define CAN_GET_ENCODER_POSITION	20
#define CAN_SET_DESIRED_POSITION	21
#define CAN_GET_DESIRED_POSITION	22
#define CAN_SET_DESIRED_VELOCITY	23
#define CAN_GET_DESIRED_VELOCITY	24
#define CAN_SET_DESIRED_ACCELER		25
#define CAN_GET_DESIRED_ACCELER		26
#define CAN_POSITION_MOVE			27
#define CAN_VELOCITY_MOVE			28
#define CAN_SET_ENCODER_POSITION	29
#define CAN_SET_P_GAIN				30
#define CAN_GET_P_GAIN				31
#define CAN_SET_D_GAIN				32
#define CAN_GET_D_GAIN				33
#define CAN_SET_I_GAIN				34
#define CAN_GET_I_GAIN				35
#define CAN_SET_ILIM_GAIN			36
#define CAN_GET_ILIM_GAIN			37
#define CAN_SET_OFFSET				38
#define CAN_GET_OFFSET				39
#define CAN_SET_SCALE				40
#define CAN_GET_SCALE				41
#define CAN_SET_TLIM				42
#define CAN_GET_TLIM				43
#define CAN_SET_DESIRED_TORQUE		44
#define CAN_GET_DESIRED_TORQUE		45
#define CAN_STOP_TRAJECTORY   		46

#define CAN_SET_BOARD_ID			50
#define CAN_GET_BOARD_ID			51
#define CAN_SET_TORQUE_SOURCE       52

#define CAN_GET_PID_ERROR			55
#define CAN_GET_ERROR_STATUS		60
#define CAN_GET_ENCODER_VELOCITY	61
#define CAN_SET_COMMAND_POSITION	62
#define CAN_GET_PID_OUTPUT			63
#define CAN_SET_MIN_POSITION		64
#define CAN_GET_MIN_POSITION		65
#define CAN_SET_MAX_POSITION		66
#define CAN_GET_MAX_POSITION		67
#define CAN_SET_MAX_VELOCITY		68
#define CAN_GET_MAX_VELOCITY		69

// special messages for inter board communication/synchronization 
//#define CAN_GET_ACTIVE_ENCODER_POSITION 70
//#define CAN_SET_ACTIVE_ENCODER_POSITION 71

#define CAN_SET_CURRENT_LIMIT		72
#define CAN_SET_BCAST_POLICY		73

#define CAN_SET_VEL_SHIFT			74

#define CAN_SET_OFFSET_ABS_ENCODER  75
#define CAN_GET_OFFSET_ABS_ENCODER  76

#define CAN_SET_SMOOTH_PID          77
#define CAN_SET_TORQUE_PID          78
#define CAN_GET_TORQUE_PID          79
#define CAN_SET_TORQUE_PIDLIMITS    80
#define CAN_GET_TORQUE_PIDLIMITS    81
#define CAN_SET_POS_PID             82
#define CAN_GET_POS_PID             83
#define CAN_SET_POS_PIDLIMITS       84
#define CAN_GET_POS_PIDLIMITS   	85

#define CAN_SET_VEL_TIMEOUT			86

#define CAN_SET_IMPEDANCE_PARAMS    87
#define CAN_GET_IMPEDANCE_PARAMS    88
#define CAN_SET_IMPEDANCE_OFFSET    89
#define CAN_GET_IMPEDANCE_OFFSET    90

#define CAN_GET_FIRMWARE_VERSION    91
#define CAN_SET_OPTICAL_ENC_RATIO   92
#define CAN_SET_POS_STICTION_PARAMS     93
#define CAN_GET_POS_STICTION_PARAMS     94
#define CAN_SET_TORQUE_STICTION_PARAMS  95
#define CAN_GET_TORQUE_STICTION_PARAMS  96

#define CAN_SET_BACKEMF_PARAMS          97
#define CAN_GET_BACKEMF_PARAMS          98

#define CAN_SET_MODEL_PARAMS         99
#define CAN_GET_MODEL_PARAMS        100

#define CAN_SET_CURRENT_PID       	101
#define CAN_GET_CURRENT_PID 		102
#define CAN_SET_CURRENT_PIDLIMITS   103
#define CAN_GET_CURRENT_PIDLIMITS 	104
#define CAN_SET_VELOCITY_PID        105
#define CAN_GET_VELOCITY_PID        106
#define CAN_SET_VELOCITY_PIDLIMITS  107
#define CAN_GET_VELOCITY_PIDLIMITS  108
#define CAN_SET_DESIRED_CURRENT		109
#define CAN_GET_DESIRED_CURRENT		110
#define CAN_SET_PERIODIC_MSG_CONTENTS   111 //this message is used only for 2foc boards
#define CAN_SET_I2T_PARAMS			112
#define CAN_GET_I2T_PARAMS			113
#define CAN_SET_OPENLOOP_PARAMS     114
#define CAN_GET_OPENLOOP_PARAMS		115

#define CAN_SET_OPENLOOP_PARAMS     114
#define CAN_GET_OPENLOOP_PARAMS		115

#define NUM_OF_MESSAGES             116

/* error status values */
#define ERROR_NONE					0			/* no error, all ok */
#define ERROR_UNSPECIFIED			1			/* generic error */
#define ERROR_MODE					2			/* mode error, can't apply command in current mode */
#define ERROR_FMT					3			/* format error, command in wrong format */
#define ERROR_SEND					4			/* can't send answer back */

/* 
 * class 1 messages, broadcast 
 * when in bcast mode, messages are sent periodically by the controller
 *
 */
#define CAN_BCAST_NONE				0
#define CAN_BCAST_POSITION			1
#define CAN_BCAST_PID_VAL			2
#define CAN_BCAST_STATUS			3
#define CAN_BCAST_CURRENT			4
#define CAN_BCAST_OVERFLOW			5
#define CAN_BCAST_PRINT				6
#define CAN_BCAST_VELOCITY			7
#define CAN_BCAST_PID_ERROR			8
#define CAN_BCAST_DEBUG				9
#define CAN_BCAST_MOTOR_POSITION   10
#define CAN_BCAST_MOTOR_SPEED      11
#define CAN_BCAST_MAX_MSGS		   12

#endif
