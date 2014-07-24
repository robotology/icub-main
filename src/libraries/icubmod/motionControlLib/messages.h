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
#include "stdint.h"
#include "canProtocolLib/iCubCanProtocol.h"
#include "canProtocolLib/iCubCanProto_types.h"


#define CAN_PROTOCOL_MAJOR          1 //this must be an exact match with the firmware protocol, otherwise terminates.
#define CAN_PROTOCOL_MINOR          2 //this must be smaller or equal than the firmware protocol, otherwise terminates.
#define LAST_BLL_BUILD             80 //this value is used only to print "please update" message
#define LAST_MC4_BUILD             80 //this value is used only to print "please update" message

//#define MODE_IDLE                   icubCanProto_controlmode_idle
//#define MODE_POSITION               icubCanProto_controlmode_position
//#define MODE_VELOCITY               icubCanProto_controlmode_velocity
//#define MODE_TORQUE                 icubCanProto_controlmode_torque
//#define MODE_IMPEDANCE_POS          icubCanProto_controlmode_impedance_pos
//#define MODE_IMPEDANCE_VEL          icubCanProto_controlmode_impedance_vel
//#define MODE_CALIB_ABS_POS_SENS     icubCanProto_controlmode_calib_abs_pos_sens
//#define MODE_CALIB_HARD_STOPS       icubCanProto_controlmode_calib_hard_stops
//#define MODE_HANDLE_HARD_STOPS      icubCanProto_controlmode_handle_hard_stops
//#define MODE_MARGIN_REACHED         icubCanProto_controlmode_margin_reached
//
//#define MODE_OPENLOOP               icubCanProto_controlmode_openloop

/*
 * this is 8 bits long, MSB is the channel (0 or 1). 
 */
//#define CAN_NO_MESSAGE              ICUBCANPROTO_POL_MC_CMD__NO_MESSAGE
//#define CAN_CONTROLLER_RUN          ICUBCANPROTO_POL_MC_CMD__CONTROLLER_RUN
//#define CAN_CONTROLLER_IDLE         ICUBCANPROTO_POL_MC_CMD__CONTROLLER_IDLE
//#define CAN_TOGGLE_VERBOSE          ICUBCANPROTO_POL_MC_CMD__TOGGLE_VERBOSE
//#define CAN_CALIBRATE_ENCODER       ICUBCANPROTO_POL_MC_CMD__CALIBRATE_ENCODER
//#define CAN_ENABLE_PWM_PAD          ICUBCANPROTO_POL_MC_CMD__ENABLE_PWM_PAD
//#define CAN_DISABLE_PWM_PAD         ICUBCANPROTO_POL_MC_CMD__DISABLE_PWM_PAD
//#define CAN_GET_CONTROL_MODE        ICUBCANPROTO_POL_MC_CMD__GET_CONTROL_MODE
//#define CAN_MOTION_DONE             ICUBCANPROTO_POL_MC_CMD__MOTION_DONE
//#define CAN_SET_CONTROL_MODE        ICUBCANPROTO_POL_MC_CMD__SET_CONTROL_MODE

//#define CAN_WRITE_FLASH_MEM         ICUBCANPROTO_POL_MC_CMD__WRITE_FLASH_MEM
//#define CAN_READ_FLASH_MEM          ICUBCANPROTO_POL_MC_CMD__READ_FLASH_MEM
//#define CAN_GET_ADDITIONAL_INFO     ICUBCANPROTO_POL_MC_CMD__GET_ADDITIONAL_INFO
//#define CAN_SET_ADDITIONAL_INFO     ICUBCANPROTO_POL_MC_CMD__SET_ADDITIONAL_INFO
//#define CAN_SET_SPEED_ESTIM_SHIFT   ICUBCANPROTO_POL_MC_CMD__SET_SPEED_ESTIM_SHIFT
//#define CAN_SET_DEBUG_PARAM         ICUBCANPROTO_POL_MC_CMD__SET_DEBUG_PARAM
//#define CAN_GET_DEBUG_PARAM         ICUBCANPROTO_POL_MC_CMD__GET_DEBUG_PARAM

//#define CAN_GET_ENCODER_POSITION    ICUBCANPROTO_POL_MC_CMD__GET_ENCODER_POSITION
//#define CAN_SET_DESIRED_POSITION    ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_POSITION
//#define CAN_GET_DESIRED_POSITION    ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_POSITION
//#define CAN_SET_DESIRED_VELOCITY    ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_VELOCITY
//#define CAN_GET_DESIRED_VELOCITY    ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_VELOCITY
//#define CAN_SET_DESIRED_ACCELER     ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_ACCELER
//#define CAN_GET_DESIRED_ACCELER     ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_ACCELER
//#define CAN_POSITION_MOVE           ICUBCANPROTO_POL_MC_CMD__POSITION_MOVE
//#define CAN_VELOCITY_MOVE           ICUBCANPROTO_POL_MC_CMD__VELOCITY_MOVE
//#define CAN_SET_ENCODER_POSITION    ICUBCANPROTO_POL_MC_CMD__SET_ENCODER_POSITION
//#define CAN_SET_P_GAIN              ICUBCANPROTO_POL_MC_CMD__SET_P_GAIN
//#define CAN_GET_P_GAIN              ICUBCANPROTO_POL_MC_CMD__GET_P_GAIN
//#define CAN_SET_D_GAIN              ICUBCANPROTO_POL_MC_CMD__SET_D_GAIN
//#define CAN_GET_D_GAIN              ICUBCANPROTO_POL_MC_CMD__GET_D_GAIN
//#define CAN_SET_I_GAIN              ICUBCANPROTO_POL_MC_CMD__SET_I_GAIN
//#define CAN_GET_I_GAIN              ICUBCANPROTO_POL_MC_CMD__GET_I_GAIN
//#define CAN_SET_ILIM_GAIN           ICUBCANPROTO_POL_MC_CMD__SET_ILIM_GAIN
//#define CAN_GET_ILIM_GAIN           ICUBCANPROTO_POL_MC_CMD__GET_ILIM_GAIN
//#define CAN_SET_OFFSET              ICUBCANPROTO_POL_MC_CMD__SET_OFFSET
//#define CAN_GET_OFFSET              ICUBCANPROTO_POL_MC_CMD__GET_OFFSET
//#define CAN_SET_SCALE               ICUBCANPROTO_POL_MC_CMD__SET_SCALE
//#define CAN_GET_SCALE               ICUBCANPROTO_POL_MC_CMD__GET_SCALE
//#define CAN_SET_TLIM                ICUBCANPROTO_POL_MC_CMD__SET_TLIM
//#define CAN_GET_TLIM                ICUBCANPROTO_POL_MC_CMD__GET_TLIM
//#define CAN_SET_DESIRED_TORQUE      ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_TORQUE
//#define CAN_GET_DESIRED_TORQUE      ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_TORQUE
//#define CAN_STOP_TRAJECTORY         ICUBCANPROTO_POL_MC_CMD__STOP_TRAJECTORY
//
//#define CAN_SET_BOARD_ID            ICUBCANPROTO_POL_MC_CMD__SET_BOARD_ID
//#define CAN_GET_BOARD_ID            ICUBCANPROTO_POL_MC_CMD__GET_BOARD_ID
//#define CAN_SET_TORQUE_SOURCE       52
//
//#define CAN_GET_PID_ERROR			55
//#define CAN_GET_ERROR_STATUS		60
//#define CAN_GET_ENCODER_VELOCITY	61
//#define CAN_SET_COMMAND_POSITION	62
//#define CAN_GET_PID_OUTPUT			63
//#define CAN_SET_MIN_POSITION		64
//#define CAN_GET_MIN_POSITION		65
//#define CAN_SET_MAX_POSITION		66
//#define CAN_GET_MAX_POSITION		67
//#define CAN_SET_MAX_VELOCITY		68
//#define CAN_GET_MAX_VELOCITY		69
//
//// special messages for inter board communication/synchronization
////#define CAN_GET_ACTIVE_ENCODER_POSITION 70
////#define CAN_SET_ACTIVE_ENCODER_POSITION 71
//
//#define CAN_SET_CURRENT_LIMIT		72
//#define CAN_SET_BCAST_POLICY		73
//
//#define CAN_SET_VEL_SHIFT			74
//
//#define CAN_SET_OFFSET_ABS_ENCODER  75
//#define CAN_GET_OFFSET_ABS_ENCODER  76
//
//#define CAN_SET_SMOOTH_PID          77
//#define CAN_SET_TORQUE_PID          78
//#define CAN_GET_TORQUE_PID          79
//#define CAN_SET_TORQUE_PIDLIMITS    80
//#define CAN_GET_TORQUE_PIDLIMITS    81
//#define CAN_SET_POS_PID             82
//#define CAN_GET_POS_PID             83
//#define CAN_SET_POS_PIDLIMITS       84
//#define CAN_GET_POS_PIDLIMITS   	85
//
//#define CAN_SET_VEL_TIMEOUT			86
//
//#define CAN_SET_IMPEDANCE_PARAMS    87
//#define CAN_GET_IMPEDANCE_PARAMS    88
//#define CAN_SET_IMPEDANCE_OFFSET    89
//#define CAN_GET_IMPEDANCE_OFFSET    90
//
//#define CAN_GET_FIRMWARE_VERSION    91
//#define CAN_SET_OPTICAL_ENC_RATIO   92
//#define CAN_SET_POS_STICTION_PARAMS     93
//#define CAN_GET_POS_STICTION_PARAMS     94
//#define CAN_SET_TORQUE_STICTION_PARAMS  95
//#define CAN_GET_TORQUE_STICTION_PARAMS  96
//
//#define CAN_SET_BACKEMF_PARAMS          97
//#define CAN_GET_BACKEMF_PARAMS          98
//
//#define CAN_SET_MODEL_PARAMS         99
//#define CAN_GET_MODEL_PARAMS        100
//
//#define CAN_SET_CURRENT_PID       	101
//#define CAN_GET_CURRENT_PID 		102
//#define CAN_SET_CURRENT_PIDLIMITS   103
//#define CAN_GET_CURRENT_PIDLIMITS 	104
//#define CAN_SET_VELOCITY_PID        105
//#define CAN_GET_VELOCITY_PID        106
//#define CAN_SET_VELOCITY_PIDLIMITS  107
//#define CAN_GET_VELOCITY_PIDLIMITS  108
//#define CAN_SET_DESIRED_CURRENT		109
//#define CAN_GET_DESIRED_CURRENT		110
//#define CAN_SET_PERIODIC_MSG_CONTENTS   111 //this message is used only for 2foc boards
//#define CAN_SET_I2T_PARAMS			112
//#define CAN_GET_I2T_PARAMS			113
//#define CAN_SET_OPENLOOP_PARAMS     114
//#define CAN_GET_OPENLOOP_PARAMS		115
//
//#define CAN_SET_OPENLOOP_PARAMS     114
//#define CAN_GET_OPENLOOP_PARAMS		115
#define CAN_SET_INTERACTION_MODE    116
#define CAN_GET_INTERACTION_MODE    117

//#define NUM_OF_MESSAGES             116

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
#define CAN_BCAST_NONE              0
//#define CAN_BCAST_POSITION          ICUBCANPROTO_PER_MC_MSG__POSITION
//#define CAN_BCAST_PID_VAL           ICUBCANPROTO_PER_MC_MSG__PID_VAL
//#define CAN_BCAST_STATUS            ICUBCANPROTO_PER_MC_MSG__STATUS
//#define CAN_BCAST_CURRENT           ICUBCANPROTO_PER_MC_MSG__CURRENT
//#define CAN_BCAST_OVERFLOW          ICUBCANPROTO_PER_MC_MSG__OVERFLOW
//#define CAN_BCAST_PRINT             ICUBCANPROTO_PER_MC_MSG__PRINT
//#define CAN_BCAST_VELOCITY          ICUBCANPROTO_PER_MC_MSG__VELOCITY
//#define CAN_BCAST_PID_ERROR         ICUBCANPROTO_PER_MC_MSG__PID_ERROR
//#define CAN_BCAST_DEBUG             ICUBCANPROTO_PER_MC_MSG__DEBUG
//#define CAN_BCAST_MOTOR_POSITION    ICUBCANPROTO_PER_MC_MSG__MOTOR_POSITION
//#define CAN_BCAST_MOTOR_SPEED       ICUBCANPROTO_PER_MC_MSG__MOTOR_SPEED
//#define CAN_BCAST_MAX_MSGS          ICUBCANPROTO_PER_MC_MSG_MAXNUM

#endif
