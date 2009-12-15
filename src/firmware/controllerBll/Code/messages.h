/*
 * messages.h
 *	macros for CAN bus message handling.
 *
 */
 
#ifndef __messagesh__
#define __messagesh__

/* error status values */
#define ERROR_NONE					0			/* no error, all ok */
#define ERROR_UNSPECIFIED			1			/* generic error */
#define ERROR_MODE					2			/* mode error, can't apply command in current mode */
#define ERROR_FMT					3			/* format error, command in wrong format */
#define ERROR_SEND					4			/* can't send answer back */

/**
 * it takes the existing header, swaps src and dest
 * leaves the channel number in place and the message
 * type. Doesn't change the priority either (3msb of 
 * the ID).
 */
#define PREPARE_HEADER\
{ \
	CAN_ID >>= 4; \
	CAN_ID &= 0xffffff0f; \
	if (axis <= 1) CAN_ID |= (_board_ID << 4); \
	else		  CAN_ID |= (_board_ID+1 << 4); \
}
extern char    _additional_info [32];

//-------------------------------------------------------------------
#define CAN_NO_MESSAGE_HANDLER(x) \
{ \
	_general_board_error = ERROR_UNSPECIFIED; \
}

//-------------------------------------------------------------------
#if VERSION == 0x0152 
#define CAN_CONTROLLER_RUN_HANDLER(x) \
	{ \
		if ((_pad_enabled[0]==false) || (_pad_enabled[1]==false))\
			can_printf("WARNING: RUN called before AMP");\
		if (((_control_mode[0] == MODE_IDLE) || (_control_mode[1] == MODE_IDLE)))  \
		{ \
		 	if ((_received_pid[0].rec_pid==0x7F) || (_received_pid[1].rec_pid==0x7F))   \
			{ \
				_control_mode[0] = MODE_POSITION; \
				_control_mode[1] = MODE_POSITION; \
				_desired[0] = _position[0]; \
				_desired[1] = _position[1]; \
				_integral[0] = 0; \
				_integral[1] = 0; \
				_set_point[0] = _position[0]; \
				_set_point[1] = _position[1]; \
				init_trajectory (0, _position[0], _position[0], 1); \
				init_trajectory (1, _position[1], _position[1], 1); \
				_general_board_error = ERROR_NONE; \
			} \
			else \
			{ \
			  can_printf("WARNING:PID IS NOT SET");\
			} \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	}
#else
	#define CAN_CONTROLLER_RUN_HANDLER(x) \
	{ \
		if (_pad_enabled[axis]==false)\
			can_printf("WARNING: RUN called before AMP");\
		else if ((_control_mode[axis] == MODE_IDLE) ) \
		{ \
			if (_received_pid[axis].rec_pid==0x7F) \
			{ \
				_control_mode[axis] = MODE_POSITION; \
				_desired[axis] = _position[axis]; \
				_integral[axis] = 0; \
				_set_point[axis] = _position[axis]; \
				init_trajectory (axis, _position[axis], _position[axis], 1); \
				_general_board_error = ERROR_NONE; \
			} \
			else \
			{ \
			  can_printf("WARNING:PID IS NOT SET");\
			} \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	}
#endif
//messo else if 
//-------------------------------------------------------------------

#define CAN_CONTROLLER_IDLE_HANDLER(x) \
{ \
	if (_control_mode[axis] != MODE_IDLE) \
	{ \
		_control_mode[axis] = MODE_IDLE; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_MODE; \
}

//-------------------------------------------------------------------
#define CAN_TOGGLE_VERBOSE_HANDLER(x) \
{ \
	_verbose = !_verbose; \
}

//-------------------------------------------------------------------
#define CAN_CALIBRATE_ENCODER_HANDLER(x) \
{ \
	calibrate (axis, CAN_DATA[1], BYTE_W(CAN_DATA[2], CAN_DATA[3]), \
								  BYTE_W(CAN_DATA[4], CAN_DATA[5]), \
   								  BYTE_W(CAN_DATA[6], CAN_DATA[7])); \
	_calibrated[axis] = false; \
	_general_board_error = ERROR_NONE; \
}
//-------------------------------------------------------------------
#if VERSION == 0x0152 
	#define CAN_ENABLE_PWM_PAD_HANDLER(x) \
	{ \
		if (_pad_enabled[0] == false &&	_pad_enabled[1] == false) \
		{ \
			PWM_outputPadEnable(0); \
			PWM_outputPadEnable(1); \
			_general_board_error = ERROR_NONE; \
			can_printf("PWM ENA COUPLED:0 & 1");\
		} \
	}
#else
	#define CAN_ENABLE_PWM_PAD_HANDLER(x) \
	{ \
		PWM_outputPadEnable(axis); \
		_control_mode[axis] = MODE_IDLE; \
		_general_board_error = ERROR_NONE; \
		can_printf("PWM ENA:%d",axis);\
	}
#endif

//-------------------------------------------------------------------
#if VERSION == 0x0152 
	#define CAN_DISABLE_PWM_PAD_HANDLER(x) \
	{ \
		PWM_outputPadDisable(0); \
		PWM_outputPadDisable(1); \
		_pad_enabled[0] = false; \
		_pad_enabled[1] = false; \
		_general_board_error = ERROR_NONE; \
		can_printf("PWM DIS COUPLED:0 & 1");\
	}
#else
	#define CAN_DISABLE_PWM_PAD_HANDLER(x) \
	{ \
		PWM_outputPadDisable(axis); \
		_pad_enabled[axis] = false; \
		_general_board_error = ERROR_NONE; \
		can_printf("PWM DIS:%d",axis);\
	}
#endif

//-------------------------------------------------------------------
#define CAN_SET_VEL_SHIFT_HANDLER(x) \
{ \
	byte value = 0; \
	if (CAN_LEN == 3) \
	{ \
		value = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		if (value>=0 && value <=16) _vel_shift[axis] = value; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_CONTROL_MODE_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = _control_mode[axis]; \
		CAN_DATA[2] = 0; \
		CAN1_send( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_CONTROL_MODE_HANDLER(x) \
{ \
	byte value = 0; \
	if (CAN_LEN == 2) \
	{ \
		can_printf("CONTROL MODE SETTED"); \
		value = (CAN_DATA[1]); \
		if (value>=0 && value <=0x50) _control_mode[axis] = value; \
		_general_board_error = ERROR_NONE; \
		_desired_torque[axis]=0; \
		_desired[axis] = _position[axis]; \
		_integral[axis] = 0; \
		_set_point[axis] = _position[axis]; \
		init_trajectory (axis, _position[axis], _position[axis], 1); \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_MOTION_DONE_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		/* CAN_DATA[1] untouched */ \
		CAN_DATA[1] = BYTE_H(_in_position[axis]); \
		CAN_DATA[2] = BYTE_L(_in_position[axis]); \
		CAN1_send( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_WRITE_FLASH_MEM_HANDLER(x) \
{ \
	writeToFlash (_flash_addr); \
	_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_READ_FLASH_MEM_HANDLER(x) \
{ \
	readFromFlash (_flash_addr); \
	_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#if 0
#define CAN_GET_ACTIVE_ENCODER_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_ID |= 0x100; \
		CAN_LEN = 8; \
		CAN_DATA[0] = BYTE_4(_position[0]); \
		CAN_DATA[1] = BYTE_3(_position[0]); \
		CAN_DATA[2] = BYTE_2(_position[0]); \
		CAN_DATA[3] = BYTE_1(_position[0]); \
		CAN_DATA[4] = BYTE_4(_position[1]); \
		CAN_DATA[5] = BYTE_3(_position[1]); \
		CAN_DATA[6] = BYTE_2(_position[1]); \
		CAN_DATA[7] = BYTE_1(_position[1]); \
		if (CAN1_sendFrame (1, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA) != ERR_OK) \
			AS1_printStringEx ("err 70\r\n"); \
		_general_board_error = ERROR_NONE; \
}
#endif

//-------------------------------------------------------------------
// shift by 2 is because data is available every 4 control cycles 
#if VERSION == 0x0153 || VERSION == 0x0173
#define CAN_SET_ACTIVE_ENCODER_POSITION_HANDLER(x) \
{ \
	long value; \
	if (CAN_LEN == 8) \
	{ \
		value = BYTE_C(CAN_DATA[0], CAN_DATA[1], CAN_DATA[2], CAN_DATA[3]); \
		_cpl_pos_delta[0] = L_sub (value, _cpl_pos_received[0]) >> 2; \
		_cpl_pos_prediction[0] = value; \
		_cpl_pos_received[0] = value; \
		\
		value = BYTE_C(CAN_DATA[4], CAN_DATA[5], CAN_DATA[6], CAN_DATA[7]); \
		_cpl_pos_delta[1] = L_sub (value, _cpl_pos_received[1]) >> 2; \
		_cpl_pos_prediction[1] = value; \
		_cpl_pos_received[1] = value; \
		\
		_pending_request = false; \
		_timeout = 0; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
#endif

//-------------------------------------------------------------------
#if VERSION == 0x0153 || VERSION == 0x0173
#define CAN_SET_ACTIVE_PID_HANDLER(x) \
{ \
	Int16 value; \
	if (CAN_LEN == 8) \
	{ \
		value = BYTE_W(CAN_DATA[4], CAN_DATA[5]); \
		_cpl_pid_delta[0] = (value - _cpl_pid_received[0]) >> 2; \
		_cpl_pid_prediction[0] = value; \
		_cpl_pid_received[0] = value; \
		\
		value = BYTE_W(CAN_DATA[6], CAN_DATA[7]); \
		_cpl_pid_delta[1] = (value - _cpl_pid_received[1]) >> 2; \
		_cpl_pid_prediction[1] = value; \
		_cpl_pid_received[1] = value; \
		\
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
#endif

//-------------------------------------------------------------------
#if VERSION == 0x0153 || VERSION == 0x0173
#define CAN_SET_ACTIVE_ERROR_HANDLER(x) \
{ \
	Int16 value; \
	if (CAN_LEN == 8) \
	{ \
		value = BYTE_W(CAN_DATA[4], CAN_DATA[5]); \
		_cpl_err[0] = value; \
		\
		value = BYTE_W(CAN_DATA[6], CAN_DATA[7]); \
		_cpl_err[1] = value; \
		\
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
#endif

//-------------------------------------------------------------------
#define CAN_GET_ENCODER_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 7; \
		CAN_DATA[1] = BYTE_4(_position[axis]); \
		CAN_DATA[2] = BYTE_3(_position[axis]); \
		CAN_DATA[3] = BYTE_2(_position[axis]); \
		CAN_DATA[4] = BYTE_1(_position[axis]); \
		CAN_DATA[5] = BYTE_2(_speed[axis]); \
		CAN_DATA[6] = BYTE_1(_speed[axis]); \
		if (CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA) != ERR_OK) \
			AS1_printStringEx ("err 20\r\n"); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_ENCODER_POSITION_HANDLER(x) \
{ \
	long value; \
	if (CAN_LEN == 5) \
	{ \
		value = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
/*
	set_position_encoder (axis, value); \
		_position[axis] = value; \
		_position_old[axis] = value; \
		_integral[axis] = 0; \
*/	
//-------------------------------------------------------------------
#define CAN_SET_COMMAND_POSITION_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		_desired[axis] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		abort_trajectory (axis, _desired[axis]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
	{ \
		_general_board_error = ERROR_FMT; \
	} \
} 

//-------------------------------------------------------------------
#define CAN_SET_DESIRED_POSITION_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		_set_point[axis] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_DESIRED_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 5; \
		CAN_DATA[1] = BYTE_4(_set_point[axis]); \
		CAN_DATA[2] = BYTE_3(_set_point[axis]); \
		CAN_DATA[3] = BYTE_2(_set_point[axis]); \
		CAN_DATA[4] = BYTE_1(_set_point[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_MIN_POSITION_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		_min_position[axis] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_MIN_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	CAN_LEN = 5; \
	CAN_DATA[1] = BYTE_4(_min_position[axis]); \
	CAN_DATA[2] = BYTE_3(_min_position[axis]); \
	CAN_DATA[3] = BYTE_2(_min_position[axis]); \
	CAN_DATA[4] = BYTE_1(_min_position[axis]); \
	CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
	_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_MAX_POSITION_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		_max_position[axis] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_MAX_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 5; \
		CAN_DATA[1] = BYTE_4(_max_position[axis]); \
		CAN_DATA[2] = BYTE_3(_max_position[axis]); \
		CAN_DATA[3] = BYTE_2(_max_position[axis]); \
		CAN_DATA[4] = BYTE_1(_max_position[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_OFFSET_ABS_ENCODER_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		Int32  value = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		if (value >=0 && value <=4095) set_max_position(axis, value); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_OFFSET_ABS_ENCODER_HANDLER(x) \
{ \
	Int32  value = get_max_position(axis); \
	PREPARE_HEADER; \
		CAN_LEN = 5; \
		CAN_DATA[1] = BYTE_4(value); \
		CAN_DATA[2] = BYTE_3(value); \
		CAN_DATA[3] = BYTE_2(value); \
		CAN_DATA[4] = BYTE_1(value); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_MAX_VELOCITY_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_max_vel[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_MAX_VELOCITY_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_max_vel[axis]); \
		CAN_DATA[2] = BYTE_L(_max_vel[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_POSITION_MOVE_HANDLER(x) \
{ \
	if (CAN_LEN == 7) \
	{ \
		/*if (_control_mode[axis] == MODE_MARGIN_REACHED)	\
		{	\
			_general_board_error = ERROR_NONE;	\
		}	\
		else */ if (_control_mode[axis] != MODE_IDLE) \
		{ \
			if (_control_mode[axis] != MODE_IMPEDANCE) _control_mode[axis] = MODE_POSITION; \
			_set_point[axis] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
			if (_set_point[axis] < _min_position[axis]) \
				_set_point[axis] = _min_position[axis]; \
			else \
			if (_set_point[axis] > _max_position[axis]) \
				_set_point[axis] = _max_position[axis]; \
			_set_vel[axis] = BYTE_W(CAN_DATA[5], CAN_DATA[6]); \
			if (_set_vel[axis] < 1) \
				_set_vel[axis] = 1; \
			/* _set_vel needs to be checked */ \
			_set_acc[axis] = 0; \
			init_trajectory (axis, _desired[axis], _set_point[axis], _set_vel[axis]); \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_VELOCITY_MOVE_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		if (_control_mode[axis] != MODE_IDLE && IS_DONE(axis)) \
		{ \
			if (_control_mode[axis] == MODE_POSITION) \
				_desired_vel[axis] = 0; \
			_control_mode[axis] = MODE_VELOCITY; \
			_set_point[axis] = 0; \
			_set_vel[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
			_set_acc[axis] = BYTE_W(CAN_DATA[3], CAN_DATA[4]); \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_SET_DESIRED_VELOCITY_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_set_vel[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_DESIRED_VELOCITY_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_set_vel[axis]); \
		CAN_DATA[2] = BYTE_L(_set_vel[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_GET_ENCODER_VELOCITY_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_speed[axis]); \
		CAN_DATA[2] = BYTE_L(_speed[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_DESIRED_ACCELER_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_set_acc[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_DESIRED_ACCELER_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_set_acc[axis]); \
		CAN_DATA[2] = BYTE_L(_set_acc[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_DESIRED_TORQUE_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		_desired_torque[axis] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_DESIRED_TORQUE_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 5; \
		CAN_DATA[1] = BYTE_4(_desired_torque[axis]); \
		CAN_DATA[2] = BYTE_3(_desired_torque[axis]); \
		CAN_DATA[3] = BYTE_2(_desired_torque[axis]); \
		CAN_DATA[4] = BYTE_1(_desired_torque[axis]); \
		CAN1_send(CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
// GENERIC DEBUG PARAMETER
#define CAN_SET_DEBUG_PARAM_1_HANDLER(x) \
{ \
	word tmp; \
	if (CAN_DATA[1] == 0) \
	{ \
		tmp = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
	} \
	else if (CAN_DATA[1] == 1) \
	{ \
		tmp = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
	} \
}

/*
// used to change the speed of the main loop
if (CAN_LEN == 4) \
{ \
	word tmp; \
	tmp = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
	_t1c = CAN_DATA[3]; \
	_general_board_error = ERROR_NONE; \
	setReg (TMRA3_CMP1, tmp); \
} \
else if (CAN_LEN == 3) \
{ \
	word tmp; \
	tmp = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
	_t1c = 0; \
	_general_board_error = ERROR_NONE; \
	setReg (TMRA3_CMP1, tmp); \
} \
else \
{ \
	_general_board_error = ERROR_FMT; \
	setReg (TMRA3_CMP1, 39999); \
} \
*/
	
//-------------------------------------------------------------------
// GENERIC DEBUG PARAMETER
#define CAN_SET_DEBUG_PARAM_2_HANDLER(x) \
{ \
	word tmp; \
	if (CAN_DATA[1] == 0) \
	{ \
		tmp = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
	} \
	else if (CAN_DATA[1] == 1) \
	{ \
		tmp = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
	} \
}

//-------------------------------------------------------------------
// GENERIC DEBUG PARAMETER
#define CAN_GET_DEBUG_PARAM_1_HANDLER(x) \
{ \
	if (CAN_DATA[1] == 0) \
	{ \
		PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = 0; \
		CAN_DATA[2] = 0; \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
}

//-------------------------------------------------------------------
// GENERIC DEBUG PARAMETER
#define CAN_GET_DEBUG_PARAM_2_HANDLER(x) \
{ \
	if (CAN_DATA[1] == 0) \
	{ \
		PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = 0; \
		CAN_DATA[2] = 0; \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
}

//-------------------------------------------------------------------
#define CAN_GET_PID_OUTPUT_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_pid[axis]); \
		CAN_DATA[2] = BYTE_L(_pid[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_GET_PID_ERROR_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_error[axis]); \
		CAN_DATA[2] = BYTE_L(_error[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_TORQUE_PID_HANDLER(x) \
{ \
	if (CAN_LEN == 8) \
	{ \
		_kp_torque[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_ki_torque[axis] = BYTE_W(CAN_DATA[3], CAN_DATA[4]); \
		_kd_torque[axis] = BYTE_W(CAN_DATA[5], CAN_DATA[6]); \
		_kr_torque[axis] = (CAN_DATA[7]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
//-------------------------------------------------------------------
#define CAN_GET_TORQUE_PID_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 8; \
		CAN_DATA[1] = BYTE_H(_kp_torque[axis]); \
		CAN_DATA[2] = BYTE_L(_kp_torque[axis]); \
		CAN_DATA[3] = BYTE_H(_ki_torque[axis]); \
		CAN_DATA[4] = BYTE_L(_ki_torque[axis]); \
		CAN_DATA[5] = BYTE_H(_kd_torque[axis]); \
		CAN_DATA[6] = BYTE_L(_kd_torque[axis]); \
		CAN_DATA[7] = BYTE_H(_kr_torque[axis]); \
		CAN1_send(CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_TORQUE_PIDLIMITS_HANDLER(x) \
{ \
	if (CAN_LEN == 8) \
	{ \
		_ko_torque[axis] 				= BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_pid_limit_torque[axis] 		= BYTE_W(CAN_DATA[3], CAN_DATA[4]); \
		_integral_limit_torque[axis] 	= BYTE_W(CAN_DATA[5], CAN_DATA[6]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
//-------------------------------------------------------------------
#define CAN_GET_TORQUE_PIDLIMITS_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 8; \
		CAN_DATA[1] = BYTE_H(_ko_torque[axis]); \
		CAN_DATA[2] = BYTE_L(_ko_torque[axis]); \
		CAN_DATA[3] = BYTE_H(_pid_limit_torque[axis]); \
		CAN_DATA[4] = BYTE_L(_pid_limit_torque[axis]); \
		CAN_DATA[5] = BYTE_H(_integral_limit_torque[axis]); \
		CAN_DATA[6] = BYTE_L(_integral_limit_torque[axis]); \
		CAN_DATA[7] = 0; \
		CAN1_send(CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_POS_PID_HANDLER(x) \
{ \
	if (CAN_LEN == 8) \
	{ \
		_kp[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_ki[axis] = BYTE_W(CAN_DATA[3], CAN_DATA[4]); \
		_kd[axis] = BYTE_W(CAN_DATA[5], CAN_DATA[6]); \
		_kr[axis] = (CAN_DATA[7]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_POS_PID_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 8; \
		CAN_DATA[1] = BYTE_H(_kp[axis]); \
		CAN_DATA[2] = BYTE_L(_kp[axis]); \
		CAN_DATA[3] = BYTE_H(_ki[axis]); \
		CAN_DATA[4] = BYTE_L(_ki[axis]); \
		CAN_DATA[5] = BYTE_H(_kd[axis]); \
		CAN_DATA[6] = BYTE_L(_kd[axis]); \
		CAN_DATA[7] = BYTE_H(_kr[axis]); \
		CAN1_send(CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_POS_PIDLIMITS_HANDLER(x) \
{ \
	if (CAN_LEN == 8) \
	{ \
		_ko[axis] 				= BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_pid_limit[axis] 		= BYTE_W(CAN_DATA[3], CAN_DATA[4]); \
		_integral_limit[axis] 	= BYTE_W(CAN_DATA[5], CAN_DATA[6]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
//-------------------------------------------------------------------
#define CAN_GET_POS_PIDLIMITS_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 8; \
		CAN_DATA[1] = BYTE_H(_ko[axis]); \
		CAN_DATA[2] = BYTE_L(_ko[axis]); \
		CAN_DATA[3] = BYTE_H(_pid_limit[axis]); \
		CAN_DATA[4] = BYTE_L(_pid_limit[axis]); \
		CAN_DATA[5] = BYTE_H(_integral_limit[axis]); \
		CAN_DATA[6] = BYTE_L(_integral_limit[axis]); \
		CAN_DATA[7] = 0; \
		CAN1_send(CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_P_GAIN_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_kp[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_received_pid[axis].rec_pid_bits.kp =true; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_P_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_kp[axis]); \
		CAN_DATA[2] = BYTE_L(_kp[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_D_GAIN_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_kd[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_received_pid[axis].rec_pid_bits.kd =true; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_D_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_kd[axis]); \
		CAN_DATA[2] = BYTE_L(_kd[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_I_GAIN_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_ki[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_integral[axis] = 0; \
		_received_pid[axis].rec_pid_bits.ki =true; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_I_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_ki[axis]); \
		CAN_DATA[2] = BYTE_L(_ki[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_ILIM_GAIN_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_integral_limit[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
		_received_pid[axis].rec_pid_bits.ilim =true; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_ILIM_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_integral_limit[axis]); \
		CAN_DATA[2] = BYTE_L(_integral_limit[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_OFFSET_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_ko[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
		_received_pid[axis].rec_pid_bits.ko =true; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_OFFSET_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_ko[axis]); \
		CAN_DATA[2] = BYTE_L(_ko[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_SCALE_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_kr[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_received_pid[axis].rec_pid_bits.kr =true; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_SCALE_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_kr[axis]); \
		CAN_DATA[2] = BYTE_L(_kr[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_TLIM_HANDLER(x) \
{ \
	if (CAN_LEN == 3) \
	{ \
		_pid_limit[axis] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
		_received_pid[axis].rec_pid_bits.tlim =true; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_TLIM_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_pid_limit[axis]); \
		CAN_DATA[2] = BYTE_L(_pid_limit[axis]); \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_CURRENT_LIMIT_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		dword tmp; \
		tmp = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		if (tmp<=MAX_CURRENT) _max_allowed_current[axis]=tmp;\
		else can_printf("MAX CURRENT BIGGER THEN MAX:%d",axis);\
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//	_conversion_factor[axis] = (tmp * 3.3) / 32760.0f; \

//-------------------------------------------------------------------
#define CAN_GET_ERROR_STATUS_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 3; \
		CAN_DATA[1] = 0x00; \
		CAN_DATA[2] = _general_board_error; \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_BOARD_ID_HANDLER(x) \
{ \
	byte value = 0; \
	if (CAN_LEN == 2) \
	{ \
		value = CAN_DATA[1]; \
		if (value>=1 && value <=15) _board_ID = value; \
		CAN1_init (_board_ID); \
		set_can_masks(); \
		writeToFlash (_flash_addr); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

//-------------------------------------------------------------------
#define CAN_GET_BOARD_ID_HANDLER(x) \
{ \
	PREPARE_HEADER; \
		CAN_LEN = 2; \
		CAN_DATA[1] = _board_ID; \
		CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
}

//-------------------------------------------------------------------
#define CAN_SET_ADDITIONAL_INFO_HANDLER(x) \
{ \
	can_receive_additional_info(); \
	writeToFlash (_flash_addr); \
}

//-------------------------------------------------------------------
#define CAN_GET_ADDITIONAL_INFO_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	can_send_additional_info(); \
}

//-------------------------------------------------------------------
#define CAN_SET_BCAST_POLICY_HANDLER(x) \
{ \
	if (CAN_LEN == 5) \
	{ \
		broadcast_mask = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		if (broadcast_mask & (1<<(CAN_BCAST_PRINT-1))) enable_can_print(); \
		else disable_can_print(); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

// end of messages 
#endif

