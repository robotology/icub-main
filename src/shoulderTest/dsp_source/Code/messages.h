/*
 * messages.h
 *	macros for CAN bus message handling.
 *
 */
 
#ifndef __messagesh__
#define __messagesh__

/**
 * it takes the existing header, swaps src and dest
 * leaves the channel number in place and the message
 * type. Doesn't change the priority either (3msb of 
 * the ID).
 */
#define PREPARE_HEADER \
{ \
	CAN_ID >>= 4; \
	CAN_ID &= 0xffffff0f; \
	CAN_ID |= (_board_ID << 4); \
}

#define CAN_NO_MESSAGE_HANDLER(x) \
{ \
	_general_board_error = ERROR_UNSPECIFIED; \
}

#define CAN_CONTROLLER_RUN_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (_control_mode[i] == MODE_IDLE) \
	{ \
		_control_mode[i] = MODE_POSITION; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_MODE; \
}

#define CAN_CONTROLLER_IDLE_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (_control_mode[i] != MODE_IDLE) \
	{ \
		_control_mode[i] = MODE_IDLE; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_MODE; \
}

#define CAN_TOGGLE_VERBOSE_HANDLER(x) \
{ \
	_verbose = !_verbose; \
}

#define CAN_CALIBRATE_ENCODER_HANDLER(x) \
{ \
	if (CHANNEL(CAN_DATA[0]) == 0) \
		calibrate (0); \
	else \
		calibrate (1); \
	_general_board_error = ERROR_NONE; \
}

#define CAN_ENABLE_PWM_PAD_HANDLER(x) \
{ \
	if (CHANNEL(CAN_DATA[0]) == 0) \
	{ \
		if (_calibrated[0]) \
		{ \
			PWMC0_outputPadEnable(); \
			_pad_enabled[0] = true; \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
	else \
	{ \
		if (_calibrated[1]) \
		{ \
			PWMC1_outputPadEnable(); \
			_pad_enabled[1] = true; \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
}

#define CAN_DISABLE_PWM_PAD_HANDLER(x) \
{ \
	if (CHANNEL(CAN_DATA[0]) == 0) \
	{ \
		if (_calibrated[0]) \
		{ \
			PWMC0_outputPadDisable(); \
			_pad_enabled[0] = false; \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
	else \
	{ \
		if (_calibrated[1]) \
		{ \
			PWMC1_outputPadDisable(); \
			_pad_enabled[1] = false; \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
}

#define CAN_GET_CONTROL_MODE_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 2; \
		CAN_DATA[1] = _control_mode[i]; \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_MOTION_DONE_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		/* CAN_DATA[1] untouched */ \
		CAN_DATA[1] = BYTE_H(_in_position[i]); \
		CAN_DATA[2] = BYTE_L(_in_position[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_WRITE_FLASH_MEM_HANDLER(x) \
{ \
	writeToFlash (_flash_addr); \
	_general_board_error = ERROR_NONE; \
}

#define CAN_READ_FLASH_MEM_HANDLER(x) \
{ \
	readFromFlash (_flash_addr); \
	_general_board_error = ERROR_NONE; \
}


#if 0
#define CAN_GET_ACTIVE_ENCODER_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
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
	} \
	else \
	{ \
		_general_board_error = ERROR_SEND; \
		AS1_printStringEx ("err 70 - no msg\r\n"); \
	} \
}
#endif

/* 
 * shift by 2 is because data is available every 4 control cycles 
 */
#if VERSION == 0x0113
#define CAN_SET_ACTIVE_ENCODER_POSITION_HANDLER(x) \
{ \
	long value; \
	if (CAN_LEN == 8) \
	{ \
		value = BYTE_C(CAN_DATA[0], CAN_DATA[1], CAN_DATA[2], CAN_DATA[3]); \
		_adjustment[0] = _other_position[0]; \
		_delta_adj[0] = L_sub (value, _other_position[0]) >> 2; \
		_adjustment[0] = L_add (_adjustment[0], _delta_adj[0]); \
		_other_position[0] = value; \
		\
		value = BYTE_C(CAN_DATA[4], CAN_DATA[5], CAN_DATA[6], CAN_DATA[7]); \
		_adjustment[1] = _other_position[1]; \
		_delta_adj[1] = L_sub (value, _other_position[1]) >> 2; \
		_adjustment[1] = L_add (_adjustment[1], _delta_adj[1]); \
		_other_position[1] = value; \
		\
		_pending_request = false; \
		_timeout = 0; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}
#endif


#define CAN_GET_ENCODER_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 7; \
		CAN_DATA[1] = BYTE_4(_position[i]); \
		CAN_DATA[2] = BYTE_3(_position[i]); \
		CAN_DATA[3] = BYTE_2(_position[i]); \
		CAN_DATA[4] = BYTE_1(_position[i]); \
		CAN_DATA[5] = BYTE_2(_speed[i]); \
		CAN_DATA[6] = BYTE_1(_speed[i]); \
		if (CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA) != ERR_OK) \
			AS1_printStringEx ("err 20\r\n"); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_ENCODER_POSITION_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	long value; \
	if (CAN_LEN == 5) \
	{ \
		value = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		if (i == 0) \
			QD0_setPosition (value); \
		else \
			QD1_setPosition (value); \
		_position[i] = value; \
		_position_old[i] = value; \
		_integral[i] = 0; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_SET_COMMAND_POSITION_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 5) \
	{ \
		_desired[i] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		abort_trajectory (i, _desired[i]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
	{ \
		_general_board_error = ERROR_FMT; \
	} \
} 

#define CAN_SET_DESIRED_POSITION_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 5) \
	{ \
		_set_point[i] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_DESIRED_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 5; \
		CAN_DATA[1] = BYTE_4(_set_point[i]); \
		CAN_DATA[2] = BYTE_3(_set_point[i]); \
		CAN_DATA[3] = BYTE_2(_set_point[i]); \
		CAN_DATA[4] = BYTE_1(_set_point[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_MIN_POSITION_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
/*	int value; */ \
	if (CAN_LEN == 5) \
	{ \
		_min_position[i] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_MIN_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 5; \
		CAN_DATA[1] = BYTE_4(_min_position[i]); \
		CAN_DATA[2] = BYTE_3(_min_position[i]); \
		CAN_DATA[3] = BYTE_2(_min_position[i]); \
		CAN_DATA[4] = BYTE_1(_min_position[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_MAX_POSITION_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	/* int value; */\
	if (CAN_LEN == 5) \
	{ \
		_max_position[i] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_MAX_POSITION_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 5; \
		CAN_DATA[1] = BYTE_4(_max_position[i]); \
		CAN_DATA[2] = BYTE_3(_max_position[i]); \
		CAN_DATA[3] = BYTE_2(_max_position[i]); \
		CAN_DATA[4] = BYTE_1(_max_position[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_MAX_VELOCITY_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_max_vel[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_MAX_VELOCITY_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_max_vel[i]); \
		CAN_DATA[2] = BYTE_L(_max_vel[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_POSITION_MOVE_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 7) \
	{ \
		if (_control_mode[i] != MODE_IDLE && IS_DONE(i)) \
		{ \
			_control_mode[i] = MODE_POSITION; \
			_set_point[i] = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
			if (_set_point[i] < _min_position[i]) \
				_set_point[i] = _min_position[i]; \
			else \
			if (_set_point[i] > _max_position[i]) \
				_set_point[i] = _max_position[i]; \
			_set_vel[i] = BYTE_W(CAN_DATA[5], CAN_DATA[6]); \
			if (_set_vel[i] < 1) \
				_set_vel[i] = 1; \
			/* _set_vel needs to be checked */ \
			_set_acc[i] = 0; \
			init_trajectory (i, _desired[i], _set_point[i], _set_vel[i]); \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_VELOCITY_MOVE_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 5) \
	{ \
		if (_control_mode[i] != MODE_IDLE && IS_DONE(i)) \
		{ \
			if (_control_mode[i] == MODE_POSITION) \
				_desired_vel[i] = 0; \
			_control_mode[i] = MODE_VELOCITY; \
			_set_point[i] = 0; \
			_set_vel[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
			_set_acc[i] = BYTE_W(CAN_DATA[3], CAN_DATA[4]); \
			_general_board_error = ERROR_NONE; \
		} \
		else \
			_general_board_error = ERROR_MODE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_SET_DESIRED_VELOCITY_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_set_vel[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_DESIRED_VELOCITY_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_set_vel[i]); \
		CAN_DATA[2] = BYTE_L(_set_vel[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_GET_ENCODER_VELOCITY_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_speed[i]); \
		CAN_DATA[2] = BYTE_L(_speed[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_DESIRED_ACCELER_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_set_acc[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_DESIRED_ACCELER_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_set_acc[i]); \
		CAN_DATA[2] = BYTE_L(_set_acc[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_GET_PID_OUTPUT_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_pid[i]); \
		CAN_DATA[2] = BYTE_L(_pid[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_GET_PID_ERROR_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_error[i]); \
		CAN_DATA[2] = BYTE_L(_error[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_P_GAIN_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_kp[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_P_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_kp[i]); \
		CAN_DATA[2] = BYTE_L(_kp[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_D_GAIN_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_kd[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_D_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_kd[i]); \
		CAN_DATA[2] = BYTE_L(_kd[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_I_GAIN_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_ki[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_integral[i] = 0; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_I_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_ki[i]); \
		CAN_DATA[2] = BYTE_L(_ki[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_ILIM_GAIN_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_integral_limit[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_ILIM_GAIN_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_integral_limit[i]); \
		CAN_DATA[2] = BYTE_L(_integral_limit[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_OFFSET_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_ko[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_OFFSET_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_ko[i]); \
		CAN_DATA[2] = BYTE_L(_ko[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_SCALE_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_kr[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_SCALE_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_kr[i]); \
		CAN_DATA[2] = BYTE_L(_kr[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_TLIM_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 3) \
	{ \
		_pid_limit[i] = BYTE_W(CAN_DATA[1], CAN_DATA[2]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_TLIM_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		int i = CHANNEL(CAN_DATA[0]); \
		CAN_LEN = 3; \
		CAN_DATA[1] = BYTE_H(_pid_limit[i]); \
		CAN_DATA[2] = BYTE_L(_pid_limit[i]); \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_CURRENT_LIMIT_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 5) \
	{ \
		dword tmp; \
		tmp = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_conversion_factor[i] = (tmp * 3.3) / 360360.0f; \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_GET_ERROR_STATUS_HANDLER(x) \
{ \
	PREPARE_HEADER; \
	if (CAN1_getStateTX () != 0) \
	{ \
		CAN_LEN = 3; \
		CAN_DATA[1] = 0x00; \
		CAN_DATA[2] = _general_board_error; \
		CAN1_sendFrame (CAN_BUFFER, CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_SEND; \
}

#define CAN_SET_BOARD_ID_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 2) \
	{ \
		/* address is in CAN_DATA[2] */ \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

#define CAN_SET_BCAST_POLICY_HANDLER(x) \
{ \
	int i = CHANNEL(CAN_DATA[0]); \
	if (CAN_LEN == 5) \
	{ \
		_broadcast_mask = BYTE_C(CAN_DATA[1], CAN_DATA[2], CAN_DATA[3], CAN_DATA[4]); \
		_general_board_error = ERROR_NONE; \
	} \
	else \
		_general_board_error = ERROR_FMT; \
}

/* end of messages */
#endif

