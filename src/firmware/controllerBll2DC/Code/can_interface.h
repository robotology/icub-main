
#ifndef __can_interfaceh__
#define __can_interfaceh__

/******************************************************/
// can interface prototypes 
/******************************************************/

void can_interface_init (byte channels_num);
byte can_interface (void);
void can_send_broadcast(void);
void set_can_masks();
void print_can (byte data[], byte len, char c);
void create_F_M(UInt32 *filter,UInt32 *mask,byte _class1, byte _source1, byte _dest1, byte _class2, byte _source2, byte _dest2);
void setmask(UInt32 filter1, UInt32 filter2, UInt32 mask1,UInt32 mask2);
void can_send_additional_info();
void can_receive_additional_info();

/******************************************************/
// can messages
/******************************************************/

//-----------------------------------------------------
// this is 8 bits long, MSB is the channel (0 or 1). 

#define CAN_NO_MESSAGE				0
#define CAN_CONTROLLER_RUN		 	1
#define CAN_CONTROLLER_IDLE			2
#define CAN_TOGGLE_VERBOSE			3
#define CAN_CALIBRATE_ENCODER		4
#define CAN_ENABLE_PWM_PAD			5
#define CAN_DISABLE_PWM_PAD			6
#define CAN_GET_CONTROL_MODE		7
#define CAN_MOTION_DONE				8

#define CAN_WRITE_FLASH_MEM			10
#define CAN_READ_FLASH_MEM			11
#define CAN_GET_ADDITIONAL_INFO		12
#define CAN_SET_ADDITIONAL_INFO		13

#define CAN_GET_ENCODER_POSITION	20
#define CAN_SET_DESIRED_POSITION	21
#define CAN_GET_DESIRED_POSITION	22
#define CAN_SET_DESIRED_VELOCITY	23
#define CAN_GET_DESIRED_VELOCITY	24
#define CAN_SET_DESIRED_ACCELER		25
#define CAN_GET_DESIRED_ACCELER		26

#define CAN_SET_ENCODER_POSITION	29
#define CAN_GET_ENCODER_VELOCITY	61
#define CAN_SET_COMMAND_POSITION	62

#define CAN_POSITION_MOVE			27
#define CAN_VELOCITY_MOVE			28

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
#define CAN_SET_DEBUG_PARAM_1		46
#define CAN_GET_DEBUG_PARAM_1		47
#define CAN_SET_DEBUG_PARAM_2		48
#define CAN_GET_DEBUG_PARAM_2		49

#define CAN_SET_BOARD_ID			50
#define CAN_GET_BOARD_ID			51

#define CAN_GET_ERROR_STATUS		60

#define CAN_GET_PID_OUTPUT			63
#define CAN_GET_PID_ERROR			55

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
#define CAN_SET_OFFSET_ABS_ENCODER   75
#define CAN_GET_OFFSET_ABS_ENCODER   76
#define NUM_OF_MESSAGES				77

// class 1 messages, broadcast 
// when in bcast mode, messages are sent periodically by the controller

#define CAN_BCAST_NONE				0
#define CAN_BCAST_POSITION			1
#define CAN_BCAST_PID_VAL			2
#define CAN_BCAST_STATUS			3
#define CAN_BCAST_CURRENT			4
#define CAN_BCAST_OVERFLOW			5
#define CAN_BCAST_PRINT				6
#define CAN_BCAST_VELOCITY			7

#define CAN_BCAST_MAX_MSGS			8



#define CAN_ID_COUPLED_BOARD        1
#if VERSION== 0x113

#warning The CAN_ID_COUPLED_BOARD must be set 


#endif

/******************************************************/
// macro
/******************************************************/
#define ASK_PARM(msg, var1) \
	AS1_printStringEx (msg); \
	AS1_printStringEx (" ["); \
	AS1_printWord16AsChars (*var1); \
	AS1_printStringEx ("] : "); \
	AS1_getStringEx (buffer, SMALL_BUFFER_SIZE, true); \
	iretval = AS1_atoi (buffer, AS1_strlen(buffer, SMALL_BUFFER_SIZE)); \
	*var1 = iretval;
	
//----------------------------------------
#define ASK_PARM_32(msg, var1) \
	AS1_printStringEx (msg); \
	AS1_printStringEx (" ["); \
	AS1_printDWordAsChars (*var1); \
	AS1_printStringEx ("] : "); \
	AS1_getStringEx (buffer, SMALL_BUFFER_SIZE, true); \
	iretval = AS1_atoi (buffer, AS1_strlen(buffer, SMALL_BUFFER_SIZE)); \
	*var1 = (dword)iretval;

//----------------------------------------
#define BEGIN_SPECIAL_MSG_TABLE(x) \
	switch (x & 0x7F) \
	{ \
		default: \
			break;

//----------------------------------------
#define END_SPECIAL_MSG_TABLE \
	}

//----------------------------------------	
// message table macros 
// 0x7f = 0111 1111b
#define BEGIN_MSG_TABLE(x) \
	CAN_TEMP16 = (word)extract_l(x); \
	switch (CAN_TEMP16 & 0x7f) \
	{ \
		default: \
			return ERR_OK; \
			break;
			
//----------------------------------------
#define HANDLE_MSG(x, y) \
		case x: \
			y(CAN_DATA) ; \
			break;

//----------------------------------------	
#define END_MSG_TABLE \
	}


#endif 
