#include<p30f4011.h>
#include<can.h>
//#include"Pin Definitions.h"

/* Frame formats */
#define STANDARD_FORMAT            0
#define EXTENDED_FORMAT            1

/* Frame types   */
#define DATA_FRAME                 1
#define REMOTE_FRAME               0

#define CAN_TX_SOFTWARE_BUFFER_SIZE 10
#define CAN_BOARD_ID 1
#define CURRENT_BOARD_TYPE       5   //SKIN

// CAN RX message buffer
#define CAN_RX_SOFTWARE_BUFFER_SIZE 5


// unsigned char can1TxEn;
//#define CAN1_TX_EN  can1TxEn=1;
//#define CAN1_TX_DIS can1TxEn=0;

/* can bus message structure */
typedef struct canmsg_tag 
{
	unsigned char	CAN_data[8];					// CAN bus message 
	unsigned int 	CAN_messID;						// message ID - arbitration 
	unsigned char 	CAN_frameType;					// data or remote frame 
	unsigned char 	CAN_frameFormat;				// standard or extended frame 
	unsigned char 	CAN_length;						// len of the data 
} canmsg_t;

void CAN_Init();
unsigned char CAN1_send(unsigned int MessageID,unsigned char FrameType,unsigned char Length,unsigned char *Data);
void CAN1_interruptTx (void);
void CAN1_interruptRx (void);
int  CAN1_getRxbufferIndex();
void CAN1_RxbufferIndex_dec(); //canRxBufferIndex --
int CAN1_handleRx    (unsigned int board_id);
void can_receive_additional_info(); 
void can_send_additional_info(); 

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

#define CAN_SET_SMOOTH_PID          75

//Tactile Sensor

#define CAN_TACT_SETUP              76
#define CAN_TACT_CALIB              77


#define NUM_OF_MESSAGES				78

// class 1 messages, broadcast 
// when in bcast mode, messages are sent periodically by the controller

#define CAN_BCAST_NONE				0
#define CAN_BCAST_POSITION			1
#define CAN_BCAST_PID_VAL			2
#define CAN_BCAST_STATUS				3
#define CAN_BCAST_CURRENT			4
#define CAN_BCAST_OVERFLOW			5
#define CAN_BCAST_PRINT				6
#define CAN_BCAST_VELOCITY			7

#define CAN_BCAST_MAX_MSGS			8


//canLoader messages

#define CMD_BROADCAST 0xFF
#define CMD_BOARD     0x0
#define CMD_