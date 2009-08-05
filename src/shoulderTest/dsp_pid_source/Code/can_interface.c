#include "dsp56f807.h"
#include "options.h"
#include "pid.h"
#include "trajectory.h"
#include "can_interface.h"
#include "flash_interface.h"
#include "pwm_interface.h"
#include "calibration.h"
#include "encoders_interface.h"
#include "currents_interface.h"
#include "messages.h"
#include "asc.h"
#include "can1.h"

#ifndef VERSION
#	error "No valid version specified"
#endif

//************************************
// CAN bus communication global vars 
//************************************

canmsg_t can_fifo[CAN_FIFO_LEN];
Int16 write_p = 0;
Int16 read_p = -1;					// -1 means empty, last_read == fifo_ptr means full 
canmsg_t _canmsg;					// buffer to prepare messages for send 

byte  _general_board_error = ERROR_NONE;
byte  can_channels = 0; 
dword broadcast_mask = 0;					/* specifies which broadcast messages are to be sent */

extern byte	_board_ID;
extern void bootStart(void);            // Defined in linker.cmd file  

/**********************************************************************
  Converts four bytes in a double word
 **********************************************************************/
dword BYTE_C(byte x4, byte x3, byte x2, byte x1)
{
	dword ret;
	word *p = (word *)&ret;
	*p++ = __shl(x3,8) | x4;
	*p++ = __shl(x1,8) | x2;
	return ret;
}

/***************************************************************************/
/**
 * This method inits the CAN interface
 * @param b_type is the board type, can be one of the following: 
 * BOARD_TYPE_2DC
 * BOARD_TYPE_2BLL
 * BOARD_TYPE_4DC
 ***************************************************************************/
void can_interface_init (byte channels_num)
{
	can_channels = channels_num;
	CAN1_init ();
	set_can_masks();
}

/***************************************************************************/
/**
 * This method extracts from the can message the information about the target
 * axis of the board.
 * @return the axis number: 
 * 0-1 for 2DC/2BLL boards,
 * 0-3 for 4DC boards
 ***************************************************************************/
 inline int get_axis (void) 
 {
 	if (can_channels == 2 )
 	 	return ((_canmsg.CAN_data[0] & 0x80) ? 1 : 0) ;
 	
 	if (can_channels == 4 )
 	{
 		if (_board_ID == (_canmsg.CAN_messID & 0x000F)) 
			return ((_canmsg.CAN_data[0] & 0x80) ? 1 : 0) ;
		else if (_board_ID+1 == (_canmsg.CAN_messID & 0x000F)) 
			return ((_canmsg.CAN_data[0] & 0x80) ? 3 : 2) ; 
 	}
 }
 
/***************************************************************************/
/**
 * This method reads and executes the CAN messages.
/***************************************************************************/
byte can_interface (void)
{
	bool done = false;
	word ikk;
	dword IdTx;
	byte  write_buffer = 0;	

	CAN_DI;
	if (read_p != -1)
	{
		CAN_EI;
		while (!done)
		{
			canmsg_t *p;
			CAN_DI;	
			p = can_fifo + read_p;
			
			/* makes a private copy of the message */
			_canmsg.CAN_data[0] = p->CAN_data[0];
			_canmsg.CAN_data[1] = p->CAN_data[1];
			_canmsg.CAN_data[2] = p->CAN_data[2];
			_canmsg.CAN_data[3] = p->CAN_data[3];
			_canmsg.CAN_data[4] = p->CAN_data[4];
			_canmsg.CAN_data[5] = p->CAN_data[5];
			_canmsg.CAN_data[6] = p->CAN_data[6];
			_canmsg.CAN_data[7] = p->CAN_data[7];
			_canmsg.CAN_messID = p->CAN_messID;
			_canmsg.CAN_frameType = p->CAN_frameType;
			_canmsg.CAN_frameFormat = p->CAN_frameFormat;
			_canmsg.CAN_length = p->CAN_length;
			
			if (read_p == write_p)
			{
				read_p = -1;	/* empty */
				done = true;
			}
			else
			{
				read_p ++;
				if (read_p >= CAN_FIFO_LEN)
					read_p = 0;
				//done = true;
			}
			CAN_EI;
			
#ifdef DEBUG_CAN_MSG
		if (_verbose)
		{
			AS1_printStringEx ("id: ");
			AS1_printDWordAsChars (_canmsg.CAN_messID);
			AS1_printStringEx (" ");
			print_can (_canmsg.CAN_data, _canmsg.CAN_length, 'i');
			CAN1_getError (&err);
			CAN1_print_error (&err);
		}
#endif

			if (write_buffer == 0)
				write_buffer = 2;
			else
				write_buffer = 0;

#define CAN_BUFFER write_buffer
#define CAN_DATA _canmsg.CAN_data
#define CAN_FRAME_TYPE _canmsg.CAN_frameType
#define CAN_FRAME_FMT _canmsg.CAN_frameFormat
#define CAN_LEN _canmsg.CAN_length
#define CAN_ID _canmsg.CAN_messID
#define CAN_TEMP16 ikk


			// special message, not too neat
			// ID 0x0100 (001 0000 0000b) message class = periodic message  
			
#if VERSION == 0x0113

			if ((_canmsg.CAN_messID & 0x00000700) == 0x0100)
			{
				// for messages of class 001, the meaning of data/ID is:
				// AAA BBBB CCCC, with AAA=Message class, BBBB Source address, CCCC message command
				
				// Switch on the message command: 000 0000 1111b
				switch (_canmsg.CAN_messID & 0x000f)
				{
					case CAN_BCAST_POSITION:
						CAN_SET_ACTIVE_ENCODER_POSITION_HANDLER(0)
						break;
					
					default:
						break;
				}
			}
			else
#else
			if ((_canmsg.CAN_messID & 0x00000700) == 0x0100)
			{
			}
			else
#endif
			// special message for the can loader 
			// ID 0x0700 (111 0000 0000b) message class = broadcast message 
			if ((_canmsg.CAN_messID & 0x00000700) == 0x0700)
			{
				if ((_canmsg.CAN_length == 1)) 
				{
	 				IdTx = (_canmsg.CAN_messID & 0x0700);			
					IdTx |= (L_deposit_l (_board_ID) << 4);
					IdTx |= ((_canmsg.CAN_messID >> 4) & 0x000F);
					
			    	switch (_canmsg.CAN_data[0]) 
			    	{
			    		//request for board type and firmware versione
						case 0xFF:
							_canmsg.CAN_data[0] = 0xFF;
							_canmsg.CAN_data[1] = 0;  // board type (always 0 for motor control card).
							_canmsg.CAN_data[2] = (_version & 0xff00) >> 8;  // firmware version.	
							_canmsg.CAN_data[3] = _version & 0x00ff; 		 // firmware revision.
							CAN1_sendFrame (1, IdTx, DATA_FRAME, 4, _canmsg.CAN_data);
							break;
						
						// ping command	
					    case 4:
		    				if (_board_ID == (_canmsg.CAN_messID & 0x000F)) 
		    				{
								_canmsg.CAN_data[0] = 4;
								_canmsg.CAN_data[1] = 1; 
								CAN1_sendFrame (1, IdTx, DATA_FRAME, 2, _canmsg.CAN_data);
							}	
							break;
						
						// restart bootloader	
					    case 0:
		    				if (_board_ID == (_canmsg.CAN_messID & 0x000F))
		    					asm(jsr bootStart);	/// check whether this has to be a JMP rather than a JSR.
		    				break;
		    		}	
				}
			}
			
			// polling message 
			// ID 0x0000 (000 xxxx xxxxb) message class = polling message 
			else
			{
			int axis = get_axis();
			
			BEGIN_MSG_TABLE (_canmsg.CAN_data[0])
			HANDLE_MSG (CAN_NO_MESSAGE, CAN_NO_MESSAGE_HANDLER)
			HANDLE_MSG (CAN_CONTROLLER_RUN, CAN_CONTROLLER_RUN_HANDLER)
			HANDLE_MSG (CAN_CONTROLLER_IDLE, CAN_CONTROLLER_IDLE_HANDLER)
			HANDLE_MSG (CAN_TOGGLE_VERBOSE, CAN_TOGGLE_VERBOSE_HANDLER)
			HANDLE_MSG (CAN_CALIBRATE_ENCODER, CAN_CALIBRATE_ENCODER_HANDLER)
			HANDLE_MSG (CAN_ENABLE_PWM_PAD, CAN_ENABLE_PWM_PAD_HANDLER)
			HANDLE_MSG (CAN_DISABLE_PWM_PAD, CAN_DISABLE_PWM_PAD_HANDLER)
			HANDLE_MSG (CAN_GET_CONTROL_MODE, CAN_GET_CONTROL_MODE_HANDLER)
			HANDLE_MSG (CAN_MOTION_DONE, CAN_MOTION_DONE_HANDLER)
			
			HANDLE_MSG (CAN_WRITE_FLASH_MEM, CAN_WRITE_FLASH_MEM_HANDLER)
			HANDLE_MSG (CAN_READ_FLASH_MEM, CAN_READ_FLASH_MEM_HANDLER)
			
			HANDLE_MSG (CAN_GET_ENCODER_POSITION, CAN_GET_ENCODER_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_DESIRED_POSITION, CAN_SET_DESIRED_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_DESIRED_POSITION, CAN_GET_DESIRED_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_DESIRED_VELOCITY, CAN_SET_DESIRED_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_GET_DESIRED_VELOCITY, CAN_GET_DESIRED_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_SET_DESIRED_ACCELER, CAN_SET_DESIRED_ACCELER_HANDLER)
			HANDLE_MSG (CAN_GET_DESIRED_ACCELER, CAN_GET_DESIRED_ACCELER_HANDLER)

			HANDLE_MSG (CAN_SET_ENCODER_POSITION, CAN_SET_ENCODER_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_ENCODER_VELOCITY, CAN_GET_ENCODER_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_SET_COMMAND_POSITION, CAN_SET_COMMAND_POSITION_HANDLER)
			
			HANDLE_MSG (CAN_SET_P_GAIN, CAN_SET_P_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_P_GAIN, CAN_GET_P_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_D_GAIN, CAN_SET_D_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_D_GAIN, CAN_GET_D_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_I_GAIN, CAN_SET_I_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_I_GAIN, CAN_GET_I_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_ILIM_GAIN, CAN_SET_ILIM_GAIN_HANDLER)
			HANDLE_MSG (CAN_GET_ILIM_GAIN, CAN_GET_ILIM_GAIN_HANDLER)
			HANDLE_MSG (CAN_SET_OFFSET, CAN_SET_OFFSET_HANDLER)
			HANDLE_MSG (CAN_GET_OFFSET, CAN_GET_OFFSET_HANDLER)
			HANDLE_MSG (CAN_SET_SCALE, CAN_SET_SCALE_HANDLER)
			HANDLE_MSG (CAN_GET_SCALE, CAN_GET_SCALE_HANDLER)

			HANDLE_MSG (CAN_POSITION_MOVE, CAN_POSITION_MOVE_HANDLER)
			HANDLE_MSG (CAN_VELOCITY_MOVE, CAN_VELOCITY_MOVE_HANDLER)
		
			HANDLE_MSG (CAN_GET_PID_OUTPUT, CAN_GET_PID_OUTPUT_HANDLER)
			HANDLE_MSG (CAN_GET_PID_ERROR, CAN_GET_PID_ERROR_HANDLER)
			
			HANDLE_MSG (CAN_SET_MIN_POSITION, CAN_SET_MIN_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_MIN_POSITION, CAN_GET_MIN_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_MAX_POSITION, CAN_SET_MAX_POSITION_HANDLER)
			HANDLE_MSG (CAN_GET_MAX_POSITION, CAN_GET_MAX_POSITION_HANDLER)
			HANDLE_MSG (CAN_SET_MAX_VELOCITY, CAN_SET_MAX_VELOCITY_HANDLER)
			HANDLE_MSG (CAN_GET_MAX_VELOCITY, CAN_GET_MAX_VELOCITY_HANDLER)
		
			HANDLE_MSG (CAN_SET_TLIM, CAN_SET_TLIM_HANDLER)
			HANDLE_MSG (CAN_GET_TLIM, CAN_GET_TLIM_HANDLER)
			HANDLE_MSG (CAN_SET_CURRENT_LIMIT, CAN_SET_CURRENT_LIMIT_HANDLER)
			HANDLE_MSG (CAN_SET_BCAST_POLICY, CAN_SET_BCAST_POLICY_HANDLER)
			HANDLE_MSG (CAN_GET_ERROR_STATUS, CAN_GET_ERROR_STATUS_HANDLER)

//			HANDLE_MSG (CAN_GET_ACTIVE_ENCODER_POSITION, CAN_GET_ACTIVE_ENCODER_POSITION_HANDLER)
			
			END_MSG_TABLE		
			}

#ifdef DEBUG_CAN_MSG
			if (_verbose)
			{
				AS1_printStringEx ("id: ");
				AS1_printDWordAsChars (_canmsg.CAN_messID);
				AS1_printStringEx (" ");
				print_can (_canmsg.CAN_data, _canmsg.CAN_length, 'o'); 
			}
#endif
	
		} /* end of while() */
		
	} /* end of if () */
	else
	{
		CAN_EI;
	}
			
	return ERR_OK;
}


/*********************************************************** 
 * send broadcast messages according to mask 
 * bit 1: position
 * bit 2: velocity + acceleration (not yet implemented)
 * bit 3: fault bits
 * bit 4: e.g. PWMA_PMFSA register
 * bit 5: current feedback + position error
 *
 ***********************************************************/
void can_send_broadcast(void)
{
	int iretval; 
	bool send;
	
	if (!broadcast_mask)
		return;
	
	if ((broadcast_mask & 0x02) && _counter == 0)
	{
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_POSITION;

		_canmsg.CAN_data[0] = BYTE_4(_position[0]);
		_canmsg.CAN_data[1] = BYTE_3(_position[0]);
		_canmsg.CAN_data[2] = BYTE_2(_position[0]);
		_canmsg.CAN_data[3] = BYTE_1(_position[0]);
		_canmsg.CAN_data[4] = BYTE_4(_position[1]);
		_canmsg.CAN_data[5] = BYTE_3(_position[1]);
		_canmsg.CAN_data[6] = BYTE_2(_position[1]);
		_canmsg.CAN_data[7] = BYTE_1(_position[1]);	
			
		_canmsg.CAN_length = 8;
		_canmsg.CAN_frameType = DATA_FRAME;
		if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
		/*
		// send trajectory info		
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_TRAJECTORY;

		_canmsg.CAN_data[0] = BYTE_4(_desired[0]);
		_canmsg.CAN_data[1] = BYTE_3(_desired[0]);
		_canmsg.CAN_data[2] = BYTE_2(_desired[0]);
		_canmsg.CAN_data[3] = BYTE_1(_desired[0]);
		_canmsg.CAN_data[4] = BYTE_4(_desired[1]);
		_canmsg.CAN_data[5] = BYTE_3(_desired[1]);
		_canmsg.CAN_data[6] = BYTE_2(_desired[1]);
		_canmsg.CAN_data[7] = BYTE_1(_desired[1]);	
			
		_canmsg.CAN_length = 8;
		_canmsg.CAN_frameType = DATA_FRAME;
		if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");*/		
		
		
	//	if (can_channels==4)
			{
			_canmsg.CAN_messID = 0x100;
			_canmsg.CAN_messID |= (_board_ID+1) << 4;
			_canmsg.CAN_messID |= CAN_BCAST_POSITION;

			_canmsg.CAN_data[0] = BYTE_4(_position[2]);
			_canmsg.CAN_data[1] = BYTE_3(_position[2]);
			_canmsg.CAN_data[2] = BYTE_2(_position[2]);
			_canmsg.CAN_data[3] = BYTE_1(_position[2]);
			_canmsg.CAN_data[4] = BYTE_4(_position[3]);
			_canmsg.CAN_data[5] = BYTE_3(_position[3]);
			_canmsg.CAN_data[6] = BYTE_2(_position[3]);
			_canmsg.CAN_data[7] = BYTE_1(_position[3]);
				
			_canmsg.CAN_length = 8;
			_canmsg.CAN_frameType = DATA_FRAME;
			if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");			
			}
			/*
			// send trajectory info		
			_canmsg.CAN_messID = 0x100;
			_canmsg.CAN_messID |= (_board_ID+1) << 4;
			_canmsg.CAN_messID |= CAN_BCAST_TRAJECTORY;

			_canmsg.CAN_data[0] = BYTE_4(_desired[2]);
			_canmsg.CAN_data[1] = BYTE_3(_desired[2]);
			_canmsg.CAN_data[2] = BYTE_2(_desired[2]);
			_canmsg.CAN_data[3] = BYTE_1(_desired[2]);
			_canmsg.CAN_data[4] = BYTE_4(_desired[3]);
			_canmsg.CAN_data[5] = BYTE_3(_desired[3]);
			_canmsg.CAN_data[6] = BYTE_2(_desired[3]);
			_canmsg.CAN_data[7] = BYTE_1(_desired[3]);	
				
			_canmsg.CAN_length = 8;
			_canmsg.CAN_frameType = DATA_FRAME;
			if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");*/
	}

	if ((broadcast_mask & 0x04) && _counter == 1)
	{
		/* CHANGED: send PID (control) value */
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_PID_VAL;

		_canmsg.CAN_data[0] = BYTE_H(_pid[0]);
		_canmsg.CAN_data[1] = BYTE_L(_pid[0]);
		_canmsg.CAN_data[2] = BYTE_H(_pid[1]);
		_canmsg.CAN_data[3] = BYTE_L(_pid[1]);
		
		_canmsg.CAN_length = 4;
		_canmsg.CAN_frameType = DATA_FRAME;
		if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
		
		if (can_channels==4)
			{	
			_canmsg.CAN_messID = 0x100;
			_canmsg.CAN_messID |= (_board_ID+1) << 4;
			_canmsg.CAN_messID |= CAN_BCAST_PID_VAL;

			_canmsg.CAN_data[0] = BYTE_H(_pid[2]);
			_canmsg.CAN_data[1] = BYTE_L(_pid[2]);
			_canmsg.CAN_data[2] = BYTE_H(_pid[3]);
			_canmsg.CAN_data[3] = BYTE_L(_pid[3]);
			
			_canmsg.CAN_length = 4;
			_canmsg.CAN_frameType = DATA_FRAME;
			if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");	
			}
	}
	
	if ((broadcast_mask & 0x08) && _counter == 2)
	{
		/* if a new fault is detected then sends a message */
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_FAULT;
		_canmsg.CAN_data[0] = 0;
		_canmsg.CAN_data[1] = 0;
		_canmsg.CAN_data[2] = 0;
		_canmsg.CAN_data[3] = 0;
		send = false;
			
		iretval = getReg (PWMA_PMFSA);
		
		if (_fault[0] == 0 && iretval != 0)
		{
			// new fault, copy iretval in _fault.
			_fault[0] = iretval;
			_canmsg.CAN_data[0] = BYTE_H(iretval);
			_canmsg.CAN_data[1] = BYTE_L(iretval);
			send = true;
		}
		else
		if (_fault[0] != 0 && iretval == 0)
		{
			// reset fault.
			_fault[0] = 0;
			_canmsg.CAN_data[0] = BYTE_H(iretval);
			_canmsg.CAN_data[1] = BYTE_L(iretval);
			send = true;
		}
		
		iretval = getReg (PWMB_PMFSA);
		
		if (_fault[1] == 0 && iretval != 0)
		{
			// new fault, copy iretval in _fault.
			_fault[1] = iretval;
			_canmsg.CAN_data[2] = BYTE_H(iretval);
			_canmsg.CAN_data[3] = BYTE_L(iretval);
			send = true;
		}
		else
		if (_fault[1] != 0 && iretval == 0)
		{
			// reset fault.
			_fault[1] = 0;
			_canmsg.CAN_data[2] = BYTE_H(iretval);
			_canmsg.CAN_data[3] = BYTE_L(iretval);
			send = true;
		}

		// if new fault, send message.		
		if (send)
		{
			_canmsg.CAN_length = 4;
			_canmsg.CAN_frameType = DATA_FRAME;
			if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");
		}
	}
	
	if ((broadcast_mask & 0x10) && _counter == 3)
	{
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_CURRENT;

		_canmsg.CAN_data[0] = BYTE_H(_current[0]);
		_canmsg.CAN_data[1] = BYTE_L(_current[0]);
		_canmsg.CAN_data[2] = BYTE_H(_current[1]);
		_canmsg.CAN_data[3] = BYTE_L(_current[1]);
		
		_canmsg.CAN_data[4] = BYTE_H(_error[0]);
		_canmsg.CAN_data[5] = BYTE_L(_error[0]);
		_canmsg.CAN_data[6] = BYTE_H(_error[1]);
		_canmsg.CAN_data[7] = BYTE_L(_error[1]);
			
		_canmsg.CAN_length = 8;
		_canmsg.CAN_frameType = DATA_FRAME;
		if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
	
		if (can_channels==4)
			{
			_canmsg.CAN_messID = 0x100;
			_canmsg.CAN_messID |= (_board_ID+1) << 4;
			_canmsg.CAN_messID |= CAN_BCAST_CURRENT;

			_canmsg.CAN_data[0] = BYTE_H(_current[2]);
			_canmsg.CAN_data[1] = BYTE_L(_current[2]);
			_canmsg.CAN_data[2] = BYTE_H(_current[3]);
			_canmsg.CAN_data[3] = BYTE_L(_current[3]);
			
			_canmsg.CAN_data[4] = BYTE_H(_error[2]);
			_canmsg.CAN_data[5] = BYTE_L(_error[2]);
			_canmsg.CAN_data[6] = BYTE_H(_error[3]);
			_canmsg.CAN_data[7] = BYTE_L(_error[3]);
				
			_canmsg.CAN_length = 8;
			_canmsg.CAN_frameType = DATA_FRAME;
			if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");
			}
	}
}

/**************************************************************
Set Can Mask
**************************************************************/
void set_can_masks()
{
	UInt32 filter1=0;
	UInt32 filter2=0;
	UInt32 mask1=0;
	UInt32 mask2=0;
	
	if (can_channels==2)
	{
		create_F_M(&filter1, &mask1,0b000,0xFF, _board_ID,   0b111, 0xFF, _board_ID);
		create_F_M(&filter2, &mask2,0b000,0xFF, _board_ID,   0b111, 0x0,  0b1111);
	}
	else if (can_channels==4)
	{
		create_F_M(&filter1, &mask1,0b000,0xFF, _board_ID,   0b111, 0xFF, _board_ID);
		create_F_M(&filter2, &mask2,0b000,0xFF, _board_ID+1, 0b111, 0x0,  0b1111);	
	}
	
	setmask(filter1,filter2, mask1, mask2);
}

/**************************************************************
Print CAN
**************************************************************/
void print_can (byte data[], byte len, char c)
{
	int i;
	AS1_printString (&c, 1);
	AS1_printStringEx (": ");
	
	for (i = 0; i < len; i++)
	{
		AS1_printByteAsChars (data[i]);
		AS1_printStringEx (" ");
	}
	AS1_printStringEx ("\r\n");
}

/*************************************************************************************/
/**
 * This method creates two 16 bits filters and two 16 bits masks for the CAN module
 * See CAN protocol.
 *
 * @param _class1 the first 3 bits of the CAN ID.  0xFF for don't care
 * @param _source1 source address in CAN ID.	   0xFF for don't care	
 * @param _dest1 destination address in CAN ID.    0xFF for don't care 
 * @param _class2 the first 3 bits of the CAN ID.  0xFF for don't care
 * @param _source2 source address in CAN ID.       0xFF for don't care
 * @param _dest2 destination address in CAN ID.    0xFF for don't care
 * @return set filter 32 bits filter, and mask 32 bits mask;
 *************************************************************************************/
void create_F_M(UInt32 *filter, UInt32 *mask, byte class1, byte source1, byte dest1, byte class2, byte source2, byte dest2)
{
	UInt16 mask1=0x1F; //the first 5 bits don't care in any case
	UInt16 mask2=0x1F; //the first 5 bits don't care in any case
	UInt16 filter1=0;
	UInt16 filter2=0;
	UInt16 temp=0;
	*filter=0;
	*mask=0;
	if (class1 == 0xFF) mask1 |=0xE000;
	class1 == 0xFF ? mask1 |=0xE000 : 0;
	source1 == 0xFF ? mask1 |=0x1E00 : 0;
	dest1 == 0xFF ? mask1 |=0x01E0 : 0; 
	
	class2 == 0xFF ? mask2 |=0xE000 : 0;
	source2 == 0xFF ? mask2 |=0x1E00 : 0;
	dest2 == 0xFF ? mask2 |=0x01E0 : 0;
	
	// devo mettere byte rovesciati;
	temp=mask1;
	mask1=(mask1<<8 &0xFF00) | (temp>>8 &0x00FF);
	temp=mask2;
	mask2=(mask2<<8 &0xFF00) | (temp>>8 &0x00FF);
	
	filter1  =(dest1 & 0xF)<<5;
	filter1 |=(source1 & 0xF)<<9;
	filter1 |=(class1 & 0x7)<<13;
	temp=filter1;
	filter1=(filter1<<8 &0xFF00) | (temp>>8 &0x00FF);
	
	filter2  =(dest2 & 0xF)<<5;
	filter2 |=(source2 & 0xF)<<9;
	filter2 |=(class2 & 0x7)<<13;
	temp=filter2;
	filter2=(filter2<<8 &0xFF00) | (temp>>8 &0x00FF);
	
	
	*filter=L_deposit_h(filter1) | filter2; //create filter and mask
	*mask=L_deposit_h(mask1) | mask2; 
}

/***********************************************************************/
/**
 * This method set the 4 16 bits filters and masks for the CAN module
 * See CAN protocol.
 *
 * @param filter1
 * @param filter2
 * @param mask1
 * @param mask2 
 ***********************************************************************/
void setmask(UInt32 filter1, UInt32 filter2, UInt32 mask1,UInt32 mask2)
{
	CAN1_setAcceptanceMask (mask1,mask2);
	CAN1_setAcceptanceCode (filter1,filter2);	
}