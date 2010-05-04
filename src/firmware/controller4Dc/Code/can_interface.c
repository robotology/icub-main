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

#if VERSION == 0x0125
	#include "strain_board.h"
#endif

//************************************
// CAN bus communication global vars 
//************************************

byte ERR_STATUS=0;

extern canmsg_t can_fifo[];
extern Int16 write_p;
extern Int16 read_p;
extern bool mainLoopOVF[2];
extern int 	_countBoardStatus[2];
extern UInt8   highcurrent[4];
extern UInt8 mais_counter;
canmsg_t _canmsg;					// buffer to prepare messages for send 

byte  _general_board_error = ERROR_NONE;
byte  can_channels = 0; 
dword broadcast_mask[JN]=INIT_ARRAY (0);					/* specifies which broadcast messages are to be sent */

extern byte	_board_ID;
extern void bootStart(void);            // Defined in linker.cmd file  
extern char    _additional_info [32];
extern word _build_number;


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
 * @param b_type
 ***************************************************************************/
void can_interface_init (byte channels_num)
{
	can_channels = channels_num;
	CAN1_init (_board_ID);
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
	byte i=0;
	byte j=0;
	byte T_init=0;
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
			_canmsg.CAN_ID_class=(_canmsg.CAN_messID>>8 & 0x7);
			_canmsg.CAN_ID_src=(_canmsg.CAN_messID>>4 & 0xf);
			_canmsg.CAN_ID_dst=(_canmsg.CAN_messID  & 0xf);
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
		//	CAN1_getError (&err);
		//	CAN1_print_error (&err);
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

			if (_canmsg.CAN_ID_class == CLASS_PERIODIC_DSP)
			{
				// for messages of class 001, the meaning of data/ID is:
				// AAA BBBB CCCC, with AAA=Message class, BBBB Source address, CCCC message command
				
				// Switch on the message command: 000 0000 1111b
				switch (_canmsg.CAN_ID_dst)
				{
					case CAN_BCAST_POSITION:
						CAN_SET_ACTIVE_ENCODER_POSITION_HANDLER(0)
						break;
					
					default:
						break;
				}
			}
			
#elif VERSION == 0x0125
			
			if (_canmsg.CAN_ID_class == CLASS_PERIODIC_SENS)
			{
				switch(_canmsg.CAN_ID_dst)
				{
				
					case 0xa:
					_strain[0] = BYTE_W(_canmsg.CAN_data[0],_canmsg.CAN_data[1])-_strain_init[0];
					_strain[1] = BYTE_W(_canmsg.CAN_data[2],_canmsg.CAN_data[3])-_strain_init[1];
					_strain[2] = BYTE_W(_canmsg.CAN_data[4],_canmsg.CAN_data[5])-_strain_init[2];
					break;
					
					case 0xb:
					_strain[3] = BYTE_W(_canmsg.CAN_data[0],_canmsg.CAN_data[1])-_strain_init[3];
					_strain[4] = BYTE_W(_canmsg.CAN_data[2],_canmsg.CAN_data[3])-_strain_init[4];
					_strain[5] = BYTE_W(_canmsg.CAN_data[4],_canmsg.CAN_data[5])-_strain_init[5];
					break;
				}
				if(_strain_init[5]==0 ||_strain_init[4]==0)
				{
					_Feq=0;
					for(T_init=0;i<6;i++)
					_strain_init[i]=_strain[i];
				}
			}
	
#elif (VERSION == 0x0121 || VERSION == 0x0128 || VERSION == 0x0130)

			if (_canmsg.CAN_ID_class == CLASS_PERIODIC_SENS)
			{
			
				if ((_canmsg.CAN_messID==MAIS_8bit_D_MSG) || (_canmsg.CAN_messID==MAIS_8bit_C_MSG))
				{	
					mais_counter=0;
					CAN_SET_ACTIVE_ENCODER_POSITION_HANDLER(0)	
				}
			}
		
#else
			if (_canmsg.CAN_ID_class == CLASS_PERIODIC_DSP)
			{
			}

			
#endif
			// special message for the can loader 
			// ID 0x0700 (111 0000 0000b) message class = broadcast message 
			if (_canmsg.CAN_ID_class == CLASS_CANLOADER)
			{
				// if canloader message is for me or is for all...
				if (_canmsg.CAN_ID_dst == _board_ID ||
					_canmsg.CAN_ID_dst == 0x0f)
					{					
						IdTx = (_canmsg.CAN_messID & 0x0700);			
						IdTx |= (L_deposit_l (_board_ID) << 4);
						IdTx |= ((_canmsg.CAN_messID >> 4) & 0x000F);
						
				    	switch (_canmsg.CAN_data[0]) 
				    	{
				    		//request for board type and firmware versione
							case 0xFF:
								_canmsg.CAN_data[0] = 0xFF;
								_canmsg.CAN_data[1] = CURRENT_BOARD_TYPE;  
								_canmsg.CAN_data[2] = (_version & 0xff00) >> 8;  // firmware version.	
								_canmsg.CAN_data[3] = _version & 0x00ff; 		 // firmware revision
								_canmsg.CAN_data[4] = _build_number & 0x00ff;    // build number
								CAN1_send(IdTx, DATA_FRAME, 5, _canmsg.CAN_data);

								
								// disable PWM
								for (i=0; i<JN; i++)
								{
									PWM_outputPadDisable(i); 								
									_pad_enabled[i] = false; 
									_general_board_error = ERROR_NONE; 								
								}
								can_printf("CANLOADER DIS");
								break;							
							
							// ping command	
						    case 4:
			    				if (_board_ID == _canmsg.CAN_ID_dst) 
			    				{
									_canmsg.CAN_data[0] = 4;
									_canmsg.CAN_data[1] = 1; 
									CAN1_send(IdTx, DATA_FRAME, 2, _canmsg.CAN_data);
								}	
								break;
							
							// restart bootloader	
						    case 0:
			    				if (_board_ID == _canmsg.CAN_ID_dst)
			    					asm(jsr bootStart);	/// check whether this has to be a JMP rather than a JSR.
			    				break;
			    			
			    			// set additional info					
				    		case CAN_SET_ADDITIONAL_INFO:
				    			if (_board_ID == _canmsg.CAN_ID_dst) 
			    				{
					    			can_receive_additional_info(); 
									writeToFlash (_flash_addr); 
			    				}
			    			break;
			    			
			    			// get additional info	
				    		case CAN_GET_ADDITIONAL_INFO:
				    			if (_board_ID == _canmsg.CAN_ID_dst) 
			    				{
					    			CAN_ID >>= 4; 
									CAN_ID &= 0xffffff0f; 
									CAN_ID |= (_board_ID << 4); 
					    			CAN_ID |= 0x700;
					    			can_send_additional_info(); 
			    				}
				    		break;
			    		}	
					}
			}
			
			// polling message 
			// ID 0x0000 (000 xxxx xxxxb) message class = polling message 
			if (_canmsg.CAN_ID_class == CLASS_POLLING_DSP)
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
				HANDLE_MSG (CAN_GET_ADDITIONAL_INFO, CAN_GET_ADDITIONAL_INFO_HANDLER)
				HANDLE_MSG (CAN_SET_ADDITIONAL_INFO, CAN_SET_ADDITIONAL_INFO_HANDLER)
				
				HANDLE_MSG (CAN_GET_ENCODER_POSITION, CAN_GET_ENCODER_POSITION_HANDLER)
				HANDLE_MSG (CAN_SET_DESIRED_POSITION, CAN_SET_DESIRED_POSITION_HANDLER)
				HANDLE_MSG (CAN_GET_DESIRED_POSITION, CAN_GET_DESIRED_POSITION_HANDLER)
				HANDLE_MSG (CAN_SET_DESIRED_VELOCITY, CAN_SET_DESIRED_VELOCITY_HANDLER)
				HANDLE_MSG (CAN_GET_DESIRED_VELOCITY, CAN_GET_DESIRED_VELOCITY_HANDLER)
				HANDLE_MSG (CAN_SET_DESIRED_ACCELER, CAN_SET_DESIRED_ACCELER_HANDLER)
				HANDLE_MSG (CAN_GET_DESIRED_ACCELER, CAN_GET_DESIRED_ACCELER_HANDLER)
				HANDLE_MSG (CAN_SET_DESIRED_TORQUE, CAN_SET_DESIRED_TORQUE_HANDLER)
				HANDLE_MSG (CAN_GET_DESIRED_TORQUE, CAN_GET_DESIRED_TORQUE_HANDLER)
				HANDLE_MSG (CAN_SET_DEBUG_PARAM_1, CAN_SET_DEBUG_PARAM_1_HANDLER)
				HANDLE_MSG (CAN_GET_DEBUG_PARAM_1, CAN_GET_DEBUG_PARAM_1_HANDLER)
				HANDLE_MSG (CAN_SET_DEBUG_PARAM_2, CAN_SET_DEBUG_PARAM_2_HANDLER)
				HANDLE_MSG (CAN_GET_DEBUG_PARAM_2, CAN_GET_DEBUG_PARAM_2_HANDLER)
			
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
				
				HANDLE_MSG (CAN_SET_BOARD_ID, CAN_SET_BOARD_ID_HANDLER)
				HANDLE_MSG (CAN_GET_BOARD_ID, CAN_GET_BOARD_ID_HANDLER)
				
				HANDLE_MSG (CAN_SET_CURRENT_LIMIT, CAN_SET_CURRENT_LIMIT_HANDLER)
				HANDLE_MSG (CAN_SET_BCAST_POLICY, CAN_SET_BCAST_POLICY_HANDLER)
				HANDLE_MSG (CAN_GET_ERROR_STATUS, CAN_GET_ERROR_STATUS_HANDLER)
				HANDLE_MSG (CAN_SET_VEL_SHIFT, CAN_SET_VEL_SHIFT_HANDLER)
				
				HANDLE_MSG (CAN_SET_SMOOTH_PID, CAN_SET_SMOOTH_PID_HANDLER)
				
				HANDLE_MSG (CAN_SET_TORQUE_PID, CAN_SET_TORQUE_PID_HANDLER)
				HANDLE_MSG (CAN_GET_TORQUE_PID, CAN_GET_TORQUE_PID_HANDLER)
				HANDLE_MSG (CAN_SET_TORQUE_PIDLIMITS, CAN_SET_TORQUE_PIDLIMITS_HANDLER)
				HANDLE_MSG (CAN_GET_TORQUE_PIDLIMITS, CAN_GET_TORQUE_PIDLIMITS_HANDLER)
				HANDLE_MSG (CAN_SET_POS_PID, CAN_SET_POS_PID_HANDLER)
				HANDLE_MSG (CAN_GET_POS_PID, CAN_GET_POS_PID_HANDLER)
				HANDLE_MSG (CAN_SET_POS_PIDLIMITS, CAN_SET_POS_PIDLIMITS_HANDLER)
				HANDLE_MSG (CAN_GET_POS_PIDLIMITS, CAN_GET_POS_PIDLIMITS_HANDLER)
								
		//		HANDLE_MSG (CAN_GET_ACTIVE_ENCODER_POSITION, CAN_GET_ACTIVE_ENCODER_POSITION_HANDLER)
				
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
 * bit 2: pid
 * bit 3: fault bits
 * bit 4: current feedback and position error
 * bit 5: velocity and acceleration
 *
 ***********************************************************/
void can_send_broadcast(void)
{
	int iretval,k; 
	bool sendA;
	bool sendB;
	bool FAULT_EXT0;
	bool FAULT_EXT1;
	bool FAULT_OVL0;
	bool FAULT_OVL1;
	bool FAULT_OVL2;
	bool FAULT_OVL3;
	bool FAULT_UND;
	
	/*
		//USE THIS TO BROADCAST EVERY 1ms (DEBUG ONLY!)
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
		if (CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
		return;
	*/	
	
				
	_countBoardStatus[0]++;
	_countBoardStatus[1]++;			
				
	if (!broadcast_mask[0])
		return;
	
	if ((broadcast_mask[0] & (1<<(CAN_BCAST_POSITION-1))) && _counter == 0)
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
		if (CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
	}
	if ((broadcast_mask[1] & (1<<(CAN_BCAST_POSITION-1))) && _counter == 0)
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

			if (CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");			
			
	}
	
		
	if ((broadcast_mask[0] & (1<<(CAN_BCAST_PID_VAL-1))) && _counter == 1)
	{
		/* CHANGED: send PID (control) value */
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_PID_VAL;

		_canmsg.CAN_data[0] = BYTE_H(_pid[0]);
		_canmsg.CAN_data[1] = BYTE_L(_pid[0]);
		_canmsg.CAN_data[2] = BYTE_H(_pid[1]);
		_canmsg.CAN_data[3] = BYTE_L(_pid[1]);	

/// @@@@@@@@@@@@@ MEMO TEST
#if VERSION == 0x0125
		_canmsg.CAN_data[0] = BYTE_H(_strain[5]);
		_canmsg.CAN_data[1] = BYTE_L(_strain[5]);
		_canmsg.CAN_data[2] = BYTE_H(_strain[4]);
		_canmsg.CAN_data[3] = BYTE_L(_strain[4]);	
#endif
/// @@@@@@@@@@@@@ MEMO TEST


		_canmsg.CAN_length = 4;
		_canmsg.CAN_frameType = DATA_FRAME;

		if (CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
			AS1_printStringEx("send err\r\n");
	}	
	if ((broadcast_mask[1] & (1<<(CAN_BCAST_PID_VAL-1))) && _counter == 1)
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

			if (CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");	
			
	}
	
	if ((broadcast_mask[0] & (1<<(CAN_BCAST_STATUS-1))) && _counter == 2)
	{
		for (k=0;k<8;k++)
		{
			_canmsg.CAN_data[k] = 0;
		}
		/* if a new fault is detected then sends a message */
		sendA = false;
		
		//  - - - PWM CHANNEL A (axis 0 and 1 ) - - -
		_fault[0] = getReg (PWMA_PMFSA);
		setReg(PWMA_PMFSA,	0x55);
		
		//undervoltage overload external fault axis 0 1 
		_canmsg.CAN_data[0]=((_fault[0]>>8) & 0x01)  | ((_fault[0]>>9) & 0x02)  | ((_fault[0]>>12) & 0x04);						
		_canmsg.CAN_data[2]=((_fault[0]>>8) & 0x01)  | ((_fault[0]>>11) & 0x02) | ((_fault[0]>>12) & 0x04);
		#ifdef DEBUG_CAN_MSG
		if (_canmsg.CAN_data[0] !=0)		
			can_printf("SENDING FAULTS 0%d", (UInt8) _canmsg.CAN_data[0]);
		if (_canmsg.CAN_data[2] !=0)	
			can_printf("SENDING FAULTS 1%d", (UInt8) _canmsg.CAN_data[2]);
		#endif
		
		//  --- HIGH CURRENT CH 0---
		if (highcurrent[0])
		{
			_canmsg.CAN_data[0] |=highcurrent[0]<<3;
			sendA=true;
		}
		//  --- HIGH CURRENT CH 1---
		if (highcurrent[1])
		{
			_canmsg.CAN_data[3] |=highcurrent[1]<<3;
			sendA=true;
		}
		//Control Mode axes 0
		if (_control_mode[0]==0)
		{
			_canmsg.CAN_data[1]= MODE_IDLE;	
		}
		if (_control_mode[0]&0x03)
		{
			_canmsg.CAN_data[1]= MODE_CONTROLLED; 			
		}
		if (_control_mode[0]>0x03)
		{
			_canmsg.CAN_data[1]= MODE_CALIB; 		
		}
		//Control Mode axes 1
		if (_control_mode[1]==0)
		{
			_canmsg.CAN_data[3]= MODE_IDLE;	
		}
		if (_control_mode[1]&0x03)
		{
			_canmsg.CAN_data[3]= MODE_CONTROLLED; 			
		}
		if (_control_mode[1]>0x03)
		{
			_canmsg.CAN_data[3]= MODE_CALIB; 		
		}
		//CANBUS ERRORS
		_canmsg.CAN_data[4] = getCanStatus();
		//MAIN LOOP overflow
		_canmsg.CAN_data[5] |= mainLoopOVF[0];
		
		if ((_canmsg.CAN_data[0] &0x7) ) //the first 3 bits
		{
		#ifdef DEBUG_CAN_MSG	
			can_printf("AXIS 0 ERROR %d", (UInt8) _canmsg.CAN_data[0]);	
		#endif
			sendA = true;
		}
		if ((_canmsg.CAN_data[2] &0x7) ) //the first 3 bits
		{
		#ifdef DEBUG_CAN_MSG	
			can_printf("AXIS 1 ERROR %d", (UInt8) _canmsg.CAN_data[2]);	
		#endif
			sendA = true;
		}
		if (_canmsg.CAN_data[4])
		{
			setCanStatus(0); //reset of the can tx status
		#ifdef DEBUG_CAN_MSG	
			can_printf("CAN_ERR %d", (UInt8)_canmsg.CAN_data[4]);	
		#endif
			sendA=true;	
		} 
		if (_canmsg.CAN_data[5] & 0x7) //the first 3 bits
		{	
		#ifdef DEBUG_CAN_MSG	
			can_printf("BOARD STATUS ERROR %d", (UInt8)_canmsg.CAN_data[5]);	
		#endif	
			if (mainLoopOVF[0]) mainLoopOVF[0]=false;
			sendA=true;
		}
		// TO BE SEND EVERY BStatusTime  ms
		if (_countBoardStatus[0]>=BOARDSTATUSTIME) 		
		{
			_countBoardStatus[0]=0;
			//debug
			_canmsg.CAN_data[6]=getTxError();
	    	_canmsg.CAN_data[7]=getRxError();
			setTxError();
			setRxError();	
			sendA=true;   
		}
	//debug	
	//	_canmsg.CAN_data[6]=_received_pid[0].rec_pid;
	//	_canmsg.CAN_data[7]=_received_pid[1].rec_pid;	    
	//debug	
		if (sendA)
		{
			_canmsg.CAN_messID = 0x100;
			_canmsg.CAN_messID |= (_board_ID) << 4;
			_canmsg.CAN_messID |= CAN_BCAST_STATUS;
		
			_canmsg.CAN_length = 8;
			_canmsg.CAN_frameType = DATA_FRAME;
			CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data);
		}
	}
	if ((broadcast_mask[1] & (1<<(CAN_BCAST_STATUS-1))) && _counter == 2)
	{
		/* if a new fault is detected then sends a message */
		sendB = false;
		for (k=0;k<8;k++)
		{
			_canmsg.CAN_data[k] = 0;
		}
		//  - - - PWM CHANNEL B (axis 2 and 3 ) - - -
		_fault[1] = getReg (PWMB_PMFSA);
		setReg(PWMB_PMFSA,	0x55);
		
		//undervoltage overload external fault axis 2 e 3 
		_canmsg.CAN_data[0]=((_fault[1]>>8) & 0x01)  | ((_fault[1]>>9) & 0x02)  | ((_fault[1]>>12) & 0x04);						
		_canmsg.CAN_data[2]=((_fault[1]>>8) & 0x01)  | ((_fault[1]>>11) & 0x02) | ((_fault[1]>>12) & 0x04);
		#ifdef DEBUG_CAN_MSG
		if (_canmsg.CAN_data[0] !=0)	
			can_printf("SENDING FAULTS 2%d", (UInt8) _canmsg.CAN_data[0]);
		if (_canmsg.CAN_data[2] !=0)
			can_printf("SENDING FAULTS 3%d", (UInt8) _canmsg.CAN_data[2]);
		#endif		

		//  --- HIGH CURRENT CH 2---
		if (highcurrent[2])
		{
			_canmsg.CAN_data[0] |=highcurrent[2]<<3;
			sendB=true;
		}
		//  --- HIGH CURRENT CH 3---
		if (highcurrent[3])
		{
			_canmsg.CAN_data[2] |=highcurrent[3]<<3;
			sendB=true;
		}
		//Control Mode axes 2
		if (_control_mode[2]==0)
		{
			_canmsg.CAN_data[1]= MODE_IDLE;	
		}
		if (_control_mode[2]&0x03)
		{
			_canmsg.CAN_data[1]= MODE_CONTROLLED; 			
		}
		if (_control_mode[2]>0x03)
		{
			_canmsg.CAN_data[1]= MODE_CALIB; 	
		}
				//Control Mode axes 1
		if (_control_mode[3]==0)
		{
			_canmsg.CAN_data[3]= MODE_IDLE;	
		}
		if (_control_mode[3]&0x03)
		{
			_canmsg.CAN_data[3]= MODE_CONTROLLED; 			
		}
		if (_control_mode[3]>0x03)
		{
			_canmsg.CAN_data[3]= MODE_CALIB; 		
		}
		//CANBUS ERRORS
		_canmsg.CAN_data[4] = getCanStatus();
		//MAIN LOOP overflow
		_canmsg.CAN_data[5] |= mainLoopOVF[1];
		
		if ((_canmsg.CAN_data[0] &0x7) ) //the first 3 bits
		{
		#ifdef DEBUG_CAN_MSG	
			can_printf("AXIS 0 ERROR %d", (UInt8) _canmsg.CAN_data[0]);	
		#endif
			sendB = true;
		}
		if ((_canmsg.CAN_data[2] &0x7) ) //the first 3 bits
		{
		#ifdef DEBUG_CAN_MSG	
			can_printf("AXIS 1 ERROR %d", (UInt8) _canmsg.CAN_data[2]);	
		#endif
			sendB = true;
		}
		if (_canmsg.CAN_data[4])
		{
			setCanStatus(0); //reset of the can tx status
		#ifdef DEBUG_CAN_MSG	
			can_printf("CAN_ERR %d", (UInt8)_canmsg.CAN_data[4]);	
		#endif
			sendB =true;	
		} 
		if (_canmsg.CAN_data[5] & 0x7) //the first 3 bits
		{	
		#ifdef DEBUG_CAN_MSG	
			can_printf("BOARD STATUS ERROR %d", (UInt8)_canmsg.CAN_data[5]);	
		#endif	
			if (mainLoopOVF[1]) mainLoopOVF[1]=false;
			sendB =true;
		}
		// TO BE SEND EVERY BStatusTime  ms
		if (_countBoardStatus[1]>=BOARDSTATUSTIME) 		
		{
			_countBoardStatus[1]=0;
		//debug
			_canmsg.CAN_data[6]=getTxError();
	    	_canmsg.CAN_data[7]=getRxError();
			setTxError();
			setRxError();	
			sendB=true;   
		}
	//debug	
	//	_canmsg.CAN_data[6]=_received_pid[0].rec_pid;
	//	_canmsg.CAN_data[7]=_received_pid[1].rec_pid;	    
	//debug	
		
		if (sendB)
		{
			_canmsg.CAN_messID = 0x100;
			_canmsg.CAN_messID |= (_board_ID+1) << 4;
			_canmsg.CAN_messID |= CAN_BCAST_STATUS;
			
			_canmsg.CAN_length = 8;
			_canmsg.CAN_frameType = DATA_FRAME;
			CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data);

		}
	}
	
	if ((broadcast_mask[0] & (1<<(CAN_BCAST_CURRENT-1))) && _counter == 3)
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
	   CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data);
		
	}
	if ((broadcast_mask[1] & (1<<(CAN_BCAST_CURRENT-1))) && _counter == 3)
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

			CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data);
	
	}
	
	if ((broadcast_mask[0] & (1<<(CAN_BCAST_VELOCITY-1))) && _counter == 4)
	{
		_canmsg.CAN_messID = 0x100;
		_canmsg.CAN_messID |= (_board_ID) << 4;
		_canmsg.CAN_messID |= CAN_BCAST_VELOCITY;

		_canmsg.CAN_data[0] = BYTE_H(_speed[0]);
		_canmsg.CAN_data[1] = BYTE_L(_speed[0]);
		_canmsg.CAN_data[2] = BYTE_H(_speed[1]);
		_canmsg.CAN_data[3] = BYTE_L(_speed[1]);
		
		_canmsg.CAN_data[4] = BYTE_H(_accel[0]);
		_canmsg.CAN_data[5] = BYTE_L(_accel[0]);
		_canmsg.CAN_data[6] = BYTE_H(_accel[1]);
		_canmsg.CAN_data[7] = BYTE_L(_accel[1]);
			
		_canmsg.CAN_length = 8;
		_canmsg.CAN_frameType = DATA_FRAME;
		CAN1_send(_canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data);
			
	
	}
	if ((broadcast_mask[1] & (1<<(CAN_BCAST_VELOCITY-1))) && _counter == 4)
	{
			_canmsg.CAN_messID = 0x100;
			_canmsg.CAN_messID |= (_board_ID+1) << 4;
			_canmsg.CAN_messID |= CAN_BCAST_VELOCITY;

			_canmsg.CAN_data[0] = BYTE_H(_speed[2]);
			_canmsg.CAN_data[1] = BYTE_L(_speed[2]);
			_canmsg.CAN_data[2] = BYTE_H(_speed[3]);
			_canmsg.CAN_data[3] = BYTE_L(_speed[3]);
			
			_canmsg.CAN_data[4] = BYTE_H(_accel[2]);
			_canmsg.CAN_data[5] = BYTE_L(_accel[2]);
			_canmsg.CAN_data[6] = BYTE_H(_accel[3]);
			_canmsg.CAN_data[7] = BYTE_L(_accel[3]);
				
			_canmsg.CAN_length = 8;
			_canmsg.CAN_frameType = DATA_FRAME;
			if (CAN1_sendFrame (1, _canmsg.CAN_messID, _canmsg.CAN_frameType, _canmsg.CAN_length, _canmsg.CAN_data) != ERR_OK)
				AS1_printStringEx("send err\r\n");
			
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
	
	if (VERSION == 0x0125)
		{

			create_F_M(&filter1, &mask1,CLASS_POLLING_DSP,0xFF, _board_ID  , CLASS_PERIODIC_SENS, 0xFF, 0xFF);   
			create_F_M(&filter2, &mask2,CLASS_POLLING_DSP,0xFF, _board_ID+1, CLASS_CANLOADER,     0x00, 0xFF);  
			 	
		}
	if (VERSION == 0x0121 || VERSION == 0x0128 || VERSION == 0x0130)
		{

			create_F_M(&filter1, &mask1,CLASS_POLLING_DSP,0xFF, _board_ID  , CLASS_PERIODIC_SENS, 0xFF, 0xFF);   
			create_F_M(&filter2, &mask2,CLASS_POLLING_DSP,0xFF, _board_ID+1, CLASS_CANLOADER,     0x00, 0xFF);  
			 	
		}	
	else
		{
			create_F_M(&filter1, &mask1,CLASS_POLLING_DSP,0xFF, _board_ID,   CLASS_CANLOADER,     0x00, 0xFF);
			create_F_M(&filter2, &mask2,CLASS_POLLING_DSP,0xFF, _board_ID+1, CLASS_CANLOADER,     0x00, 0xFF); 
		}
		
//	}
	
	
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

/***********************************************************************/
void can_send_additional_info()
{
	byte j = 0;
	byte write_buffer = 0;
    byte counter = 0;
    byte axis = 0;
    
    for (counter = 0 ; counter < 8; counter++)
    {
		do {}
		while (CAN1_getStateTX () == 0) ;
		{ 
			CAN_LEN = 6; 
			CAN_DATA[1]=counter;
			for (j=0; j<4; j++)	CAN_DATA[2+j] = _additional_info[j+counter*4]; 		
			CAN1_send ( CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); 
		} 	
    }		
}
/***********************************************************************/
void can_receive_additional_info()
{
	byte value = 0; 
	byte j = 0;
	
	if (CAN_LEN == 6) 
	{ 
		value = CAN_DATA[1]; 
		
		for (j=0; j<4; j++)
			_additional_info[value*4+j] = CAN_DATA[2+j]; 		
	}
}