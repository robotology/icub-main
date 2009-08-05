#include "strain_board.h"
#include "ad.h"
#include "can1.h" 

Int16 _strain[STRAIN_MAX][6];
Int16 _strain_init[STRAIN_MAX][6];
Int16 _strain_old[STRAIN_MAX][6];

byte  _strain_wtd[STRAIN_MAX];

/*************************************************************************** 
 * Initialization
 ***************************************************************************/
void start_strain (word can_address)
{
	//start the can transmission. CAN must be already intialized.
	canmsg_t canmsg;
	canmsg.CAN_messID = can_address;
	canmsg.CAN_data[0] = 0x07;
	canmsg.CAN_data[1] = 0x00;
	canmsg.CAN_length = 2;
	canmsg.CAN_frameType = DATA_FRAME;
	CAN1_send (canmsg.CAN_messID, canmsg.CAN_frameType, canmsg.CAN_length, canmsg.CAN_data);	
}

/*************************************************************************** 
 * Initialization
 ***************************************************************************/
void stop_strain (word can_address)
{
	//start the can transmission. CAN must be already intialized.
	canmsg_t canmsg;
	canmsg.CAN_messID = can_address;
	canmsg.CAN_data[0] = 0x07;
	canmsg.CAN_data[1] = 0x01;
	canmsg.CAN_length = 2;
	canmsg.CAN_frameType = DATA_FRAME;
	CAN1_send (canmsg.CAN_messID, canmsg.CAN_frameType, canmsg.CAN_length, canmsg.CAN_data);	
}

/*************************************************************************** 
 * Initialization
 ***************************************************************************/
void init_strain (void)
{
	byte i=0;
	byte k=0;
	canmsg_t canmsg;
		
	for (i=0; i<6; i++)
		for (k=0; k<STRAIN_MAX; k++)
		{
			_strain[k][i] = 0;
			_strain_init[k][i] = 0;
			_strain_old[k][i] = 0;			
		}
	
	for (k=0; k<STRAIN_MAX; k++)
		{
			_strain_wtd[k]= STRAIN_SAFE;	
		}
				
	#if VERSION == 0x0170 || VERSION == 0x0172
		//start the can transmission. CAN must be already intialized.
		start_strain(0x205);
	#elif VERSION ==0x0173 || VERSION ==0x0174
		//start the can transmission. CAN must be already intialized.
		start_strain(0x20B);
		start_strain(0x20C);	
	#elif VERSION == 0x0171 
		AD_init ();
		AD_enableIntTriggerA ();
		AD_enableIntTriggerB ();		
	#endif
}

/*************************************************************************** 
 * strain gauges sampling
 ***************************************************************************/
word read_strain(byte jnt, bool sign)
{
	word temp;
	Int32 temporary;	

	#if VERSION == 0x0170 || VERSION == 0x0172
		return _strain[0][jnt];
	#else if VERSION == 0x0171 
		switch (jnt)
		{
			case 0: AD_getChannel16A (2, &temp); break;
			case 1: AD_getChannel16B (2, &temp); break;			
		}

		temporary = (Int32) temp;
		if (!sign)	temporary = -temporary;
		
		return temporary;
	#endif
		
	
}