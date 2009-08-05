#include "bootloader.h"
#include "comCAN.h"
#include "CAN1.h"
#include "controller.h"
#include "sparser.h"
#include "prog.h"

/*****************************************************************************/
#define SPRS_FIELDLEN_START         5
#define SPRS_FIELDLEN_ADDRESS     	5


static UWord16           sprsIndex;

static UWord16           sprsLength;
static UWord16           sprsAddress;
static mem_eMemoryType   sprsMemoryType;
static UWord16           sprsData[SPRS_BUFFER_LEN];
static UWord16           sprsChecksum;
static UWord16           sprsStart;

static bool              sprsBegin;     /* start record resived */
static bool              sprsEnd;       /* end record received */

extern bool 		    comTimerEn;
extern bool   			downloadEn;
extern byte             _board_ID;

/*****************************************************************************/

////////////////////////////////////////////////////////////////////////
//
// sprsReady()
//
// Parse of CAN bus message.
//
//	Paramter:
//		ReadBuffer - CAN bus messagge
//
////////////////////////////////////////////////////////////////////////
void sprsReady( canmsg_t * ReadBuffer)
{
   UWord16 i;
   dword msgID=0x700;
   msgID |=  _board_ID ;
   switch(ReadBuffer->CAN_data[0])
   {
      case CMD_BOARD:  // command to enable board to downloading
      if (ReadBuffer->CAN_messID ==msgID )
      {
  	    if (ReadBuffer->CAN_length == 1)     
		{
	   	  	comACK(CMD_BOARD);
	  	  	
	   	  	downloadEn = TRUE;
	   	  	comTimerEn = FALSE;
	   	  	setRegBitGroup(TMRC0_CTRL,CM,0);   // Stop counter 
	   	  	TmpXdataVar = 255;	   	  	
	   	}  	
	  }
	  break;
      
      case CMD_BROADCAST: // request board type
  	  {
  	    if (ReadBuffer->CAN_length == 1)     
		{
	      	comTimerEn = FALSE;
   		  	setRegBitGroup(TMRC0_CTRL,CM,0);   // Stop counter 
   	  		TmpXdataVar = 255;
        	comSendType();
        }	
  	  }
	  break;

      case CMD_ADDINFO: // additional info
	  {
	  	comSendAddInfo();
	  } 
	  break;      
	  
   	  case CMD_ADDRESS: // lenght, memory type and address of S-format data packet
	  if (downloadEn == TRUE)
	  {	
      	 comTimerEn = FALSE;
		 if (ReadBuffer->CAN_length == SPRS_FIELDLEN_ADDRESS)     
		 {
	      	sprsLength = ReadBuffer->CAN_data[1];
	        sprsAddress =  ReadBuffer->CAN_data[3];
	        sprsAddress <<= 8;
	        sprsAddress |=  ReadBuffer->CAN_data[2];
            if (ReadBuffer->CAN_data[4])
            {
               sprsMemoryType  = XData;
            }
            else
            {
               sprsMemoryType  = PData;
            }
	        
            sprsIndex = 0;
         }
        else
        {
       //     userError(INDICATE_ERROR_CHARACTER);
       }
	  }
      break;
      
      case CMD_START:  // packet start program 
      if (downloadEn == TRUE)
	  {
		 if (ReadBuffer->CAN_length == SPRS_FIELDLEN_START)     
		 {
	        progFlush();    /* Save the latest data, and write Timeout variable into P flash */
			comACK(CMD_START);	        
         }
         else
         {
        //    userError(INDICATE_ERROR_CHARACTER);
         }
	  }
      break;
      
      case CMD_DATA: // data packet
      if (downloadEn == TRUE)
	  {
    	 for ( i = 1; i < ReadBuffer->CAN_length ; i+=2)
   		 	sprsData[sprsIndex++] = (ReadBuffer->CAN_data[i]) | (ReadBuffer->CAN_data[i+1]<<8);

         sprsLength -= (ReadBuffer->CAN_length - 1);

         if (sprsLength == 0)
         {
			progSaveData ( sprsData, sprsIndex, sprsAddress,  sprsMemoryType);   
			comACK(CMD_DATA);      
         }
	  }
      break;
      
      case CMD_END: // command to end downloading section
 	  {
	    if (ReadBuffer->CAN_length == 1)     
		{
			downloadEn=FALSE;
      		comACK(CMD_END);
      		comExit();
      	} 	
 	  }
      break;

      default:
      {
     //   userError(INDICATE_ERROR_INTERNAL);
      }
      break;   
   }
   
 // switch
}

