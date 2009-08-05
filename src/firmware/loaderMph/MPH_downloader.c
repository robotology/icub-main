// Firmware per download firmware via CAN bus
//
// Il codice è situato nella parte alta della memoria da 0x3000 a 0x3FFF

#include "MPH_bootloader.h"
#include "comCAN.h"
#include "sparser.h"
#include "can-18F248.h"

#define COM_BUFFER_LEN              20

#define SPRS_FIELDLEN_START         5
#define SPRS_FIELDLEN_ADDRESS     	7

#define DEST 0
#define CMD_DONWL 7

static int1    comContinue = TRUE;

extern int16 	CAN_messID;

canmsg_t comBuffer;

struct rx_stat  rxstat;

static int8 	CAN_data[8];					/* CAN bus message */

static int16   sprsIndex;
static int8		sprsErase=0;
static int16   sprsLength;
static int32   sprsAddress;
static int8   	sprsMemoryType;
static int8    sprsData[SPRS_BUFFER_LEN];

static int8 	_board_ID;


#ORG 0x0006, 0x2FFF {}

#org 0x3000, 0x39FF DEFAULT

////////////////////////////////////////////////////////////////////////
//
// can_set_baud()
//
// Configures the baud rate control registers.
//
// CAN bus speed: 1 MBit/s
//
////////////////////////////////////////////////////////////////////////
void can_set_baud(void)
{
   BRGCON1.brp=CAN_BRG_PRESCALAR;
   BRGCON1.sjw=CAN_BRG_SYNCH_JUMP_WIDTH;

   BRGCON2.prseg=CAN_BRG_PROPAGATION_TIME;
   BRGCON2.seg1ph=CAN_BRG_PHASE_SEGMENT_1;
   BRGCON2.sam=CAN_BRG_SAM;
   BRGCON2.seg2phts=CAN_BRG_SEG_2_PHASE_TS;

   BRGCON3.seg2ph=CAN_BRG_PHASE_SEGMENT_2;
   BRGCON3.wakfil=CAN_BRG_WAKE_FILTER;
}

////////////////////////////////////////////////////////////////////////
//
// can_set_mode()
//
// Configures the CAN mode .
//
//   Paramaters:
//		mode - CAN mode
//
////////////////////////////////////////////////////////////////////////
void can_set_mode(CAN_OP_MODE mode)
{
   CANCON.reqop=mode;
   while ( (CANSTAT.opmode) != mode );
}

////////////////////////////////////////////////////////////////////////
//
// can_set_id()
//
// Configures the xxxxSIDL and xxxxSIDH registers to
// configure the defined buffer to use the specified ID
//
//   Paramaters:
//     addr - pointer to first byte of ID register
//     id - ID to set buffer to
//
////////////////////////////////////////////////////////////////////////
void can_set_id(int* addr, int32 id)
{
   int *ptr;

   ptr=addr;

   //eidl
   *ptr=0;

   //eidh
   ptr--;
   *ptr=0;

   //sidl
   ptr--;
   *ptr=(make8(id,0) << 5) & 0xE0;

   //sidh
   ptr--;
   *ptr=(make8(id,0) >> 3) & 0x1F;
   *ptr|=(make8(id,1) << 5) & 0xE0;
}

////////////////////////////////////////////////////////////////////////
//
// can_get_id()
//
// Returns the ID of the specified buffer.  (The opposite of can_set_id())
// This is used after receiving a message, to see which ID sent the message.
//
//   Paramaters:
//     addr - pointer to first byte of ID register
//
//   Returns:
//     The ID of the buffer
//
////////////////////////////////////////////////////////////////////////
int32 can_get_id(int * addr)
{
   int32 ret;
   int * ptr;

   ret=0;
   ptr=addr;

   ptr-=2;    //sidl
   ret=((int32)*ptr & 0xE0) >> 5;

   ptr--;     //sidh
   ret|=((int32)*ptr << 3);
   return(ret);
}

////////////////////////////////////////////////////////////////////////
//
// can_putd()
//
// Puts data on a transmit buffer, at which time the CAN peripheral will
// send when the CAN bus becomes available.
//
//    Paramaters:
//       id - ID to transmit data as
//       data - pointer to data to send
//       len - length of data to send
//       priority - priority of message.  The higher the number, the
//                  sooner the CAN peripheral will send the message.
//                  Numbers 0 through 3 are valid.
//       rtr - TRUE to set the RTR (request) bit in the ID, false if NOT
//
//    Returns:
//       If successful, it will return TRUE
//       If un-successful, will return FALSE
//
////////////////////////////////////////////////////////////////////////
int1 can_putd(int32 id, int * data, int len, int priority, int1 rtr)
{
   int i;
   int * txd0;
   int port;

   txd0=&TXRXBaD0;

    // find emtpy transmitter
    //map access bank addresses to empty transmitter
   if (!TXB0CON.txreq)
   {
      CANCON.win=CAN_WIN_TX0;
      port=0;
   }
   else if (!TXB1CON.txreq)
   {
      CANCON.win=CAN_WIN_TX1;
      port=1;
   }
   else if (!TXB2CON.txreq)
   {
      CANCON.win=CAN_WIN_TX2;
      port=2;
   }
   else
   {
      return(0);
   }

   //set priority.
   TXBaCON.txpri=priority;

   //set tx mask
   can_set_id(TXRXBaID, id);

   //set tx data count
   TXBaDLC=len;
   TXBaDLC.rtr=rtr;

    for (i=0; i<len; i++)
    {
      *txd0=*data;
      txd0++;
      data++;
    }

   //enable transmission
   TXBaCON.txreq=1;

   CANCON.win=CAN_WIN_RX0;

   return(1);
}

////////////////////////////////////////////////////////////////////////
//
// can_getd()
//
// Gets data from a receive buffer, if the data exists
//
//    Returns:
//      id - ID who sent message
//      data - pointer to array of data
//      len - length of received data
//      stat - structure holding some information (such as which buffer
//             recieved it, ext or standard, etc)
//
//    Returns:
//      Function call returns a TRUE if there was data in a RX buffer, FALSE
//      if there was none.
//
////////////////////////////////////////////////////////////////////////
int1 can_getd(int32 & id, int * data, int & len, struct rx_stat & stat)
{
    int i;
    int * ptr;

    if (RXB0CON.rxful)
    {
        CANCON.win=CAN_WIN_RX0;
        stat.buffer=0;

        CAN_INT_RXB0IF=0;

        stat.err_ovfl=COMSTAT.rx0ovfl;
        COMSTAT.rx0ovfl=0;

        if (RXB0CON.rxb0dben)
        {
           stat.filthit=RXB0CON.filthit0;
        }
    }
    else if ( RXB1CON.rxful )
    {
        CANCON.win=CAN_WIN_RX1;
        stat.buffer=1;

        CAN_INT_RXB1IF=0;

        stat.err_ovfl=COMSTAT.rx1ovfl;
        COMSTAT.rx1ovfl=0;

        stat.filthit=RXB1CON.filthit;
    }
    else
    {
        return (0);
    }

    len = RXBaDLC.dlc;
    stat.rtr=RXBaDLC.rtr;

    stat.ext=TXRXBaSIDL.ext;
    id=can_get_id(TXRXBaID);

    ptr = &TXRXBaD0;
    for ( i = 0; i < len; i++ )
    {
        *data = *ptr;
        data++;
        ptr++;
    }

    // return to default addressing
    CANCON.win=CAN_WIN_RX0;

    stat.inv=CAN_INT_IRXIF;
    CAN_INT_IRXIF = 0;

    if (stat.buffer)
    {
      RXB1CON.rxful=0;
    }
    else
    {
      RXB0CON.rxful=0;
    }
    return(1);
}

////////////////////////////////////////////////////////////////////////
//
// can_init()
//
// Initializes CAN peripheral.  Sets the RX filter and masks so the
// CAN peripheral will receive all incoming IDs.  Configures both RX buffers
// to only accept valid valid messages (as opposed to all messages, or all
// extended message, or all standard messages).  Also sets the tri-state
// setting of B2 to output, and B3 to input
//
//////////////////////////////////////////////////////////////////////////////
void can_init(void)
{
   can_set_mode(CAN_OP_CONFIG);   //must be in config mode before params can be set
   can_set_baud();

   RXB0CON=0;
   RXB0CON.rxm=CAN_RX_VALID;
   RXB0CON.rxb0dben=CAN_USE_RX_DOUBLE_BUFFER;
   RXB1CON=RXB0CON;

   CIOCON.endrhi=CAN_ENABLE_DRIVE_HIGH;
   CIOCON.cancap=CAN_ENABLE_CAN_CAPTURE;

   can_set_id(RX0MASK, 0x000F);  //set mask 0
   can_set_id(RX0FILTER0, _board_ID);  //set filter 0 of mask 0
   can_set_id(RX0FILTER1, 0x000F);  //set filter 1 of mask 0

   can_set_id(RX1MASK, 0x000F);  //set mask 1
   can_set_id(RX1FILTER2, 0x000F);  //set filter 0 of mask 1
   can_set_id(RX1FILTER3, 0x000F);  //set filter 1 of mask 1
   can_set_id(RX1FILTER4, 0x000F);  //set filter 2 of mask 1
   can_set_id(RX1FILTER5, 0x000F);  //set filter 3 of mask 1

   set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in

   can_set_mode(CAN_OP_NORMAL);
}

////////////////////////////////////////////////////////////////////////
//
// comExit()
//
// End communication loop.
//
////////////////////////////////////////////////////////////////////////
void comExit (void)
{
   comContinue = FALSE;
}

////////////////////////////////////////////////////////////////////////
//
// comACK()
//
// Send acknowledge message.
//
////////////////////////////////////////////////////////////////////////
void comACK (byte cmd )
{
	CAN_data[0] = cmd;
	CAN_data[1] = 1;
	can_putd(CAN_messID, CAN_data, 2,3,0);
}

////////////////////////////////////////////////////////////////////////
//
// comSendType()
//
// Send board characteristic: board type, version e revision of firmware.
//
////////////////////////////////////////////////////////////////////////
void comSendType(void)
{
	CAN_data[0] = CMD_BROADCAST;
	CAN_data[1] = 1;//_board_TYPE;
	CAN_data[2] = read_eeprom(1);//_board_FW_VER;
	CAN_data[3] = read_eeprom(2);//_board_FW_REV;
	can_putd(CAN_messID, CAN_data, 4,3,0); //  invia messaggio
}

////////////////////////////////////////////////////////////////////////
//
// comWaitEnd()
//
// Wait the end of trasmission
//
//
////////////////////////////////////////////////////////////////////////
void comWaitEnd(void)
{
	do
   {
      while (!(RXB0CON.rxful || RXB1CON.rxful));
      can_getd(comBuffer.CAN_messID, comBuffer.CAN_data,
            	comBuffer.CAN_length, rxstat);  // leggi messaggio
	} while ((comBuffer.CAN_data[0]==CMD_START) ||
			  (comBuffer.CAN_data[0]==CMD_END));
}

////////////////////////////////////////////////////////////////////////
//
// delayTx()
//
// Wait 50 us after that last trasmission is finished
//
//
////////////////////////////////////////////////////////////////////////
void delayTx(void)
{
	int16 i;
	while (!((!TXB0CON.txreq || !TXB1CON.txreq || !TXB2CON.txreq)));
	for (i=0; i<1000; i++)
	  delay_cycles(200); // delay 50 us
}

////////////////////////////////////////////////////////////////////////
//
// userError()
//
// Send error message, wait the end of trasmission and reset CPU.
//
//
////////////////////////////////////////////////////////////////////////
void userError(int Error)
{
	CAN_data[0] = CMD_ERR;
	CAN_data[1] = Error;
  	can_putd(CAN_messID, CAN_data, 2,3,0); //  invia messaggio
	comWaitEnd();
	delayTx();
// jmp to program.......
   reset_cpu();
}

////////////////////////////////////////////////////////////////////////
//
// WriteAndVerify()
//
// Write and verify program memory.
//
//
////////////////////////////////////////////////////////////////////////
int1 WriteAndVerify(void)
{
	int8 i,current;
	write_program_memory(sprsAddress,sprsData,sprsIndex);
	for (i=0; i<sprsIndex;	i++) {
		current = read_program_eeprom(sprsAddress+i);
		if (current != sprsData[i]) return(1);
	}
	return(0);
}

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
   int16 i;
   switch(ReadBuffer->CAN_data[0])
   {
   	  case CMD_BOARD:  // command to enable board to downloading
   	  {
   	  	comACK(CMD_BOARD);
   	  }
   	  break;
      case CMD_ADDRESS:	// lenght and address of the Hex-Intel data packet
      {
		 if (ReadBuffer->CAN_length == SPRS_FIELDLEN_ADDRESS)
		 {
	      	sprsLength = ReadBuffer->CAN_data[1];
	        sprsAddress =  ReadBuffer->CAN_data[6];
	        sprsAddress <<= 8;
	        sprsAddress |=  ReadBuffer->CAN_data[5];
	        sprsAddress <<= 8;
	        sprsAddress |=  ReadBuffer->CAN_data[3];
	        sprsAddress <<= 8;
	        sprsAddress |=  ReadBuffer->CAN_data[2];
            if (ReadBuffer->CAN_data[4])
            {
               sprsMemoryType  = 0;
            }
            else
            {
               sprsMemoryType  = 1;
            }
            sprsIndex = 0;
            if (!sprsErase) {
	          // if first time ease the program memory
            	for(i=0x0000;i<=0x2FFF;i+=getenv("FLASH_ERASE_SIZE"))
					erase_program_eeprom(i);
				sprsErase=1;
			}
         }
         else
         {
            userError(INDICATE_ERROR_CHARACTER);
         }
      }
      break;
      case CMD_START:  // packet start program (not used)
      {
		 if (ReadBuffer->CAN_length == SPRS_FIELDLEN_START)
		 {
			comACK(CMD_START);
         }
         else
         {
            userError(INDICATE_ERROR_CHARACTER);
         }
      }
      break;
      case CMD_DATA: // data packet
      {
    	 for ( i = 1; i < ReadBuffer->CAN_length ; i++)
   		 	sprsData[sprsIndex++] = (ReadBuffer->CAN_data[i]);

         sprsLength -= (ReadBuffer->CAN_length - 1);

         if ((sprsLength == 0) && (sprsAddress <= 0x7FFF))
         {
	        if (!WriteAndVerify())  // write and verify program memory
	        	comACK(CMD_DATA);
	        else
	         	userError(INDICATE_ERROR_FLASH);
         }
      }
      break;
      case CMD_END:  // command to end downloading section
      {
      	comACK(CMD_END);
      	comExit();
      }
      break;
      case CMD_BROADCAST:  // request board type
      {
        comSendType();
      }
      break;
      default:
      {
         userError(INDICATE_ERROR_INTERNAL);
      }
   } /* switch */
}

////////////////////////////////////////////////////////////////////////
//
// comMainLoop()
//
// Communication loop. Wait for CAN frames and parses the messagge.
// The loop ends when comContinue is false.
//
////////////////////////////////////////////////////////////////////////
void comMainLoop(void)
{
   while (comContinue)
   {
      if ((RXB0CON.rxful || RXB1CON.rxful))
      {
         /* Received CAN frame */
         if(can_getd(comBuffer.CAN_messID, comBuffer.CAN_data,
            		  comBuffer.CAN_length, rxstat)) { // read messagge
           sprsReady(&comBuffer);      /* call protocol parser */
         }
		 else
         {
            userError(INDICATE_ERROR_RECEIVE);
         }
      }
   }  /* while */

}

//
//
//
#ORG DEFAULT
#org 0x3A00, 0x3CBE
void main(void)
{
   // set can bus address here.
   write_eeprom (0, 5);
   write_eeprom (1, 1);
   write_eeprom (2, 2);
   
	_board_ID = read_eeprom(0);  // read ID board

	can_init();					     // CAN init

   CAN_messID = (CMD_DONWL << 8) | (_board_ID << 4) | DEST;  // build ID tx CAN messagge

   comACK(CMD_BOARD);			  // send acknowledge for CMD_BOARD command
   // Start communication loop. Wait for Hex-Intel file via CAN bus /

   comMainLoop();
	delayTx();

   // jmp to program.......
	reset_cpu();
}
