/////////////////////////////////////////////////////////////////////////
//// CAN Library routines for Microchip's PIC18Cxx8 and 18Fxx8 line  ////
////                                                                 ////
//// This library provides the following functions:                  ////
////  (for more information on these functions see the comment       ////
////   header above each function)                                   ////
////                                                                 ////
////    can_init - Configures the PIC18xxx8 CAN peripheral           ////
////                                                                 ////
////    can_set_baud - Sets the baud rate control registers          ////
////                                                                 ////
////    can_set_mode - Sets the CAN module into a specific mode      ////
////                                                                 ////
////    can_set_id - Sets the standard and extended ID               ////
////                                                                 ////
////    can_get_id - Gets the standard and extended ID               ////
////                                                                 ////
////    can_putd - Sends a message/request with specified ID         ////
////                                                                 ////
////    can_getd - Returns specifid message/request and ID           ////
////                                                                 ////
////    can_kbhit - Returns true if there is data in one of the      ////
////                receive buffers                                  ////
////                                                                 ////
////    can_tbe - Returns true if the transmit buffer is ready to    ////
////              send more data                                     ////
////                                                                 ////
////    can_abort - Aborts all pending transmissions                 ////
////                                                                 ////
//// PIN_B3 is CANRX, and PIN_B2 is CANTX.  You will need a CAN      ////
//// transeiver to connect these pins to CANH and CANL bus lines.    ////
////                                                                 ////
////                                                                 ////
/////////////////////////////////////////////////////////////////////////

#include "can-18F248.h"

int16 _board_ID;

#if CAN_DO_DEBUG
 #define can_debug printf
#else
 #define can_debug
#endif


//macros
#define can_kbhit()                 (RXB0CON.rxful || RXB1CON.rxful)
#define can_tbe()                   (!TXB0CON.txreq || !TXB1CON.txreq || !TXB2CON.txreq)
#define can_abort()                 (CANCON.abat=1)


////////////////////////////////////////////////////////////////////////
//
// can_init()
//
// Initializes PIC18xxx8 CAN peripheral.  Sets the RX filter and masks so the
// CAN peripheral will receive all incoming IDs.  Configures both RX buffers
// to only accept valid valid messages (as opposed to all messages, or all
// extended message, or all standard messages).  Also sets the tri-state
// setting of B2 to output, and B3 to input (apparently the CAN peripheral
// doesn't keep track of this)
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

   can_set_id(RX0MASK,0x000F, CAN_USE_STANDAR_ID);  //set mask 0 //0x000F
   can_set_id(RX0FILTER0, _board_ID, CAN_USE_STANDAR_ID);  //set filter 0 of mask 0
   can_set_id(RX0FILTER1, 0x000F, CAN_USE_STANDAR_ID);  //set filter 1 of mask 0

   can_set_id(RX1MASK, 0x0FFF, CAN_USE_STANDAR_ID);  //set mask 1
   can_set_id(RX1FILTER2, 0x200, CAN_USE_STANDAR_ID);  //set filter 0 of mask 1
   can_set_id(RX1FILTER3, 0x200, CAN_USE_STANDAR_ID);  //set filter 1 of mask 1
   can_set_id(RX1FILTER4, 0x200, CAN_USE_STANDAR_ID);  //set filter 2 of mask 1
   can_set_id(RX1FILTER5, 0x200, CAN_USE_STANDAR_ID);  //set filter 3 of mask 1

   set_tris_b((*0xF93 & 0xFB ) | 0x08);   //b3 is out, b2 is in

   can_set_mode(CAN_OP_NORMAL);
}

////////////////////////////////////////////////////////////////////////
//
// can_set_baud()
//
// Configures the baud rate control registers.
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

void can_set_mode(CAN_OP_MODE mode)
{
   CANCON.reqop=mode;
   while( (CANSTAT.opmode) != mode );
}


////////////////////////////////////////////////////////////////////////
//
// can_set_id()
//
// Configures the xxxxEIDL, xxxxEIDH, xxxxSIDL and xxxxSIDH registers to
// configure the defined buffer to use the specified ID
//
//   Paramaters:
//     addr - pointer to first byte of ID register, starting with xxxxEIDL.
//            For example, a pointer to RXM1EIDL
//     id - ID to set buffer to
//     ext - Set to TRUE if this buffer uses an extended ID, FALSE if not
//
////////////////////////////////////////////////////////////////////////
void can_set_id(int* addr, int32 id, int1 ext)
{
   int *ptr;

   ptr=addr;

   if (ext) {  //extended
      //eidl
      *ptr=make8(id,0); //0:7

      //eidh
      ptr--;
      *ptr=make8(id,1); //8:15

      //sidl
      ptr--;
      *ptr=make8(id,2) & 0x03;   //16:17
      *ptr|=(make8(id,2) << 3) & 0xE0; //18:20
      *ptr|=0x08;


      //sidh
      ptr--;
      *ptr=((make8(id,2) >> 5) & 0x07 ); //21:23
      *ptr|=((make8(id,3) << 3) & 0xF8);//24:28
   }
   else {   //standard
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
}

////////////////////////////////////////////////////////////////////////
//
// can_get_id()
//
// Returns the ID of the specified buffer.  (The opposite of can_set_id())
// This is used after receiving a message, to see which ID sent the message.
//
//   Paramaters:
//     addr - pointer to first byte of ID register, starting with xxxxEIDL.
//            For example, a pointer to RXM1EIDL
//     ext - Set to TRUE if this buffer uses an extended ID, FALSE if not
//
//   Returns:
//     The ID of the buffer
//
////////////////////////////////////////////////////////////////////////
int32 can_get_id(int * addr, int1 ext)
{
   int32 ret;
   int * ptr;

   ret=0;
   ptr=addr;

   if (ext) {
      ret=*ptr;  //eidl

      ptr--;     //eidh
      ret|=((int32)*ptr << 8);

      ptr--;     //sidl
      ret|=((int32)*ptr & 0x03) << 16;
      ret|=((int32)*ptr & 0xE0) << 13;

      ptr--;     //sidh
      ret|=((int32)*ptr << 21);

   }
   else {
      ptr-=2;    //sidl
      ret=((int32)*ptr & 0xE0) >> 5;

      ptr--;     //sidh
      ret|=((int32)*ptr << 3);
   }

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
//       ext - TRUE to use an extended ID, FALSE if not
//       rtr - TRUE to set the RTR (request) bit in the ID, false if NOT
//
//    Returns:
//       If successful, it will return TRUE
//       If un-successful, will return FALSE
//
////////////////////////////////////////////////////////////////////////
int1 can_putd(int32 id, int * data, int len, int priority, int1 ext, int1 rtr)
{
   int i;
   int * txd0;
   int port;

   txd0=&TXRXBaD0;

    // find emtpy transmitter
    //map access bank addresses to empty transmitter
   if (!TXB0CON.txreq) {
      CANCON.win=CAN_WIN_TX0;
      port=0;
   }
   else if (!TXB1CON.txreq) {
      CANCON.win=CAN_WIN_TX1;
      port=1;
   }
   else if (!TXB2CON.txreq) {
      CANCON.win=CAN_WIN_TX2;
      port=2;
   }
   else {
      #if CAN_DO_DEBUG
         can_debug("\r\nCAN_PUTD() FAIL: NO OPEN TX BUFFERS\r\n");
      #endif
      return(0);
   }

   //set priority.
   TXBaCON.txpri=priority;

   //set tx mask
   can_set_id(TXRXBaID, id, ext);

   //set tx data count
   TXBaDLC=len;
   TXBaDLC.rtr=rtr;

    for (i=0; i<len; i++) {
      *txd0=*data;
      txd0++;
      data++;
    }

   //enable transmission
   TXBaCON.txreq=1;

   CANCON.win=CAN_WIN_RX0;

   #if CAN_DO_DEBUG
            can_debug("\r\nCAN_PUTD(): BUFF=%U ID=%LX LEN=%U PRI=%U EXT=%U RTR=%U\r\n", port, id, len, priority, ext, rtr);
            if ((len)&&(!rtr)) {
               data-=len;
               can_debug("  DATA = ");
               for (i=0;i<len;i++) {
                  can_debug("%X ",*data);
                  data++;
               }
               can_debug("\r\n");
            }
   #endif

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
      #if CAN_DO_DEBUG
         can_debug("\r\nFAIL ON CAN_GETD(): NO MESSAGE IN BUFFER\r\n");
      #endif
      return (0);
    }

    len = RXBaDLC.dlc;
    stat.rtr=RXBaDLC.rtr;

    stat.ext=TXRXBaSIDL.ext;
    id=can_get_id(TXRXBaID,stat.ext);

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

    #if CAN_DO_DEBUG
       can_debug("\r\nCAN_GETD(): BUFF=%U ID=%LX LEN=%U OVF=%U ", stat.buffer, id, len, stat.err_ovfl);
       can_debug("FILT=%U RTR=%U EXT=%U INV=%U", stat.filthit, stat.rtr, stat.ext, stat.inv);
       if ((len)&&(!stat.rtr))
       {
          data-=len;
          can_debug("\r\n    DATA = ");
          for (i=0;i<len;i++)
          {
            can_debug("%X ",*data);
            data++;
          }
       }
       can_debug("\r\n");
    #endif

    return(1);
}



