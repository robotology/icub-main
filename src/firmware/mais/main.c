//  A.d.C. M A I S / S T R A I N 
//  Author C.Lorini and M.Maggiali
//  Rev. 0.0 del 25/09/2007
//
//  Tested on dsPIC30F4013
//  pic30-gcc V4.03
//  MPLAB IDE ver 8.0
// 
//  Revisiom List
//  Revision 0.0: 
// TODO: Buffer SW trasmissione messaggi can (se la linea e`piena si rischia l'overrun)
// TODO: verifica del rumore con 1 segnale forte sul MUX e segnali piccoli
// TODO: verifica della mascheratura dei comandi (risponde a 0x205, 0x605, ma non a 0x105)
// TODO: verifica del comportamento a comandi non gestiti (nel case manca il DEFAULT!)

//  Revision 1.0: (Marco Randazzo)
// TODO:

//  Revision 1.1.2: Marco Maggiali 20/04/09
//
//  The main has been divided in many files
//  The CAN TX is done with interrupt
//
//
//  Revision 1.1.3: Marco Maggiali 30/04/09
//
//  In T2 (from 1ms to 10-20ms) interrupt the CAN messages are prepared and sent
//  T1 (1ms) is used for the for(;;) where is done the acquisition of the AD signals
//  
//
//  Revision 1.1.4: Marco Maggiali 15/05/09
//
//  The yellow led is now switched off, because of power consumption issue 
//  The red led is switched on.
//  maggia: default address 0xf  
//  periodic message id 0x300
//
//  Revision 1.1.5: Marco Maggiali 09/06/09
//
//  The Firmware does not stop to do the broadcast if a connection from the CANLOADER is done. DisableT1,T2 and T3 are in CMD_BOARD only. 
//
//
//  Revision 1.1.6: Marco Maggiali 10/06/09
//
//  The 8bits data messages are organized as following: data 0to 6 in the first message, from 7 to 14 in the second one.
//
//  Revision 2.0: Marco Maggiali 28/01/10
//
//  After SVN installation there was a mass in the code and there is no way to go back
//

#include <p30f4013.h>
#include <timer.h>
#include <can.h>
#include <adc12.h>
#include <spi.h>
#include <string.h>
#include <libpic30.h>
#include <dsp.h>
#include <reset.h>
#include "can_interface.h"
#include "eeprom.h"
#include "IIR_filter.h"
#include "ports.h"
#include "int_adc.h"
#include "ext_mux.h"
#include "errors.h"
#include "utils.h"
#include "options.h"



#ifdef MAIS 
#ifdef STRAIN
#error "Cannot define both MAIS and STRAIN!"
#endif
#endif

#ifndef MAIS 
#ifndef STRAIN
#error "At least MAIS or STRAIN must be defined!"
#endif
#endif

//board types
#define BOARD_TYPE_STRAIN  0x06
#define BOARD_TYPE_MAIS    0x07

#define MAIS_VERSION       0x02

#define MAIS_RELEASE       0x00


#define MAIS_BUILD         0x00


#ifdef MAIS 
  char VERSION=   	MAIS_VERSION;
  char RELEASE=    	MAIS_RELEASE;
  char BUILD=     	MAIS_BUILD;
  char BOARD_TYPE= 	BOARD_TYPE_MAIS;
#endif 

//leds 
#define ledY   LATFbits.LATF4 
#define ledB   LATDbits.LATD1
unsigned char  toggledY=0;  

unsigned char  T1flag=0;

//options (1/0)
#define SAVE_EEPROM_ATONCE    1
char CAN_ACK_EVERY_MESSAGE=   1;

//board types
#define BOARD_TYPE_STRAIN  0x06
#define BOARD_TYPE_MAIS    0x07

//// CONFIGURATION BITS INITIALIZATION

// inizializzazione bit di configurazione (p30f4013.h)
_FOSC(CSW_FSCM_OFF & ECIO_PLL8); 
  // Clock switching disabled Fail safe Clock Monitor disabled
  // External clock with PLL x8 (10MHz*8->Fcycle=80/4=20MIPS)
_FGS(CODE_PROT_OFF); // Code protection disabled

_FWDT(WDT_OFF);     // WD disabled
//_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_2); // WD enabled 1:512*16

//_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_27);  // BOR 2.7V POR 64msec
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_27);  // BOR 2.7V POR 64msec

// info codes
// reset from BOR
#define INFO_TRSET_isBOR            9
#define INFO_TRSET_isPOR            10
#define INFO_TRSET_isLVD            11
#define INFO_TRSET_isMCLR           12
#define INFO_TRSET_isWDTTO          13
#define INFO_TRSET_isWDTWU          14
#define INFO_TRSET_isWU             15

struct s_eeprom _EEDATA(1) ee_data = 
{
  0x0,           // EE_B_EEErased             :1
  0x0,           // EE_B_EnableWD             :1
  0x1,           // EE_B_EnableBOR            :1
  0x0E,          // EE_CAN_BoardAddress;      :8
  0x01,          // EE_CAN_MessageDataRate    :8
  0x04,          // EE_CAN_Speed;             :8
  {
    0x01,0x01,   // EE_AN_ActiveChannels      :8
    0x01,0x01,
    0x01,0x01,
    0x01,0x01,
    0x01,0x01,
    0x01,0x01,
    0x01
  },
  0x00,          // EE_AN_Selected channel
  0x0001,        // EE_AN_ChannelScanningTime :16
  {
    0x01FF,0x01FF, // EE_AN_ChannelOffset[6]  :6*16
    0x01FF,0x01FF,
    0x01FF,0x01FF 
  },
  {
    0x0000,0x0000, // EE_AN_ChanneValue[13]    :13*16
    0x0000,0x0000,
    0x0000,0x0000, 
    0x0000,0x0000, 
    0x0000,0x0000,
    0x0000,0x0000,
    0x0000
  },
  {
    0x0000,0x0000,0x0000 // EE_TF_TorqueValue :3*16
  },
  {
    0x0000,0x0000,0x0000 // EE_TF_ForceValue[3] :3*16
  },
  {
    {0x7FFF,0,0,0,0,0},  // TT_TF_TMatrix
    {0,0x7FFF,0,0,0,0},
    {0,0,0x7FFF,0,0,0},
    {0,0,0,0x7FFF,0,0},
    {0,0,0,0,0x7FFF,0},
    {0,0,0,0,0,0x7FFF},
  },
  {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31},
  0x0000 // Checksum
};

// Board Configuration image from EEPROM
struct s_eeprom BoardConfig = {0}; 


char filter_enable = 1;
char mux_enable = 1;
char muxed_chans =5;

//
// prototypes 
//
extern void EEnqueue(unsigned x);

//
// This function clear the wachtdog
//
void idle(void)
// things that can be done during idle time
{
  ClrWdt(); // Reset WDT
}
//
// ADC12 IRQ Service Routines
// 
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void)
{   
  IFS0bits.ADIF = 0;
}  
//
// SPI IRQ Service Routines
// 
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{   
  IFS0bits.SPI1IF = 0;
}  
//
// TIMER 1 IRQ Service Routine
// used for Timestqamping and low frequency operation (LED ....)
//
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{    
  T1flag=1; 
  IFS0bits.T1IF = 0; // clear flag  
}

//
// TIMER 3 IRQ Service Routine
// used for strain gauges sampling
// and IIR Low pass filtering
// Durata totale 17 uSec 
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
{    
  IFS0bits.T3IF = 0; // clear flag 
}

//
// TIMER 2 IRQ Service Routine
// used for CAN messages transmission of Force/Torque
// durata totale 140uSec
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{    
  unsigned int SID; //,adc;
  unsigned char HESData1[8], HESData2[8], HESData3[8],HESData4[8];
  CANin12bit msg1;
//  static unsigned char Alt = 1; 
//  ledY=0;
switch (HESDATA_RESOLUTION)
{
 	case HESDATA_IS_8_BIT:
 	{
	  HESData1[0] = BoardConfig.EE_AN_ChannelValue[0] >>4;  
	  HESData1[1] = BoardConfig.EE_AN_ChannelValue[1] >>4;  
	  HESData1[2] = BoardConfig.EE_AN_ChannelValue[2] >>4;  
	  HESData1[3] = BoardConfig.EE_AN_ChannelValue[3] >>4;  
	  HESData1[4] = BoardConfig.EE_AN_ChannelValue[4] >>4;  
	  HESData1[5] = BoardConfig.EE_AN_ChannelValue[5] >>4;  
	  HESData1[6] = BoardConfig.EE_AN_ChannelValue[6] >>4;  
	  
	  
	  HESData2[0] = BoardConfig.EE_AN_ChannelValue[7] >>4;  
	  HESData2[1] = BoardConfig.EE_AN_ChannelValue[8 ] >>4;  
	  HESData2[2] = BoardConfig.EE_AN_ChannelValue[9 ] >>4;  
	  HESData2[3] = BoardConfig.EE_AN_ChannelValue[10] >>4;  
	  HESData2[4] = BoardConfig.EE_AN_ChannelValue[11] >>4;  
	  HESData2[5] = BoardConfig.EE_AN_ChannelValue[12] >>4;  
	  HESData2[6] = BoardConfig.EE_AN_ChannelValue[13] >>4;  
	  HESData2[7] = BoardConfig.EE_AN_ChannelValue[14] >>4;
	  
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES0TO6) ;
//	 CAN1_send(SID,0,7,HESData1);
	  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData1,7,0); // buffer 0 
	  // HES data 
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES7TO14) ;
	// CAN1_send(SID,0,8,HESData2);
	  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData2,8,1); // buffer 1 
 	}
 	break;
 	
 	case HESDATA_IS_16_BIT:
 	{
	  memcpy(HESData1,&BoardConfig.EE_AN_ChannelValue[0],8);
	  memcpy(HESData2,&BoardConfig.EE_AN_ChannelValue[4],8);
	
	  memcpy(HESData3,&BoardConfig.EE_AN_ChannelValue[8],8);
	  memcpy(HESData4,&BoardConfig.EE_AN_ChannelValue[12],6);
	  // Load message ID , Data into transmit buffer and set transmit request bit
	  // class, source, type for periodIc messages
	  // HES data 
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES0TO3) ;
//	  CAN1_send(SID,0,8,HESData1);
	  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData1,8,0); // buffer 0 
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES4TO7) ;
//	  CAN1_send(SID,0,8,HESData2);
	  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData1,8,1); // buffer 0 
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES8TO11) ;
//	  CAN1_send(SID,0,8,HESData3);
	  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData1,8,2); // buffer 0 
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES12TO14) ;
	  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData1,8,0); // buffer 0 
//	  CAN1_send(SID,0,6,HESData4);
 	}
 	break;
 	
 	case HESDATA_IS_12_BIT:
 	{
 	  memcpy(HESData1,&BoardConfig.EE_AN_ChannelValue[0],8);
	  memcpy(HESData2,&BoardConfig.EE_AN_ChannelValue[4],8);
	  memcpy(HESData3,&BoardConfig.EE_AN_ChannelValue[8],8);
	  memcpy(HESData4,&BoardConfig.EE_AN_ChannelValue[12],6);
	  
	  msg1.data0=BoardConfig.EE_AN_ChannelValue[0]&0xFFF;
	  msg1.data1=BoardConfig.EE_AN_ChannelValue[1]&0xFFF;
	  msg1.data2=BoardConfig.EE_AN_ChannelValue[2]&0xFFF;
	  msg1.data3=BoardConfig.EE_AN_ChannelValue[3]&0xFFF;
	  msg1.data4=BoardConfig.EE_AN_ChannelValue[4]&0xFFF;
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES0TO4) ;
//	  CAN1_send(SID,0,8,msg1.data);
	  
      msg1.data0=BoardConfig.EE_AN_ChannelValue[5]&0xFFF;
	  msg1.data1=BoardConfig.EE_AN_ChannelValue[6]&0xFFF;
	  msg1.data2=BoardConfig.EE_AN_ChannelValue[7]&0xFFF;
	  msg1.data3=BoardConfig.EE_AN_ChannelValue[8]&0xFFF;
	  msg1.data4=BoardConfig.EE_AN_ChannelValue[9]&0xFFF;
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES5TO9) ;
//	  CAN1_send(SID,0,8,msg1.data);
	  
	  msg1.data0=BoardConfig.EE_AN_ChannelValue[10]&0xFFF;
	  msg1.data1=BoardConfig.EE_AN_ChannelValue[11]&0xFFF;
	  msg1.data2=BoardConfig.EE_AN_ChannelValue[12]&0xFFF;
	  msg1.data3=BoardConfig.EE_AN_ChannelValue[13]&0xFFF;
	  msg1.data4=BoardConfig.EE_AN_ChannelValue[14]&0xFFF;
	  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES10TO14) ;
//	  CAN1_send(SID,0,8,msg1.data);
	  
 	}
 	default:
 	break;
 	
}
  toggledY++;  
  // WriteTimer2(0x0);
//  ledY=1;
  IFS0bits.T2IF = 0; // clear flag  
}

int main(void)
{ 
  unsigned int i;
  unsigned int match_value;
  int ch; //the channel of the mux
  unsigned short Chn = 0;
  unsigned flag=0;
  // Test CAN vars
  unsigned char datalen;
  unsigned char Txdata[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88}; 
  canmsg_t CAN_Msg;
  char FilterNo,tx_rx_no;
  // #define  DATAARRAY 0x1820
  // unsigned char *datareceived = (unsigned char *) DATAARRAY;  
  unsigned char Rxdata[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}; 
  // EEDATA Access vars
  _prog_addressT EE_addr;
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                               init code
  //////////////////////////////////////////////////////////////////////////////////////////////////////////// 
  
  // init/clear error queue and flags

  // turn IO ports and periferals
  InitPorts();
    //
  // ADC12 used only for MAIS Boards
  // HES sampling
  //
  init_internal_adc();
  //Init external Mux
  AMUXInit();
  
  // Reset   
  // Enqueue of reset cause
  if(isBOR!=0)
    EEnqueue(INFO_TRSET_isBOR);
  if(isPOR!=0)
    EEnqueue(INFO_TRSET_isPOR);
  if(isLVD!=0)
    EEnqueue(INFO_TRSET_isLVD);
  if(isMCLR!=0)
    EEnqueue(INFO_TRSET_isMCLR);
  if(isWDTTO!=0)
    EEnqueue(INFO_TRSET_isWDTTO);
  if(isWDTWU!=0)
    EEnqueue(INFO_TRSET_isWDTWU);
  if(isWU!=0)
    EEnqueue(INFO_TRSET_isWU);
  //
  // EEPROM Data Recovery
  // 
  // Initialize BoardConfig variable in RAM with the Data EEPROM stored values 
  RecoverConfigurationFromEEprom();
  // todo: checksum verification
  //
  // Timer 1 irq
  // Low frequency operation
  //
  ConfigIntTimer1(T1_INT_PRIOR_7 & T1_INT_OFF);
  WriteTimer1(0);
  // with clock set as 10MHz x8PLL 
  // (fosc*8/4)/(prescaler*match) -> 80/4MHz / (64*39062) = 0,125 sec 
  match_value = 312;
  //OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP & T1_PS_1_64 & 
  //  T1_SYNC_EXT_OFF & T1_SOURCE_INT, match_value);
  OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP & T1_PS_1_64 & 
              T1_SOURCE_INT, match_value);
  
  
  // 3906 -> 0,0125 sec 
  // 1953 -> 0,00625 sec
  // 312 -> 1ms
  // 156 -> 500us
  // 31 -> 100ns  
    
    
  //
  // Timer 2 irq
  // CAN Message Framerate 
  //
  ConfigIntTimer2(T2_INT_PRIOR_2 & T2_INT_OFF);
  WriteTimer2(0);
  // with clock set as 10MHz x8PLL 
  // (fosc*8/4)/(prescaler*match) -> 80/4MHz / (64*39062) = 0,125 sec
  // 3906 -> 0,0125 sec 
  // 1953 -> 0,00625 sec
  // 312 -> 1ms
  // 31 -> 100ns
  
  // saturate match_value to 0xFFFF in case of delays larger than 209
  if (BoardConfig.EE_CAN_MessageDataRate >= 209)
    match_value = 0xFFFF;
  else
    match_value = (312.5 * BoardConfig.EE_CAN_MessageDataRate);
  
  OpenTimer2(T2_ON & T2_GATE_OFF & T2_IDLE_STOP & T2_PS_1_64 & 
    T2_SOURCE_INT, match_value);

  //
  // Timer 3 irq
  // CAN Message Straingauges sampling
  //
  ConfigIntTimer3(T3_INT_PRIOR_7 & T3_INT_OFF);
  WriteTimer3(0);
  // with clock set as 10MHz x8PLL 
  // (fosc*8/4)/(prescaler*match) -> 80/4MHz / (64*39062) = 0,125 sec 
  // saturate match_value to 0xFFFF in case of delays larger than 2097
  if (BoardConfig.EE_AN_ChannelScanningTime >= 2097)
    match_value = 0xFFFF;
  else
    match_value = (31.25 * BoardConfig.EE_AN_ChannelScanningTime);
  
  OpenTimer3(T3_ON & T3_GATE_OFF & T3_IDLE_STOP & T3_PS_1_64 & 
    T3_SOURCE_INT, match_value);

 


  // flashled 
  
  for(i=1;i<=20;i++)
  {  
  // 
  // MAIS
  //
    ledY  = ~ledY; //toggLED  
    ledB  = ~ledB; //toggLED
    __delay32(800000);
  }

  // CAN Configuration

  // INIT CAN
  
  InitCanInterface();
  // Cofigure CAN filter
  SetBoardCanFilter();

  Chn=0;
  flag=0;
  EnableIntT1;	
  ledB=1;
  for(;;)
  {
//	ledB=0;   
    while(T1flag) 
    {
//	    ledB=1;
	    T1flag=0; //reset of the T1 timer flag
	    
	    // CAN command Parsing
      	ParseCommand(); 
      	idle();
      	if (toggledY>=((3-HESDATA_RESOLUTION)<<6))
      	{
      		toggledY=0;
    //     	ledY=~ledY;
      	}
	    
// reading of the 15 HES signals	    
    	for(i=0;i<15;i++)
	    {
		       // selsct analog channel 
			if(Chn>=11)
		    { 
			  ch = 11;
			  AMUXChannelSelect(Chn);
			}
			else
			ch = Chn;
		    SetChanADC12(ch & ADC_CH0_NEG_SAMPLEA_NVREF);
		    
		    ADCON1bits.DONE = 0; // 
		    __delay32(50);       //<--- togliendo si schanta
		
		    // start sampling
		    ADCON1bits.SAMP = 1;
		    __delay32(34);      // tempo minimo per il sample/hold
		
		    ConvertADC12();
		    while(!ADCON1bits.DONE);
		    BoardConfig.EE_AN_ChannelValue[Chn] =  ReadADC12(0);
		 
		    // increase channel
		    Chn++;
		    if(Chn==15)
		    {
			    Chn=0;
			    flag=0;
		    }  
		} 
//////////////////////////		
    }
  }
}
