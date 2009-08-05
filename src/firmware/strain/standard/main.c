//  A.d.C. M A I S / S T R A I N 
//  Author C.Lorini
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

#include <p30f4013.h>
#include <timer.h>
#include <can.h>
#include <adc12.h>
#include <spi.h>
#include <string.h>
#include <libpic30.h>
#include <dsp.h>
#include <reset.h>

#include "utils.h"
#include "errors.h"
#include "ext_adc_dac.h"
#include "int_adc.h"
#include "IIR_filter.h"
#include "eeprom.h"
#include "ports.h"
#include "ext_mux.h"
#include "can_interface.h"

// define the DUT MAIS or STRAIN
//#define MAIS
#define STRAIN

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

#define MAIS_VERSION       0x01
#define STRAIN_VERSION     0x01
#define MAIS_RELEASE       0x01
#define STRAIN_RELEASE     0x01

#define MAIS_BUILD         0x01
#define STRAIN_BUILD       0x01

#ifdef STRAIN 
  char VERSION=   		STRAIN_VERSION;
  char RELEASE=     	STRAIN_RELEASE;
  char BUILD=     		STRAIN_BUILD;
  char BOARD_TYPE= 		BOARD_TYPE_STRAIN;
#endif

//options (1/0)
char CAN_ACK_EVERY_MESSAGE=   1;

//board types
#define BOARD_TYPE_STRAIN  0x06
#define BOARD_TYPE_MAIS    0x07

// todo: inizializzazione a varie velocità

// inizializzazione bit di configurazione (p30f4013.h)
_FOSC(CSW_FSCM_OFF & ECIO_PLL8); 
  // Clock switching disabled Fail safe Clock Monitor disabled
  // External clock with PLL x8 (10MHz*8->Fcycle=80/4=20MIPS)
_FGS(CODE_PROT_OFF); // Code protection disabled

//_FWDT(WDT_OFF);     // WD disabled
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_2); // WD enabled 1:512*16

//_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_27);  // BOR 2.7V POR 64msec
_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_20);  // BOR 2.7V POR 64msec @@@ now 2.0V


//
// globals 
//
struct s_eeprom _EEDATA(1) ee_data = 
{
  0x0,           // EE_B_EEErased             :1
  0x0,           // EE_B_EnableWD             :1
  0x1,           // EE_B_EnableBOR            :1
  0x0F,          // EE_CAN_BoardAddress;      :8
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

//
//  variables

char filter_enable = 0;
char can_enable = 0;
char mux_enable = 1;
char muxed_chans =5;
void T2(void);



// dimensione del buffer di ingresso dei campioni analogici
// #define AN_SAMPLES_BUFF_DIM 256 
// extern fractional square1k[AN_SAMPLES_BUFF_DIM];
// fractional FilterOut; 


// info codes
// reset from BOR
#define INFO_TRSET_isBOR            9
#define INFO_TRSET_isPOR            10
#define INFO_TRSET_isLVD            11
#define INFO_TRSET_isMCLR           12
#define INFO_TRSET_isWDTTO          13
#define INFO_TRSET_isWDTWU          14
#define INFO_TRSET_isWU             15


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
#ifdef MAIS 
	get_sample_internal_adc();
#endif 
  IFS0bits.ADIF = 0;
}  


//
// TIMER 1 IRQ Service Routine
// used for Timestqamping and low frequency operation (LED ....)
//
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{    
#ifdef MAIS 
  // 
  // MAIS
  // 
  // LATFbits.LATF4  = ~LATFbits.LATF4; //toggLED  
  // LATFbits.LATF5  = ~LATFbits.LATF5;
#else 
  // 
  // STRAIN
  // 
  // LATBbits.LATB12 = ~LATBbits.LATB12; // tolggle led
#endif 
  // WriteTimer1(0x0);
  IFS0bits.T1IF = 0; // clear flag  
}

//
// TIMER 3 IRQ Service Routine
// used for strain gauges sampling
// and IIR Low pass filtering
// Durata totale 17 uSec 
char trasmission_counter=0;
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void)
{    
#ifdef MAIS 
  // 
  // MAIS
  //
  // ADC is performed in autonomous mode in ADCirq
#else 
  // 
  // STRAIN
  // 

  #define ADCS 4
  unsigned int adcs[ADCS];
  float adct=0;
  unsigned int adc=0;
  unsigned int i=0;
    
  // LATBbits.LATB12 = 1; // led on

  // channel active
  if (BoardConfig.EE_AN_SelectedChannel <= muxed_chans)
	 {
		if (filter_enable==0)
		{
				SetDACGetADC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[0], &adc);
				//	SetDACGetADC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[BoardConfig.EE_AN_SelectedChannel], &adc);
		}
		else
		{
				/*	    
				// This code is used to perform an avarage on multiple samples
				for (i=0; i<ADCS; i++)	SetDACGetADC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[BoardConfig.EE_AN_SelectedChannel], &adcs[i]);		
			    for (i=0; i<ADCS; i++)	adct+=adcs[i];
				adct=adct/ADCS;
				adc = (unsigned int) adct;
				*/
		}		

		// GetADC(&adc);
	
		// noise test
        // adc=adc>>3;
	    // adc=adc<<3;
	    // LOW-PASS DIGITAL FILTER
		adc-=0x7FFF;
/*		if (filter_enable)
	    {

			// IIR LPFilter durata circa 4uSec 
	    	IIRTransposed( 1, (fractional*) &BoardConfig.EE_AN_ChannelValue[BoardConfig.EE_AN_SelectedChannel], 
	      	(fractional*) &adc, &iirt[BoardConfig.EE_AN_SelectedChannel]);

		}

		else*/
		{
			BoardConfig.EE_AN_ChannelValue[BoardConfig.EE_AN_SelectedChannel] = adc;
		}
	//	BoardConfig.EE_AN_ChannelValue[BoardConfig.EE_AN_SelectedChannel]+=0x7FFF;
	}
	

  // select next channel 
  //if (BoardConfig.EE_AN_SelectedChannel == 5)
  if (BoardConfig.EE_AN_SelectedChannel >= muxed_chans+3)    BoardConfig.EE_AN_SelectedChannel=0;
  else  BoardConfig.EE_AN_SelectedChannel++;

  if (BoardConfig.EE_AN_SelectedChannel==0) LATBbits.LATB7=1; 
  //if (!mux_enable) BoardConfig.EE_AN_SelectedChannel=0;

 
  if (BoardConfig.EE_AN_SelectedChannel <= muxed_chans) AMUXChannelSelect(BoardConfig.EE_AN_SelectedChannel);
  // set DAC value for the next reading
  if (BoardConfig.EE_AN_SelectedChannel <= muxed_chans)
		SetDACGetADC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[BoardConfig.EE_AN_SelectedChannel], &adc);
		//  SetDAC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[BoardConfig.EE_AN_SelectedChannel]);
  
#endif
  // LATBbits.LATB12 = 0; // led off
  //@@@ 
  LATBbits.LATB7=0; 

 if (BoardConfig.EE_AN_SelectedChannel== muxed_chans+1)
/* if (BoardConfig.EE_AN_SelectedChannel== muxed_chans-2 ||
	 BoardConfig.EE_AN_SelectedChannel== muxed_chans+1)*/
	{
 // if (BoardConfig.EE_AN_SelectedChannel==0)
	 if (can_enable) T2();
    }
  IFS0bits.T3IF = 0; // clear flag 
}

//
// TIMER 2 IRQ Service Routine
// used for CAN messages transmission of Force/Torque
// durata totale 140uSec
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
{
IFS0bits.T2IF = 0; // clear flag  
}

void T2(void)
{    
  unsigned int SID; //adc;
  // LATBbits.LATB12 = 1; // led on


#ifdef MAIS 
  // 
  // MAIS
  //
  unsigned char HESData1[8], HESData2[8];
  static unsigned char Alt = 1; 

  if(Alt == 1)
  {
    memcpy(HESData1,&BoardConfig.EE_AN_ChannelValue[0],8);
    memcpy(HESData2,&BoardConfig.EE_AN_ChannelValue[4],8);
  }
  else
  {
    memcpy(HESData1,&BoardConfig.EE_AN_ChannelValue[8],8);
    memcpy(HESData2,&BoardConfig.EE_AN_ChannelValue[12],6);
  }

  // Load message ID , Data into transmit buffer and set transmit request bit
  // class, source, type for periodIc messages
  // HES data 
  if(Alt == 1)
  {
    SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES0TO3) ;
    CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData1,8,0); // buffer 0 
    // HES data 
    SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES4TO7) ;
    CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData2,8,1); // buffer 1 
    Alt = 2;
  }
  else
  {
    SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES8TO11) ;
    CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData1,8,0); // buffer 0 
    // HES data 
    SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_HES12TO14) ;
    CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, HESData2,6,1); // buffer 1 
    Alt = 1;
  }
  
  // Wait till message is transmitted completely
  //  while(!CAN1IsTXReady(0)) 
  //    ;
   
#else 
  // 
  // STRAIN
  // 
  unsigned char ForceData[6], TorqueData[6]; 
  static unsigned char ChToTransmit=1; 
 
  // Tim1IRQ has to be disabled: MatrixMultiply and IIRTransposed 
  // from dsp library shares some common resources
//@@@
  //DisableIntT3;

  // 15usec 
  MatrixMultiply(
  6, // int numRows1,
  6, // int numCols1Rows2,
  1, // int numCols2,
  &BoardConfig.EE_TF_TorqueValue[0],   // fractional* dstM,
  &BoardConfig.EE_TF_TMatrix[0][0],    // fractional* srcM1,
  (int*) &BoardConfig.EE_AN_ChannelValue[0]); // fractional* srcM2 
  //@@@
  //EnableIntT3;

// send 6 analog strain gauges samples
// #define CAN_SEND_SG_VALUES
// send torque/force data
#define CAN_SEND_FT_VALUES

#ifdef CAN_SEND_SG_VALUES
// send filtered/unfiltered SG samples
  BoardConfig.EE_AN_ChannelValue[0]+=0x7FFF;
  BoardConfig.EE_AN_ChannelValue[1]+=0x7FFF;
  BoardConfig.EE_AN_ChannelValue[2]+=0x7FFF;
  BoardConfig.EE_AN_ChannelValue[3]+=0x7FFF;
  BoardConfig.EE_AN_ChannelValue[4]+=0x7FFF;
  BoardConfig.EE_AN_ChannelValue[5]+=0x7FFF;
  memcpy(ForceData,BoardConfig.EE_AN_ChannelValue,6);
  memcpy(TorqueData,&BoardConfig.EE_AN_ChannelValue[3],6);
#endif

#ifdef CAN_SEND_FT_VALUES
// send multiplied samples
  BoardConfig.EE_TF_TorqueValue[0]+=0x7FFF;
  BoardConfig.EE_TF_TorqueValue[1]+=0x7FFF;
  BoardConfig.EE_TF_TorqueValue[2]+=0x7FFF;
  BoardConfig.EE_TF_TorqueValue[3]+=0x7FFF;
  BoardConfig.EE_TF_TorqueValue[4]+=0x7FFF;
  BoardConfig.EE_TF_TorqueValue[5]+=0x7FFF;
  memcpy(ForceData,BoardConfig.EE_TF_TorqueValue,6);
  memcpy(TorqueData,BoardConfig.EE_TF_ForceValue,6);
#endif

  // memcpy(ForceData,BoardConfig.EE_AN_ChannelValue,6);
  // memcpy(TorqueData,&BoardConfig.EE_AN_ChannelValue[3],6);

  // Load message ID , Data into transmit buffer and set transmit request bit
  // class, source, type for periodoc messages
  // force data 

  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_FORCE_VECTOR) ;
  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, ForceData,6,0); // buffer 0 
  // torque data 
  SID = (CAN_MSG_CLASS_PERIODIC) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_TORQUE_VECTOR) ;
  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, TorqueData,6,1); // buffer 1 
  
  // Wait till message is transmitted completely
  //  while(!CAN1IsTXReady(0)) 
  //    ;
  
  // Next channel
  if (ChToTransmit == 5)
    ChToTransmit=0;
  else
    ChToTransmit++;

#endif

  // LATBbits.LATB12 = 0; // led off

  // WriteTimer2(0x0);
  //IFS0bits.T2IF = 0; // clear flag  
}


void ParseCommand_local(canmsg_t *msg) //, unsigned int SID)
//
// parse board test messages and renspond to the sender 
// response has SID and a variable payload different freom command 2 another
// source and dest are swapped from incoming to outgoing messages
//
// polling messages
//  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     |                |
// | class  | Source | Dest   |      ....      |
//  -------  -------  -------  -------  ------- 
// recursive messages
//  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     |      B[0-7]    |
// |class	| Source | Type   |     Payload    |
//  -------  -------  -------  -------  ------- 
// 
{
  unsigned char Txdata[9]; 
  char datalen;
  //char tx_rx_no;
  unsigned int i,match_value;
  unsigned int j,tmp,tout;

  //tx_rx_no=0;
  switch (msg->CAN_Per_Msg_Class)
  {
	case (CAN_MSG_CLASS_POLLING>>8):
	{
	  switch (msg->CAN_Per_Msg_PayLoad[0])
	  {
	    case CAN_CMD_SET_BOARD_ADX:
	      // set board CAN address 
	      if ( ( msg->CAN_Per_Msg_PayLoad[1] > 0 ) && ( msg->CAN_Per_Msg_PayLoad[1] <= 15 ))
	      {
	        BoardConfig.EE_CAN_BoardAddress = msg->CAN_Per_Msg_PayLoad[1];
			SetBoardCanFilter();	   
			if (SAVE_EEPROM_ATONCE)
			{
			    SaveEepromBoardConfig();
			    SaveEepromIIRFilter();
			}
	      }
	      else 
	      {
	        EEnqueue(ERR_CAN_PARAMETERS_INVALID);
	        return; 
	      } 
	      datalen=0;
	    break;
	
	    case CAN_CMD_SET_IIR:
	      // Set IIR Filter parameters: 0x205 len 4  data 1 i MSB LSB 
	      // N. BQ, <- number of biquads
	      // b0[s], b1[s], a1[s], b2[s], a2[s], <- 1stBQ
	      // b0[s], b1[s], a1[s], b2[s], a2[s], <- 2ndBQ 
	      // b0[s], b1[s], a1[s], b2[s], a2[s], <- 3rdBQ
	      // b0[s], b1[s], a1[s], b2[s], a2[s], <- 4thBQ
	      // b0[s], b1[s], a1[s], b2[s], a2[s], <- 5thBQ
	
	#ifdef MAIS
	      // 
	      // MAIS
	      //
	      EEnqueue(ERR_CAN_COMMAND_UNAVAILABLE);
	    return;
	#else 
	      //
	      // STRAIN
	      //
	    {
	      int index,dat;
	
	      index = msg->CAN_Per_Msg_PayLoad[1];             // indice del dato del filtro
	      dat = msg->CAN_Per_Msg_PayLoad[2]<<8 | msg->CAN_Per_Msg_PayLoad[3];  // valore del dato
	
	      if (index==0)
	      // il dato indica il n.di BiQuads
	      {
	         if(dat > IIR_LPF_N_MAX_BQ)
	         // se il numenro di BQ e` superiore a 5 
	         {
	           EEnqueue(ERR_CAN_IIR_NBQ2BIG);
	           return;
	         }
	         else
	           IirTransposedCoefs.IIR_N_BQ = dat; 
	      }
	      if (index > IirTransposedCoefs.IIR_N_BQ * 5 )
	      // se indirizzo un coeficiente oltre alla dimensione del filtro
	      {
	        EEnqueue(ERR_CAN_IIR_COEF_INDEXING);
	        return;
	      }
	      else 
	        IirTransposedCoefs.IirTransposedCoefs[index-1] = dat;
	    
	      IIRFiltersINIT();
	   }
	#endif
	      datalen=0;
	    break;

		case CAN_CMD_GET_MATRIX_RC:
		{
	        if(msg->CAN_Per_Msg_PayLoad[1] < 6)
	        {
	          if(msg->CAN_Per_Msg_PayLoad[2] < 6)
	          {
				Txdata[0] = CAN_CMD_GET_MATRIX_RC; 
				Txdata[1] = msg->CAN_Per_Msg_PayLoad[1]; 
				Txdata[2] = msg->CAN_Per_Msg_PayLoad[2]; 
				Txdata[3] = BoardConfig.EE_TF_TMatrix[msg->CAN_Per_Msg_PayLoad[1]][msg->CAN_Per_Msg_PayLoad[2]] >> 8; 
				Txdata[4] = BoardConfig.EE_TF_TMatrix[msg->CAN_Per_Msg_PayLoad[1]][msg->CAN_Per_Msg_PayLoad[2]] & 0xFF;  
				datalen=5;	            
	          }
	        }
		}
		break;

		case CAN_CMD_GET_CH_DAC:
		{
          if(msg->CAN_Per_Msg_PayLoad[1] < 6)
          {
			Txdata[0] = CAN_CMD_GET_CH_DAC; 
			Txdata[1] = msg->CAN_Per_Msg_PayLoad[1];  
			Txdata[2] = BoardConfig.EE_AN_ChannelOffset[msg->CAN_Per_Msg_PayLoad[1]] >> 8; 
			Txdata[3] = BoardConfig.EE_AN_ChannelOffset[msg->CAN_Per_Msg_PayLoad[1]] & 0xFF; 
			datalen=4;	            
          }
		}
		break;

		case CAN_CMD_GET_CH_ADC:
		{
          if(msg->CAN_Per_Msg_PayLoad[1] < 6)
          {
			Txdata[0] = CAN_CMD_GET_CH_ADC; 
			Txdata[1] = msg->CAN_Per_Msg_PayLoad[1];  
			Txdata[2] = (BoardConfig.EE_AN_ChannelValue[msg->CAN_Per_Msg_PayLoad[1]]+0x7FFF) >> 8; 
			Txdata[3] = (BoardConfig.EE_AN_ChannelValue[msg->CAN_Per_Msg_PayLoad[1]]+0x7FFF) & 0xFF; 
			datalen=4;	            
          }
		}
		break;
	
	    case CAN_CMD_SET_MATRIX_RC:
	      //  set i,j value of transform. matrix:
	      //  0x205 len 5  data 3 i j vv vv 
	#ifdef MAIS
	  // 
	  // MAIS
	  // 
	  EEnqueue(ERR_CAN_COMMAND_UNAVAILABLE);
	  return;
	#else 
	  //
	  // STRAIN
	  //
	      if(msg->CAN_Per_Msg_PayLoad[1] < 6)
	      {
	        if(msg->CAN_Per_Msg_PayLoad[2] < 6)
	        {
	          BoardConfig.EE_TF_TMatrix[msg->CAN_Per_Msg_PayLoad[1]][msg->CAN_Per_Msg_PayLoad[2]] = msg->CAN_Per_Msg_PayLoad[3]<<8 | msg->CAN_Per_Msg_PayLoad[4];
	        }
	      }
	      else 
	      {
	        EEnqueue(ERR_CAN_MATRIX_INDEXING);
	        return;
	      }
	#endif
	      datalen=0;
	    break;

	    case CAN_CMD_SET_CH_DAC:
	      //  set DAC value 0x205 len 4  data 4 ch msb lsb
	#ifdef MAIS
	  // 
	  // MAIS
	  // 
	  EEnqueue(ERR_CAN_COMMAND_UNAVAILABLE);
	  return;
	#else 
	  //
	  // STRAIN
	  //
	      i = msg->CAN_Per_Msg_PayLoad[2]<<8 | msg->CAN_Per_Msg_PayLoad[3];
	      j = msg->CAN_Per_Msg_PayLoad[1];
	      if ( (j >= 0) && (j <= 5 ))
	      {
	        //SetDACGetADC(PDM_NORMAL, i, &tmp);   //if not commented offset becomes effective immediately
	        BoardConfig.EE_AN_ChannelOffset[j]=i;
	      }
	      else 
	      {
	        EEnqueue(ERR_CAN_PARAMETERS_INVALID);
	        return; 
	     }
	#endif
	      datalen=0;
	    break;
	
	    case CAN_CMD_SET_TXMODE:
	    // set continuous or on demand tx  0x205 len 2  data 7 0/1
	      if(msg->CAN_Per_Msg_PayLoad[1]==0)
	      { 
			can_enable=1;
	        EnableIntT2;
	        EnableIntT3;
	        // ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_ON);
	      }
	      else if (msg->CAN_Per_Msg_PayLoad[1]==1)
	      {
			can_enable=0;
			DisableIntT2; 
			EnableIntT3;
	        // ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_OFF);
	      }
	      else 
	      {
			can_enable=0;
	        DisableIntT2;
	        DisableIntT3;
	        // ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_OFF);
	      }
	      datalen=0;
	    break;

	    case CAN_CMD_FILTER_EN:
	    // set continuous or on demand tx  0x205 len 2  data 7 0/1
	      if(msg->CAN_Per_Msg_PayLoad[1]==0)
	      { 
	        filter_enable=0;
	      }
		  else
	      {
	        filter_enable=1;
	      }
	      datalen=0;
	    break;

	    case CAN_CMD_MUX_EN:
	    // set continuous or on demand tx  0x205 len 2  data 7 0/1
	      if(msg->CAN_Per_Msg_PayLoad[1]==0)
	      { 
	        mux_enable=0;
			BoardConfig.EE_AN_SelectedChannel=0;
		    AMUXChannelSelect(BoardConfig.EE_AN_SelectedChannel);
			SetDAC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[BoardConfig.EE_AN_SelectedChannel]);
	      }
		  else
	      {
	        mux_enable=1;
			BoardConfig.EE_AN_SelectedChannel=0;
			AMUXChannelSelect(BoardConfig.EE_AN_SelectedChannel);
			SetDAC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[BoardConfig.EE_AN_SelectedChannel]);
	      }
	      datalen=0;
	    break;

	    case CAN_CMD_MUX_NUM:
	    // set continuous or on demand tx  0x205 len 2  data 7 0/1
	      muxed_chans=msg->CAN_Per_Msg_PayLoad[1];
	      datalen=0;
	    break;

	
	    case CAN_CMD_SET_CANDATARATE:
	    // set datarate for transmission in milliseconds 
	    // 0x205 len 2  data 8 n
	      BoardConfig.EE_CAN_MessageDataRate=msg->CAN_Per_Msg_PayLoad[1];
	      datalen=0;
	 
	      ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_OFF);
	      WriteTimer2(0);
	  
	      // saturate match_value to 0xFFFF in case of delays larger than 209
	      if (BoardConfig.EE_CAN_MessageDataRate >= 209)
	        match_value = 0xFFFF;
	      else
	        match_value = ( 312.5 * BoardConfig.EE_CAN_MessageDataRate);
	  
	      OpenTimer2(T2_ON & T2_GATE_OFF & T2_IDLE_STOP & T2_PS_1_64 & 
	        T2_SOURCE_INT, match_value);
	      
	      ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_ON);
	    break;
	
	    case CAN_CMD_SELECT_ACTIVE_CH:
	    // select witch channel is sampled and CANsmitted
	    // 0x205 len 2  data 5 0bxx000001
	#ifdef MAIS
	  // 
	  // MAIS
	  // 
	#else 
	  //
	  // STRAIN
	  //
	      if( msg->CAN_Per_Msg_PayLoad[1] & 0x1 )
	        BoardConfig.EE_AN_ActiveChannels[0] = 1;
	      else
	        BoardConfig.EE_AN_ActiveChannels[0] = 0;
	
	      if( msg->CAN_Per_Msg_PayLoad[1] & 0x2 )
	        BoardConfig.EE_AN_ActiveChannels[1] = 1;
	      else
	        BoardConfig.EE_AN_ActiveChannels[1] = 0;
	 
	      if( msg->CAN_Per_Msg_PayLoad[1] & 0x4 )
	        BoardConfig.EE_AN_ActiveChannels[2] = 1;
	      else
	        BoardConfig.EE_AN_ActiveChannels[2] = 0;
	
	      if( msg->CAN_Per_Msg_PayLoad[1] & 0x8 )
	        BoardConfig.EE_AN_ActiveChannels[3] = 1;
	      else
	        BoardConfig.EE_AN_ActiveChannels[3] = 0;
	
	      if( msg->CAN_Per_Msg_PayLoad[1] & 0x10 )
	        BoardConfig.EE_AN_ActiveChannels[4] = 1;
	      else
	        BoardConfig.EE_AN_ActiveChannels[4] = 0;
	
	      if( msg->CAN_Per_Msg_PayLoad[1] & 0x20 )
	        BoardConfig.EE_AN_ActiveChannels[5] = 1;
	      else
	        BoardConfig.EE_AN_ActiveChannels[5] = 0;
	#endif        
	      datalen=0;      
	    break;
	
	    case CAN_CMD_SAVE2EE:
	    // Save configuration data to EE
	    // 0x205 len 1  data 9
	    {
		  SaveEepromBoardConfig();
	 	  SaveEepromIIRFilter();

	      // todo: checksum calcuation
	 
	      datalen=0;
	    }
	    break;	

	    //TODO: aggiungere comando get can errors

		default:
		{
		// UNKNOWN COMMAND FOR THIS CLASS
		SendCanProblem();
		}
		break;
   	  }   
	}
	break;

	//CANLOADER CLASS MESSAGES
	case (CAN_MSG_CLASS_LOADER>>8):
	{
		DisableIntT2; 
 		DisableIntT3;

		switch (msg->CAN_Per_Msg_PayLoad[0])
		{
			case CMD_BROADCAST:
			{
   				//Create ID for CAN message
      			i = CAN_MSG_CLASS_LOADER | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | (0);
     	        Txdata[0] = CMD_BROADCAST;
		        Txdata[1] = BOARD_TYPE; 
			    Txdata[2] = VERSION;            //Firmware version number for BOOTLOADER
      			Txdata[3] = RELEASE;              //Firmware build number.
      			Txdata[4] = BUILD;              //Firmware build number.
      			datalen=5; 
			}
			break;

			case CMD_BOARD:
			{
   				//Jump to bootlader code
				asm ("reset");
			}
			break;

			case CMD_GET_ADDITIONAL_INFO:
			{
				i = CAN_MSG_CLASS_LOADER | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | (0);
				Txdata[0] = 0x0C; 
				Txdata[1] = 0x00; 
				datalen=6;
			    for (tmp = 0; tmp < 8; tmp++)
			    {
					do 
					{
						tout++;
					}
					while (!CAN1IsTXReady(0) && tout<=2000) ;
					if (tout!=2000 )
					{
						Txdata[1]=tmp;
						for (j=0; j<4; j++)	Txdata[2+j] = BoardConfig.EE_AdditionalInfo[j+tmp*4]; 		
						CAN1SendMessage((CAN_TX_SID(i)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, Txdata,datalen,0); 
					} 	
			    }
				datalen = -1;
			}
			break;

			case CMD_SET_ADDITIONAL_INFO:
			{
				static unsigned char addinfo_part=0;
				//if (msg->CAN_Per_Msg_Length == 6) 
				{ 			
					for (j=0; j<4; j++)
						{
							BoardConfig.EE_AdditionalInfo[msg->CAN_Per_Msg_PayLoad[1]*4+j] = msg->CAN_Per_Msg_PayLoad[2+j]; 
						}

				}
				if (SAVE_EEPROM_ATONCE)
				{
					if (addinfo_part==7)
					{
	 					addinfo_part=0;
					    SaveEepromBoardConfig();
					    SaveEepromIIRFilter();
					}
					else
					{					
						addinfo_part++;
					}
					datalen = 0;
				}
			}
			break;

			case CMD_ADDRESS: 
			case CMD_DATA: 
			case CMD_START: 
			case CMD_END: 
			{
				// IGNORE THESE COMMANDS
				datalen = -1;
			}
			break;

			default:
			{
				// UNKNOWN COMMAND FOR THIS CLASS
				SendCanProblem();
			}
			break;
		}
	}
	break;

	//UNKNOWN CLASS MESSAGES
	default:
	{
		// UNKNOWN COMMAND FOR THIS CLASS
		SendCanProblem();
	}
	break;
  }

  // Load message ID , Data into transmit buffer and set transmit request bit
  // swap source and destination
  i = (msg->CAN_Per_Msg_Class << 8 ) | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | ( msg->CAN_Poll_Msg_Source );

  // Send ack messages with datalen > 0
  // Skip ack messages with datalen < 0
  // Send ack messages with datalen = 0 and CAN_ACK_EVERY_MESSAGE = 1
  if ( (datalen > 0) || (datalen == 0 && CAN_ACK_EVERY_MESSAGE == 1) )
  { 
    // Wait till previous message is transmitted completely
    while(!CAN1IsTXReady(0));
    CAN1SendMessage((CAN_TX_SID(i)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, Txdata,datalen,0); 
  }
}


int main(void)
{ 
  unsigned int match_value;
  unsigned int i;
  canmsg_t CAN_Msg;

  //
  // init code
  //
#ifdef MAIS
  	InitMaisIoPorts();
#else 
	InitStrainIoPorts();

  	// Select analog channel 1 
 	AMUXChannelSelect(1);

	// Init SPI for communication with ext ADC/DAC
	InitSPI();

#endif

  // Reset   
  // Enqueue of reset cause
  if(isBOR!=0)    EEnqueue(INFO_TRSET_isBOR);
  if(isPOR!=0)    EEnqueue(INFO_TRSET_isPOR);
  if(isLVD!=0)    EEnqueue(INFO_TRSET_isLVD);
  if(isMCLR!=0)   EEnqueue(INFO_TRSET_isMCLR);
  if(isWDTTO!=0)  EEnqueue(INFO_TRSET_isWDTTO);
  if(isWDTWU!=0)  EEnqueue(INFO_TRSET_isWDTWU);
  if(isWU!=0)     EEnqueue(INFO_TRSET_isWU);
  
  //
  // EEPROM Data Recovery
  // 

  // Initialize BoardConfig variable in RAM with the Data EEPROM stored values 
  RecoverConfigurationFromEEprom();
  // IIR filter coefs
  RecoverIIRFilterFromEEprom();
  // IIR Transposed filters init
  IIRFiltersINIT();

  //
  // Timer 1 irq
  // Low frequency operation
  //
  ConfigIntTimer1(T1_INT_PRIOR_0 & T1_INT_ON);
  WriteTimer1(0);
  // with clock set as 10MHz x8PLL 
  // (fosc*8/4)/(prescaler*match) -> 80/4MHz / (64*39062) = 0,125 msec 
  match_value = 39062;
  OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP & T1_PS_1_64 & 
    T1_SYNC_EXT_OFF & T1_SOURCE_INT, match_value);

  //
  // Timer 2 irq
  // CAN Message Framerate 
  //
  ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_OFF);
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
  
  // saturate match_value to 0xFFFF in case of delays larger than 209
  if (BoardConfig.EE_AN_ChannelScanningTime >= 2097)
    match_value = 0xFFFF;
  else
    match_value = (31.25 * BoardConfig.EE_AN_ChannelScanningTime);
  
  OpenTimer3(T3_ON & T3_GATE_OFF & T3_IDLE_STOP & T3_PS_1_64 & 
    T3_SOURCE_INT, match_value);

 // ConfigIntTimer3(T3_INT_PRIOR_7 & T3_INT_ON);

  //
  // ADC12 used only for MAIS Boards
  // HES sampling
  //
#ifdef MAIS
  init_internal_adc();
#endif

  // 
  // Flash Leds
  //  
  for(i=1;i<=20;i++)
  {  
#ifdef MAIS
    LATFbits.LATF4  = ~LATFbits.LATF4; //toggLED  
    LATFbits.LATF5  = ~LATFbits.LATF5;
#else 
    LATBbits.LATB12 = ~LATBbits.LATB12;
#endif
    __delay32(800000);
  }

  // CAN Configuration
  InitCanInterface();
  // Cofigure CAN filter
  SetBoardCanFilter();

  unsigned int SID = (CAN_MSG_CLASS_POLLING) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (0) ;
  unsigned char TorqueData[6]={0,0,0,0,0,0}; 

 if(RCONbits.POR!=0)   TorqueData[0]=1;
 if(RCONbits.BOR!=0)   TorqueData[1]=1;
 if(RCONbits.IDLE!=0)   TorqueData[2]=1;
 if(RCONbits.LVDL!=0)  TorqueData[3]=1;
 if(RCONbits.LVDEN!=0) TorqueData[4]=1;
 if(RCONbits.WDTO!=0) TorqueData[5]=1;
 if(RCONbits.EXTR!=0)    TorqueData[6]=1;
RCON=0;
RCONbits.POR=0;
RCONbits.BOR=0;
RCONbits.IDLE=0;
RCONbits.SLEEP=0;
RCONbits.WDTO=0;
RCONbits.SWDTEN=0;
RCONbits.SWR=0;
RCONbits.EXTR=0;
RCONbits.LVDL =0;
RCONbits.LVDEN=0;
RCONbits.BGST=0;
RCONbits.IOPUWR=0;
RCONbits.TRAPR=0;

  CAN1SendMessage((CAN_TX_SID(SID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, TorqueData,6,1); // buffer 1 

  for(;;)
  {
	ParseCommand();
/*  if (CAN_Messages_buff_ptr != 0)
    {
	  // Something in the CAN message buffer
      //mask CANRxIRQ for atomic buffer manipulation
      DisableIntCAN1;
      memcpy(&CAN_Msg,&CAN_Messages[CAN_Messages_buff_ptr-1],sizeof(CAN_Msg));
      CAN_Messages_buff_ptr--;
      EnableIntCAN1;
      ParseCommand_local(&CAN_Msg); //&CAN_Messages[CAN_Messages_buff_ptr-1].CAN_Per_Msg_PayLoad[0], CAN_Messages[CAN_Messages_buff_ptr-1].CAN_Poll_Msg_Dest);
    }
    else 
      idle();
*/
  }
}
