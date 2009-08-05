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

// define the DUT MAIS or STRAIN
//#define MAIS
#define STRAIN

//board types
#define BOARD_TYPE_STRAIN  0x06
#define BOARD_TYPE_MAIS    0x07

#define MAIS_VERSION       0x01
#define STRAIN_VERSION     0x01
#define MAIS_RELEASE       0x01
#define STRAIN_RELEASE     0x01

#define MAIS_BUILD         0x01
#define STRAIN_BUILD       0x01

#ifdef MAIS 
  #define VERSION   	MAIS_VERSION
  #define RELEASE     	MAIS_RELEASE
  #define BUILD     	MAIS_BUILD
  #define BOARD_TYPE 	BOARD_TYPE_MAIS
#else 
  #define VERSION    	STRAIN_VERSION
  #define RELEASE     	STRAIN_RELEASE
  #define BUILD      	STRAIN_BUILD
  #define BOARD_TYPE 	BOARD_TYPE_STRAIN
#endif 

//options (1/0)
#define SAVE_EEPROM_ATONCE    1
#define CAN_ACK_EVERY_MESSAGE 1

//board types
#define BOARD_TYPE_STRAIN  0x06
#define BOARD_TYPE_MAIS    0x07

// DAC Power Down Modes
#define PDM_NORMAL     0xCFFF
#define PDM_1KTOGND    0xDFFF
#define PDM_100KTOGND  0xEFFF
#define PDM_THREESTATE 0xFFFF

// 
// todo: inizializzazione a varie velocità

// Frame formats
#define CAN_STANDARD_FORMAT 0
#define CAN_EXTENDED_FORMAT 1

// Frame types
#define CAN_DATA_FRAME      0
#define CAN_REMOTE_FRAME    1

#define CAN_MSG_CLASS_PERIODIC 0x0300
// For messages of class 011 the meaning of data/ID is defined as follows:
//  -------------------------- ----------------
// |           11b            |        8B      |
//  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     |      B[0-7]    |
// |class	| Source | Type   |     Payload    |
//  -------  -------  -------  -------  ------- 

#define CAN_MSG_CLASS_POLLING 0x0200
// For messages of class 010 the meaning of data/ID is defined as follows:
//  -------------------------- -------------------------
// |           11b            |           8B            |
//  -------  -------  -------  -------  -------  ------- 
// | 3b     | 4b     | 4b     | B[0]   |     B[1-7]     |
// |class   | Source | Dest   | C type |    Payload     |
//  -------  -------  -------  -------  -------  ------- 

#define CAN_MSG_CLASS_LOADER   0x0700

#define CAN_TX_SOFTWARE_BUFFER_SIZE 10

//////////////////////////////////////////////////////////////
// CANLOADER MESSAGES, CLASS 0x02

// Incoming messages


#define CAN_CMD_NONE              0x0

// Set board CAN address
#define CAN_CMD_SET_BOARD_ADX     0x32
// 010 SOURCE DEST 0x0 A

// select IIR filters parameters 
#define CAN_CMD_SET_IIR           0x1
// 010 SOURCE DEST 0x1 n VV

// Set SG to TF trasformation matrix  
#define CAN_CMD_SET_MATRIX_RC     0x3
// 010 SOURCE DEST 0x3 R C VV VV

// set DAC for channel x
#define CAN_CMD_SET_CH_DAC        0x4
// 010 SOURCE DEST 0x4 CH DACVALUE

// set active channels (activation mask) only active channels are transmitted
#define CAN_CMD_SELECT_ACTIVE_CH  0x5
// 010 SOURCE DEST 0x5 0bxx000000

// 
#define CAN_CMD_CALIBRATE_OFFSET  0x6
// 

// set continuous/on demand transmission mode
#define CAN_CMD_SET_TXMODE        0x7
// 010 SOURCE DEST 0x7 0/1

// set board CAN speed in milliseconds minimum, datarate 210ms  
#define CAN_CMD_SET_CANDATARATE   0x8
// 010 SOURCE DEST 0x8 SPEED

// save Config to EE
#define CAN_CMD_SAVE2EE           0x9
// 010 SOURCE DEST 0x9

// Get TF trasformation matrix  
#define CAN_CMD_GET_MATRIX_RC     0xA
// 

// Get DAC for channel x
#define CAN_CMD_GET_CH_DAC        0xB
// 

// Get ADC for channel x
#define CAN_CMD_GET_CH_ADC        0xC
// 

// ENABLE/DISABLES FILTER
#define CAN_CMD_FILTER_EN         0xD
// 

// ENABLE/DISABLES MUX
#define CAN_CMD_MUX_EN         	  0xE

#define CAN_CMD_MUX_NUM           0xF

// TODO: definire un numero disponibile in iCUB 
// il protocollo di icub prevede nelle risposte alla prima posizione un comando nullo (0x00) e quindi i dati...
//

//////////////////////////////////////////////////////////////
// BROADCAST MESSAGES, CLASS 0x03

// Transmitted Torque values 3*16 bit 
#define CAN_CMD_FORCE_VECTOR      0xA
// 010 SOURCE DEST 0xA t1 t1 t2 t2 t3 t3

#define CAN_CMD_TORQUE_VECTOR     0xB
// 010 SOURCE DEST 0xB f1 f1 f2 f2 f3 f3

// hall effect sensors from 0 to 3
#define CAN_CMD_HES0TO3           0xC
// 010 SOURCE DEST 0xC h0 h0 h1 h1 h2 h2 h3 h3

// hall effect sensors from 4 to 7
#define CAN_CMD_HES4TO7           0xD
// 010 SOURCE DEST 0xC h4 h4 h5 h5 h6 h6 h7 h7

// hall effect sensors from 8 to 11
#define CAN_CMD_HES8TO11          0xE
// 010 SOURCE DEST 0xC h8 h8 h9 h9 h10 h10 h11 h11

// hall effect sensors from 12 to 14
#define CAN_CMD_HES12TO14         0xF
// 010 SOURCE DEST 0xC h12 h12 h13 h13 h14 h14 h15 h15

//////////////////////////////////////////////////////////////
// CANLOADER MESSAGES, CLASS 0x07

//Jump to CAN loader code. Replay with 0x00, 0x01
#define CMD_BOARD                 0x00

//Address packet 
//Data[1] is the length
//Data[2], Data[3] are the address
//Data[4], is the block type (0x00 = program, 0x01 = data)
#define CMD_ADDRESS               0x01

//Start the program ????? Replay with 0x02, 0x01
#define CMD_START                 0x02

//Data packet: 6 bytes of payload are flashed to memory. Replay with 0x03, 0x01
#define CMD_DATA                  0x03

//The program is terminated. Replay with 0x04, 0x01 ????
#define CMD_END                   0x04

//Command not used
#define CMD_ERROR                 0x05

//Additional Info
#define CMD_GET_ADDITIONAL_INFO		12
#define CMD_SET_ADDITIONAL_INFO		13

// Request for board type and firmware version
#define CMD_BROADCAST             0xFF

// can bus message structure
typedef struct canmsg_tag
{
  union
  {
    struct // for polling messages
    {
      unsigned CAN_Poll_RxIDE          :1;
      unsigned CAN_Poll_SRR            :1;
      unsigned CAN_Poll_Msg_Dest   :4;  // Destination
      unsigned CAN_Poll_Msg_Source :4;  // Source
      unsigned CAN_Poll_Msg_Class  :3;  // Class of message
      unsigned Poll_NotUsed        :3;
      unsigned char CAN_Poll_Msg_CType; // C+Type
      unsigned char CAN_Poll_Msg_PayLoad[7]; // Payload 
    };
    struct // for periodic messages
    {
      unsigned CAN_Per_RxIDE               :1;
      unsigned CAN_Per_SRR                 :1;
      unsigned CAN_Per_Msg_Type        :4;  // Destination
      unsigned CAN_Per_Msg_Source      :4;  // Source
      unsigned CAN_Per_Msg_Class       :3;  // Class of message
      unsigned Per_NotUsed             :3; 
      unsigned char CAN_Per_Msg_PayLoad[8]; // Payload 
    };
    
    //TODO: verificare se è sbagliato  CAN_Msg_AsBytes[9]. Facendo le somme dei precedenti sembra essere CAN_Msg_AsBytes[10]
    unsigned short CAN_Msg_AsBytes[9];   // Message data formatted as bytes 
  };
} canmsg_t;

// define the DUT MAIS or STRAIN
// #define MAIS
#define STRAIN

// inizializzazione bit di configurazione (p30f4013.h)
_FOSC(CSW_FSCM_OFF & ECIO_PLL8); 
  // Clock switching disabled Fail safe Clock Monitor disabled
  // External clock with PLL x8 (10MHz*8->Fcycle=80/4=20MIPS)
_FGS(CODE_PROT_OFF); // Code protection disabled

_FWDT(WDT_OFF);     // WD disabled
//_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_2); // WD enabled 1:512*16

//_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_27);  // BOR 2.7V POR 64msec
//_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_20);  // BOR 2.7V POR 64msec @@@ now 2.0V
_FBORPOR(MCLR_EN);

//TODO: Error queue / error flag set/reset
//TODO: Erroristica CAN 

#define IIR_LPF_N_MAX_BQ 5  // Numero massimo di BiQuads dell'IIR LPF 

//
// EEPROM definitions
// 
                                                                           
//
// EEPROM memorized data for board configuration
//
typedef struct s_eeprom
{
  // configrazioni relative alla scheda
  unsigned EE_B_EEErased  :1;  // if 1 the ee has been erased (hopefully ;-)
  unsigned EE_B_EnableWD  :1;
  unsigned EE_B_EnableBOR :1;
  
  // configurazioni relative al CAN
  unsigned char EE_CAN_BoardAddress;  
  unsigned char EE_CAN_MessageDataRate;    // framerate of outgoing messages
  unsigned char EE_CAN_Speed;
  
  // configurazioni relative ai canali analogici
  unsigned char EE_AN_ActiveChannels[13];  // sequenza di acquisizione canali 0 se canale non usato
  unsigned char EE_AN_SelectedChannel;     // canale attualmente attivo
  unsigned int  EE_AN_ChannelScanningTime; // tempo di scansione dei canali in 100aia di usec
  unsigned int  EE_AN_ChannelOffset[6];    // DAC generated offset
  unsigned int  EE_AN_ChannelValue[13];    // ADC values

  // torque/force
  int  EE_TF_TorqueValue[3];      // Torque values
  int  EE_TF_ForceValue[3];       // Force values

  int           EE_TF_TMatrix[6][6];       // SG2TF Transformation Matrix

  // additional info
  unsigned char EE_AdditionalInfo[32]; 

  unsigned int  EE_ChkSum;                 // data validation checksum

} s_eeprom;

//
// EEPROM memorized IIR filter coefs 
// has to be separated from s_eeprom var 'couse inits data in xmem
typedef struct s_eeIIRTransposedCoefs
{
  int IIR_N_BQ;
  fractional IirTransposedCoefs[5*IIR_LPF_N_MAX_BQ];   
} s_eeIIRTransposedCoefs;

// EEDATA *must* be declared GLOBAL
// EEProm data structures init
static struct s_eeprom _EEDATA(1) ee_data = 
{
  0x0,           // EE_B_EEErased             :1
  0x1,           // EE_B_EnableWD             :1
  0x1,           // EE_B_EnableBOR            :1
  0x05,          // EE_CAN_BoardAddress;      :8
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

// eeprom filter data init
static struct s_eeIIRTransposedCoefs _EEDATA(1) eeIIRTransposedCoefs =
{
  2,      // n. BiQuads
  {
  // filter coefs LPF 50Hz 
  0x00E3,
  0x00DD,
  0x68D6,
  0x00E3,
  0xD487,
  0x0343,
  0xFDD5,
  0x728F,
  0x0343,
  0xC914
  }
};

//
// globals 
//

// Board Configuration image from EEPROM
struct s_eeprom BoardConfig = {0}; 

//
// IIRs/FIR variables

s_eeIIRTransposedCoefs IirTransposedCoefs __attribute__((__space__(xmemory))) = { 0 };
char filter_enable = 0;
char can_enable = 0;
char mux_enable = 1;
char muxed_chans =5;
void T2(void);
fractional IirTransposedState1[6][IIR_LPF_N_MAX_BQ * 2] __attribute__((__space__(ymemory)));
fractional IirTransposedState2[6][IIR_LPF_N_MAX_BQ * 2] __attribute__((__space__(ymemory)));
IIRTransposedStruct iirt[6];

// CAN message buffer
#define CAN_MAX_MESSAGES_IN_BUFF 10
// Can message buffer pointer
int CAN_Messages_buff_ptr = 0;
// can messages buffer
struct canmsg_tag CAN_Messages[CAN_MAX_MESSAGES_IN_BUFF];

// dimensione del buffer di ingresso dei campioni analogici
// #define AN_SAMPLES_BUFF_DIM 256 
// extern fractional square1k[AN_SAMPLES_BUFF_DIM];
// fractional FilterOut; 

//
// prototypes 
//

extern void IIRTransposedInit(IIRTransposedStruct* filter);
extern fractional* IIRTransposed (int numSamps, fractional* dstSamps, fractional* srcSamps, IIRTransposedStruct* filter);
void IIRFiltersINIT();

void __delay32(unsigned long);
void AMUXChannelSelect(unsigned CH);
   
void EEnqueue(unsigned x)
// Insert error in queue
{
// timestamp
// enqueue
// flag error
}

// error codes
#define ERR_MUX_INDEXING            1
#define ERR_DAC_VALUE2BIG           2
// CAN command parameters invalid 
#define ERR_CAN_PARAMETERS_INVALID  3
// CAN command unavailable 
#define ERR_CAN_COMMAND_UNAVAILABLE 4

// SG to TF matrix indexing invalid 
#define ERR_CAN_MATRIX_INDEXING     5

// IIR Filter Number of biquads too big  
#define ERR_CAN_IIR_NBQ2BIG         6
// IIR Filter Coeff Indexing error  
#define ERR_CAN_IIR_COEF_INDEXING   7

// SW CAN Messages Rx buffer overflow
#define ERR_CAN_RXBUFF_OVERFLOW     8

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

void SetDACGetADC(unsigned PowerDownMode, unsigned int DACValue, unsigned int *ADCValue)
// set the value for the DAC and get ADC value
// PAY ATTENCTION: do not change the sequence of operation: tight timing at work!
// Durata circa 3 usec
{
  unsigned int tmp; 
  if( DACValue > 0x3FF) 
  {
    EEnqueue(ERR_DAC_VALUE2BIG);
    DACValue = 0x3FF;  // saturate to max
  }
LATBbits.LATB6=1;  //led on
LATAbits.LATA11 = 1; // startconv must be longer than 3.2u  

  while(SPI1STATbits.SPITBF);  // transmission buffer full
  tmp = PDM_NORMAL & (DACValue<<2);
  __delay32(30);
 LATDbits.LATD9  = 0; // SS DAC 
LATAbits.LATA11 = 0;  
 LATBbits.LATB6=0; //led off
 
  WriteSPI1(tmp); 
  __delay32(10); 
 LATDbits.LATD9 = 1;
 *ADCValue=ReadSPI1(); // get ADC value
}

// inizializzazione porte IO
void InitPorts(void)
{    
#ifdef MAIS 
  ; // All as in reset state
#else 
  // TRISA = 0x0800;  // all inputs
  TRISB = 0x1FFF;  // all inputs
  TRISC = 0xE000;  // all inputs
  TRISD = 0x030F;  // all inputs
  // TRISE = 0x013F;  // all inputs
  TRISF = 0x007F;  // all inputs
#endif 
}

//
// ADC12 IRQ Service Routines
// 
void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void)
{   
  IFS0bits.ADIF = 0;
}  

//
// CAN IRQ Service Routines
// 
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void)
{   
  // Clear interrupt flag
  IFS1bits.C1IF = 0;
  
  // Tx IRQs should be disabled
  if(C1INTFbits.TX0IF)      //  Tx0 Interrupt
    C1INTFbits.TX0IF = 0;   
  else 
  {
    if(C1INTFbits.TX1IF)    //  Tx1 Interrupt
      C1INTFbits.TX1IF = 0; 
  }
  if(C1INTFbits.RX0IF)
  {
    C1INTFbits.RX0IF = 0;   // clear Rx0 Interrupt
    
    // Read received data from receive buffer and store it into user defined dataarray
    CAN1ReceiveMessage(&CAN_Messages[CAN_Messages_buff_ptr].CAN_Per_Msg_PayLoad[0], 8, 0);
    CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_AsBytes[0] = C1RX0SID; // get Dest, Source and Class 

    // clear Rx buffer0 full flag
    C1RX0CONbits.RXFUL = 0;
 
    // increase buff pointer
    if (CAN_Messages_buff_ptr < (CAN_MAX_MESSAGES_IN_BUFF-1))
      CAN_Messages_buff_ptr++;
    else
      EEnqueue(ERR_CAN_RXBUFF_OVERFLOW);
  }
  else
  {
    if(C1INTFbits.RX1IF)
    {
      C1INTFbits.RX1IF = 0; //If the Interrupt is due to Receive1 of CAN1 Clear the Interrupt
  
      // Read received data from receive buffer and store it into user defined dataarray
      
  // TODO: testare la modifica fatta a queste tre righe
      //CAN1ReceiveMessage(&CAN_Messages[CAN_Messages_buff_ptr].CAN_Per_Msg_PayLoad[0], 8, 0);
      //CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_AsBytes[0] = C1RX0SID; // get Dest, Source and Class
      //C1RX0CONbits.RXFUL = 0; 
      CAN1ReceiveMessage(&CAN_Messages[CAN_Messages_buff_ptr].CAN_Per_Msg_PayLoad[0], 8, 1);
      CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_AsBytes[0] = C1RX1SID; // get Dest, Source and Class 
      // clear Rx buffer0 full flag
      C1RX1CONbits.RXFUL = 0;
  // TODO: testare la modifica fatta a queste tre righe
 
      // increase buff pointer
      if (CAN_Messages_buff_ptr < (CAN_MAX_MESSAGES_IN_BUFF-1))
        CAN_Messages_buff_ptr++;
      else
        EEnqueue(ERR_CAN_RXBUFF_OVERFLOW);
    }
  }
  // toggled
  //LATBbits.LATB12 = ~LATBbits.LATB12;
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
  unsigned int adc;
    
  // LATBbits.LATB12 = 1; // led on

  //SetDACGetADC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[BoardConfig.EE_AN_SelectedChannel], &adc);

  // LATBbits.LATB7=1; 
  // AMUXChannelSelect(BoardConfig.EE_AN_SelectedChannel);
  
  // LATBbits.LATB12 = 0; // led off
  // LATBbits.LATB7=0; 

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

void AMUXChannelSelect(unsigned CH)
// select the specified analog nux channel and 
// perform phisical channel descrambling
// 
{
  //
  // STRAIN
  //
  switch (CH)
  {
  case 0: 
    // CH1 -> (J1-5) -> S5 -> A(1,0,0) -> RB(...x,x,0,0,1,x,x,x)
    LATBbits.LATB3 = 1; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 0; // A0
  break;
  case 1: 
    // CH2 -> (J1-2) -> S7 -> A(1,1,0) -> RB(...x,x,0,1,1,x,x,x)
    LATBbits.LATB3 = 1; // A2
    LATBbits.LATB4 = 1; // A1
    LATBbits.LATB5 = 0; // A0
  break;
  case 2:
    // CH3 -> (J3-2) -> S1 -> A(0,0,0) -> RB(...x,x,0,0,0,x,x,x)
    LATBbits.LATB3 = 0; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 0; // A0
  break;
  case 3: 
    // CH4 -> (J3-5) -> S4 -> A(0,1,1) -> RB(...x,x,1,1,0,x,x,x)
    LATBbits.LATB3 = 0; // A2
    LATBbits.LATB4 = 1; // A1
    LATBbits.LATB5 = 1; // A0
  break;
  case 4: 
    // CH5 -> (J2-5) -> S2 -> A(0,0,1) -> RB(...x,x,1,0,0,x,x,x)
    LATBbits.LATB3 = 0; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 1; // A0
  break;
  case 5: 
    // CH6 -> (J2-2) -> S6 -> A(1,0,1) -> RB(...x,x,1,0,1,x,x,x)
    LATBbits.LATB3 = 1; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 1; // A0
  break;

  default:
    // add an MUXINDEXINGERROR error in cueue
    EEnqueue(ERR_MUX_INDEXING);
  break;
  }
}

#define SendCanProblem() {  i = (msg->CAN_Per_Msg_Class << 8 ) | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | ( msg->CAN_Poll_Msg_Source ); datalen = 4; Txdata[0]=msg->CAN_Per_Msg_PayLoad[0];Txdata[1] ='B';Txdata [2] ='U';Txdata[3] ='G';}

void SaveEepromBoardConfig()
{
	// 
	// Save Board Cnfiguration to EE
	//

	_prog_addressT EE_addr;
	int i=0, *DMAdx;
	
	// initialize a variable to represent the Data EEPROM address 
	_init_prog_address(EE_addr, ee_data);
	
	// Erase Data EEPROM at ee_data  
	for(i=0 ; i < sizeof(BoardConfig) ; i++)
	{
	  _erase_eedata(EE_addr++, _EE_WORD);
	  _wait_eedata();
	}
	
	// Write BoardConfig to Data EEPROM
	_init_prog_address(EE_addr, ee_data);
	DMAdx = (int*) &BoardConfig;
	for(i=0 ; i < sizeof(BoardConfig) ; i++)
	{
	  _write_eedata_word(EE_addr, *DMAdx );
	  EE_addr+=2;
	  DMAdx++;
	  _wait_eedata();
	}
}

void SaveEepromIIRFilter()
{
}

void SetBoardCanFilter()
{
	// Enable CAN IRQ
	DisableIntCAN1;
	
	// Set request for configuration (inizialization) mode
	CAN1SetOperationMode(CAN_IDLE_CON & CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_CONFIG & CAN_CAPTURE_DIS);
	while(C1CTRLbits.OPMODE <=3);
	
	CAN1SetFilter(0, CAN_FILTER_SID( (CAN_MSG_CLASS_POLLING|BoardConfig.EE_CAN_BoardAddress) ) // 0x205
	  & CAN_RX_EID_DIS, CAN_FILTER_EID(0));

	CAN1SetFilter(1, CAN_FILTER_SID( (CAN_MSG_CLASS_LOADER|BoardConfig.EE_CAN_BoardAddress) )  // 0x705
    & CAN_RX_EID_DIS, CAN_FILTER_EID(0));

	CAN1SetFilter(2, CAN_FILTER_SID( (CAN_MSG_CLASS_LOADER|0x0F) )  // 0x70F
    & CAN_RX_EID_DIS, CAN_FILTER_EID(0));
	
	// set acceptance mask
	// answer commands from any source //0x70F
	CAN1SetMask(0, CAN_MASK_SID(0x70F) & CAN_MATCH_FILTER_TYPE, CAN_MASK_EID(0));
    CAN1SetMask(1, CAN_MASK_SID(0x70F) & CAN_MATCH_FILTER_TYPE, CAN_MASK_EID(0));
	
	// abort any spurious Tx
	CAN1AbortAll();
	
	// Set Operation Mode  NORMAL
	CAN1SetOperationMode(CAN_IDLE_CON & CAN_CAPTURE_DIS &	CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_NOR); 
	
	// Enable CAN IRQ
	EnableIntCAN1;
}

void ParseCommand(canmsg_t *msg) //, unsigned int SID)
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
#ifdef MAIS
#else 
  unsigned int j,tmp,tout;
#endif

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
	    break;

		case CAN_CMD_GET_MATRIX_RC:
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
	    break;

		case CAN_CMD_CALIBRATE_OFFSET:
		break;
	
	    case CAN_CMD_SET_CH_DAC:
	      //  set DAC value 0x205 len 4  data 4 ch msb lsb
	  //
	  // STRAIN
	  //
	      i = msg->CAN_Per_Msg_PayLoad[2]<<8 | msg->CAN_Per_Msg_PayLoad[3];
	      j = msg->CAN_Per_Msg_PayLoad[1];
	      if ( (j >= 0) && (j <= 5 ))
	      {
	        SetDACGetADC(PDM_NORMAL, i, &tmp);
	        BoardConfig.EE_AN_ChannelOffset[j]=i;
	      }
	      else 
	      {
	        EEnqueue(ERR_CAN_PARAMETERS_INVALID);
	        return; 
	     }
	      datalen=0;
	    break;
	
	    case CAN_CMD_SET_TXMODE:
	    break;

	    case CAN_CMD_FILTER_EN:
	    break;

	    case CAN_CMD_MUX_EN:
	    break;

	    case CAN_CMD_MUX_NUM:
	    // set continuous or on demand tx  0x205 len 2  data 7 0/1
	      muxed_chans=msg->CAN_Per_Msg_PayLoad[1];
   		  AMUXChannelSelect(muxed_chans);
	      datalen=0;
	    break;

	
	    case CAN_CMD_SET_CANDATARATE:
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

void IIRFiltersINIT()
{
  int i;
  
  for( i=0; i<6 ; i++) 
  { 
    iirt[i].numSectionsLess1 = IirTransposedCoefs.IIR_N_BQ - 1;
    iirt[i].coeffsBase = &IirTransposedCoefs.IirTransposedCoefs[0];
    iirt[i].coeffsPage = COEFFS_IN_DATA;
    iirt[i].delayBase1 = &IirTransposedState1[i][0];
    iirt[i].delayBase2 = &IirTransposedState2[i][0];
    //@@@@ cambiato 2 gennaio, prima final shidt era 0
	iirt[i].finalShift = 0; //2; // to rescale the missing 0.5 * 0.5 of the two stages
  
    // Zero the filter state
    IIRTransposedInit(&iirt[i]);
  }
}

int main(void)
{ 
  // int i;
  unsigned int match_value;
 
  // Test CAN vars
  unsigned char datalen;
  unsigned char Txdata[8] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88}; 
  canmsg_t CAN_Msg;

  char FilterNo,tx_rx_no;
  
  // #define  DATAARRAY 0x1820
  // unsigned char *datareceived = (unsigned char *) DATAARRAY;  
  unsigned char Rxdata[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}; 

  // Test ADC12 vars
  //unsigned int Channel, PinConfig, Scanselect;
  //unsigned int Adcon3_reg, Adcon2_reg, Adcon1_reg;
  
  // EEDATA Access vars
  _prog_addressT EE_addr;

  unsigned int i;
#ifdef MAIS
  unsigned int Channel, PinConfig, Scanselect;
  unsigned int Adcon3_reg, Adcon2_reg, Adcon1_reg;
#else 
  unsigned int tmp;
  unsigned int SPISTATValue; // Holds the information about SPI Enable/Disable
  static unsigned int SPICONValue; // Holds the information about SPI configuartion
#endif
 
  InitPorts();

  //
  // init code
  //
 
  // init/clear error queue and flags

  // turn IO ports and periferals
#ifdef MAIS
  // 
  // MAIS
  // 
  TRISFbits.TRISF4 = 0; // set F4 as out (LED Yellow)
  TRISFbits.TRISF5 = 0; // set F5 as out (LED Blue)

#else 
  //
  // STRAIN
  //
  TRISBbits.TRISB12 = 0; // set B12 as out (LED)

//@@@@
// FOr test puropuses
  TRISBbits.TRISB6 = 0; // set sampling trigger
  TRISBbits.TRISB7 = 0; // set mux CH1 trigger
  LATBbits.LATB6=0; 
  LATBbits.LATB7=0;
//@@@@

  TRISBbits.TRISB5  = 0;  // set B5 as out (MUXA0)
  TRISBbits.TRISB4  = 0;  // set B4 as out (MUXA1)
  TRISBbits.TRISB3  = 0;  // set B3 as out (MUXA2)
  
  TRISAbits.TRISA11 = 0; // set A11 as out (STARTCONV ADC)
  LATAbits.LATA11   = 0; // Start conversion inactive   

  TRISDbits.TRISD9  = 0; // set D9 as out (\SS1 = \SS DAC)
  LATDbits.LATD9    = 1; // SS DAC inactive

  // Select analog channel 0
  AMUXChannelSelect(0);

  // Configure SPI1 module to transmit 16 bit timer1 value in master mode
///@@@ originale SPI_CKE_OFF CLK_POL_ACTIVE_HIGH
///@@@ provato   SPI_CKE_ON  CLK_POL_ACTIVE_HIGH nn va (sempre valori bassi)
///@@@ provato   SPI_CKE_OFF CLK_POL_ACTIVE_LOW nn va (salta da 0 a 32000)
///@@@ provato   SPI_CKE_ON  CLK_POL_ACTIVE_LOW va come il caso 1....
  SPICONValue  =  FRAME_ENABLE_OFF & FRAME_SYNC_OUTPUT &
    ENABLE_SDO_PIN & SPI_SMP_ON & SPI_CKE_OFF & SPI_MODE16_ON &
    SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH &
    MASTER_ENABLE_ON & SEC_PRESCAL_1_1 & PRI_PRESCAL_1_1; 

  SPISTATValue  = SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR;
 ConfigIntSPI1(SPI_INT_PRI_4 &  SPI_INT_EN);

  OpenSPI1(SPICONValue,SPISTATValue );

#endif

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
  _init_prog_address(EE_addr, ee_data);
  _memcpy_p2d16(&BoardConfig, EE_addr, sizeof(BoardConfig));
  // todo: checksum verification
  
  // IIR filter coefs
  //$$$_init_prog_address(EE_addr, eeIIRTransposedCoefs);
  //$$$_memcpy_p2d16(&IirTransposedCoefs, EE_addr, sizeof(IirTransposedCoefs));

  //
  // IIR Transposed filters init
  //
  //$$$IIRFiltersINIT();

 // Set DAC for channel 0
  tmp=0;
  SetDACGetADC(PDM_NORMAL, BoardConfig.EE_AN_ChannelOffset[0], &tmp);
  tmp=0;

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

 
  // flashled   
  for(i=1;i<=20;i++)
  {  
  //
  // STRAIN
  //
    LATBbits.LATB12 = ~LATBbits.LATB12;
    __delay32(800000);
  }


  //
  // Test STRAIN 
  //   get commands (11 bit ID) via CAN 
  //

  // risponde a un comando: class boardtest, any source, board address destination
  //  -------  -------  -------  -------  ------- 
  // | 3b     | 4b     | 4b     |                |
  // | class  | Source | Dest   |      ....      |
  //  -------  -------  -------  -------  ------- 
  //   command:
  //     CAN_MSG_CLASS_BOARDTEST xxxx ee_data.EE_CAN_BoardAddress 
  //   mask:
  //     0b010 xxxx 0101 -> mask = 0x70F

  // CAN Configuration

  // Set request for configuration (inizialization) mode
  // Fcan=20MHz
  CAN1SetOperationMode(CAN_IDLE_CON & CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_CONFIG & CAN_CAPTURE_DIS);
  // C1CTRL = CAN_CAPTURE_DIS & CAN_IDLE_CON & CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_CONFIG ;
  while(C1CTRLbits.OPMODE <=3);

  // Load configuration register
  // 1Mbps: 
  // PRESCALER = 1 -> Tq = 4/Fcan, 
  // SINCROJUMP=1q, PHASE1=3q, PHASE2=5q,  PROPAGATION=1q  
  // q=1+3+5+1=10 quanti -> 1Meg 
  CAN1Initialize(CAN_SYNC_JUMP_WIDTH1 & CAN_BAUD_PRE_SCALE(1), CAN_WAKEUP_BY_FILTER_DIS &  CAN_PHASE_SEG2_TQ(5) & 
    CAN_PHASE_SEG1_TQ(3) & CAN_PROPAGATIONTIME_SEG_TQ(1) & CAN_SEG2_FREE_PROG & CAN_SAMPLE1TIME);
  // C1CFG1 = CAN_SYNC_JUMP_WIDTH1 & CAN_BAUD_PRE_SCALE(1)
  // C1CFG2 = CAN_WAKEUP_BY_FILTER_DIS & CAN_PHASE_SEG2_TQ(5) & CAN_PHASE_SEG1_TQ(3) & CAN_PROPAGATIONTIME_SEG_TQ(4) & CAN_SEG2_FREE_PROG & CAN_SAMPLE1TIME

  // Set transmitter and receiver mode
  tx_rx_no = 0;  // C1TX0CON
  //CAN1SetTXMode(tx_rx_no, CAN_TX_REQ & CAN_TX_PRIORITY_HIGH );
  CAN1SetTXMode(tx_rx_no, CAN_TX_PRIORITY_HIGH );
  CAN1SetRXMode(tx_rx_no, CAN_RXFUL_CLEAR & CAN_BUF0_DBLBUFFER_EN);

  // Configure CAN IRQs 
  ConfigIntCAN1(CAN_INDI_INVMESS_EN & CAN_INDI_WAK_DIS & CAN_INDI_ERR_DIS &
    CAN_INDI_TXB2_DIS & CAN_INDI_TXB1_DIS & CAN_INDI_TXB0_DIS &  // CAN TX IRQ OFF
    CAN_INDI_RXB1_EN & CAN_INDI_RXB0_EN ,                        // CANRX IRQ ON 
    CAN_INT_PRI_3 & CAN_INT_DISABLE);

  // Cofigure CAN filter
  SetBoardCanFilter();

  unsigned int SID = (CAN_MSG_CLASS_POLLING) | ((BoardConfig.EE_CAN_BoardAddress)<<4) | (CAN_CMD_TORQUE_VECTOR) ;
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
	//asm ("");
//	asm ("PWRSAV %0"::"0");

    if (CAN_Messages_buff_ptr != 0)
    // Something in the CAN message buffer
    {

      // Read received data from receive buffer and store it into user defined dataarray

      //mask CANRxIRQ for atomic buffer manipulation
      DisableIntCAN1;
      memcpy(&CAN_Msg,&CAN_Messages[CAN_Messages_buff_ptr-1],sizeof(CAN_Msg));
      CAN_Messages_buff_ptr--;
      EnableIntCAN1;
      ParseCommand(&CAN_Msg); //&CAN_Messages[CAN_Messages_buff_ptr-1].CAN_Per_Msg_PayLoad[0], CAN_Messages[CAN_Messages_buff_ptr-1].CAN_Poll_Msg_Dest);
    }
    else 
      idle();
  }
}
