//  BOOTLOADER via CAN 
//  Author F.Larosa
//  Rev. 1.0 del 08/21/2008
//
//  Tested on MAIS and STRAIN boards (DSPIC30F4013)
//  pic30-gcc V4.03
//  MPLAB IDE ver 8.02.00.00
// 




/* File Description | Notes:
** =========================
** 1]  
**
* Revision History:
** =================
**-------------------------------------------------------------------------
**Rev:   Date:        Details:                                    Who:
**-------------------------------------------------------------------------
**1.0   21 Aug 2008  First emission								F. Larosa
**
**1.1   25 Aug 2008  Modified BOOTLDR_ADDR                      F. Larosa
**                   Renamed .gld file in Bootloader.gld
**                   Updated build number
**
**1.2   15 Gen 2009  Added CAN1SetMask for FilterNo = 1         F. Larosa
**                   Added ClrWdt() in the command CMD_END
**                   Added LoadProgram flag (is set in CM_BOARD command, and is checked in 
**                   CMD_ADDRESS, CMD_DATA and CMD_START command
**                   Modified .gld file and BOOTLDR_ADDR costant (new start address is 0x6D80)
**
**1.3   21 Apr 2009  Changed the port F4 with the port F5       M.Maggiali 
**					 in the board detection section. Now 
**				     the led for the mais is F4                      
**-------------------------------------------------------------------------
**
**
***************************************/



// TODO: Buffer SW trasmissione messaggi can (se la linea e`piena si rischia l'overrun)

#include <p30f4013.h>
#include <timer.h>
#include <can.h>
#include <string.h>
#include <libpic30.h>
#include <dsp.h>
#include "Bootloader.h"


#define VERSION   1
#define BUILD     3

#define CONFIG_WORD_WRITE  0x4008

#define BOARD_TYPE_STRAIN  0x06
#define BOARD_TYPE_MAIS    0x07

#define BOOTLDR_ADDR  0x6D80      //First row address for bootloader
#define BOOTWAIT     5            //Waiting time (in sec.) to start the application                            

// todo: inizializzazione a varie velocità

// Frame formats
#define CAN_STANDARD_FORMAT 0
#define CAN_EXTENDED_FORMAT 1

// Frame types
#define CAN_DATA_FRAME      0
#define CAN_REMOTE_FRAME    1


#define CAN_MSG_CLASS_LOADER   0x700
// For messages of class 111 the meaning of data/ID is defined as follows:
//  -------------------------- ---------------------
// |           11b            |           8B        |
//  -------  -------  -------  --------------------- 
// | 3b     | 4b     | 4b     |         B[0-7]      |
// |class	| Source | Dest   |        Payload      |
//  -------  -------  -------  --------------------- 

#define CAN_TX_SOFTWARE_BUFFER_SIZE 10


// ---------------------------------------------------------------------

// CAN Loader Commands

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

// Request for board type and firmware version
#define CMD_BROADCAST             0xFF

#define CMD_ENABLELOG             0x06

#define CMD_ERRORLOG              0x07

// ---------------------------------------------------------------------

// CAN loader constant

//Instruction code = 24 bits

#define START_PM                  0x000000   //First address for program memory (16K x instruction = 16K x 24 bits)
#define END_PM                    0x007FFE   //Last address for program memory

#define START_EE                  0x7FFC00   //First address for EEPROM (1K x 16 bits)
#define END_EE                    0x7FFFFE   //Last address for EEPROM

#define START_CM                  0xF80000   //First address for Configuration registers (7 x 16 bits)
#define END_CM                    0xF8000C   //Last address for Configuration registers

//#define PM_ROWSIZE                32         //Row size for program memory: 32 instructions (24+8 bits)
#define EE_ROWSIZE                16         //Row size for EEPROM: 16 word (1 word = 16bit)
#define CM_ROWSIZE                7          //Row size for Configuratione memory: 7 word (1 word = 16 bit)  

#define PM                        1
#define EE                        2
#define CM                        3

#define FOSC                      0xF80000
#define FWDT                      0xF80002
#define FBORPOR                   0xF80004
#define FBS                       0xF00006
#define FSS                       0xF00008
#define FGS                       0xF8000A
#define FICD                      0xF8000C

#define FOSC_MASK                  0xC71F
#define FWDT_MASK                  0x803F
#define FBORPOR_MASK               0x87B3
//#define FBS_MASK                   0x310F
//#define FSS_MASK                   0x330F
#define FGS_MASK                   0x0007
#define FICD_MASK                  0xC003


//Data length (in byte) for ADDRESS command
#define FIELDLEN_ADDRESS          0x07

//Data length (in byte) for START command
#define FIELDLEN_START            0x05

//Buffer length for HEX parser
#define PRS_BUFFER_LEN           125      //  todo: verificare questo valore

//Data length (in byte) for BOARD command
#define FIELDLEN_BOARD            0x02

//Data length (in byte) for ENABLELOG command
#define FIELDLEN_ENABLELOG        0x02

// can bus message structure
typedef struct canmsg_tag
{
  union
  {
    struct // for CAN loader messages
    {
      unsigned CAN_RxIDE           :1;
      unsigned CAN_SRR             :1;
      unsigned CAN_Msg_Dest        :4;  // Destination
      unsigned CAN_Msg_Source      :4;  // Source
      unsigned CAN_Msg_Class       :3;  // Class of message
      unsigned NotUsed             :3;
      
      unsigned CAN_DLC             :4;  //Data Length Code bits (Contents of Receive Buffer)
      unsigned CAN_RB0             :1;  //Reserved by CAN Spec and read as ‘0’
      unsigned CAN_NU_CiRXnDLC     :3;  //Not Used
      unsigned CAN_RB1             :1;  //Reserved by CAN Spec and read as ‘1’
      unsigned CAN_RXRTR           :1;  //Receive Remote Transmission Request bit
      unsigned CAN_EID             :6;  //Extended Identifier bits

      unsigned char CAN_Msg_PayLoad[8]; // Payload 
    };
    unsigned short CAN_Msg_AsBytes[12]; // Message data formatted as bytes 
  };
} canmsg_t;

/*
typedef unsigned long  UWord32;
typedef unsigned short UWord16;

typedef union tuPMBuf
{
  unsigned char asByte[PM_ROWSIZE*4];
  int asWord16[PM_ROWSIZE*2];
  long asWord32[PM_ROWSIZE];
} uBuf;


typedef union tuReg32
{
	UWord32 Val32;

	struct
	{
		UWord16 LW;
		UWord16 HW;
	} Word;

	unsigned char Val[4];
} uReg32;


typedef struct
{
  uReg32 Addr;
  uReg32 Data;
} tConfReg;
*/
// Setup for configuration bit (p30f4013.h)
_FOSC(CSW_FSCM_OFF & ECIO_PLL8); 
  // Clock switching disabled Fail safe Clock Monitor disabled
  // External clock with PLL x8 (10MHz*8->Fcycle=80/4=20MIPS)
_FGS(CODE_PROT_OFF); // Code protection disabled

_FWDT(WDT_OFF);     // WD disabled
//_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_1); // WD enabled 1:512*16

_FBORPOR(MCLR_EN & PWRT_64 & PBOR_ON & BORV_27);  // BOR 2.7V POR 64msec

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
  0x05,          // EE_CAN_BoardAddress;      :8    @@
  0x01,          // EE_CAN_MessageDataRate    :8
  0x04,          // EE_CAN_Speed;             :8
  {
    0x0,0x0,   // EE_AN_ActiveChannels      :8
    0x0,0x0,
    0x0,0x0,
    0x0,0x0,
    0x0,0x0,
    0x0,0x0,
    0x0
  },
  0x00,          // EE_AN_Selected channel
  0x00,        // EE_AN_ChannelScanningTime :16
  {
    0x0,0x0, // EE_AN_ChannelOffset[6]  :6*16
    0x0,0x0,
    0x0,0x0 
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
    {0,0,0,0,0,0},  // TT_TF_TMatrix
    {0,0,0,0,0,0},
    {0,0,0,0,0,0},
    {0,0,0,0,0,0},
    {0,0,0,0,0,0},
    {0,0,0,0,0,0},
  },
  0x0000 // Checksum
};

// eeprom filter data init
static struct s_eeIIRTransposedCoefs _EEDATA(1) eeIIRTransposedCoefs =
{
  0,      // n. BiQuads
  {
  // filter coefs LPF 50Hz 
  0x0,
  0x0,
  0x0,
  0x0,
  0x0,
  0x0,
  0x0,
  0x0,
  0x0,
  0x0
  }
};

//
// globals 
//

unsigned int CANLoadWaiting = 1;    //It's 1 while CANLoad loop
unsigned int comContinue = 0;       //It's 0 to interrupt the comunication loop
unsigned int LoadProgram = 0;		//It's 1 when the HEX file program is loaded in program memory

unsigned int MemType;               //It's memory type: PM, EE, CM costant

unsigned int PMBufferIsFull = 0;    //It's 1 if program memory buffer is full

unsigned int UpdateEE = 0;          //It's 1 if EEPROM must be reprogrammed
uReg32 prsAddress;                  //Start Address for packet
unsigned char prsLength = 0;        //Packet length
unsigned char prsMemoryType = 0;    //Memory type: 0 for program, 1 for data memory
unsigned int prsErase = 0;          //It's 0 the first time, when the memory isn't clear
uBuf Buffer;                        //Data buffer for data packet

tConfReg CMBuffer[CM_ROWSIZE];      //Data buffer for configuratione memory data packet  

unsigned int T1sec = 2*BOOTWAIT;	//Delay in seconds for waiting loop
UWord32 StartAddr;                  //Address range to be programmed
UWord32 EndAddr;
unsigned int BufIndex;				//Index for Buffer.asByte[] used for Hex packet data
unsigned int CMBufIndex = 0;
unsigned char ByteCntr;             //Counter for received bytes (used to increment the prsAddress)

//UWord16 FOSC_Val;
UWord16 FWDT_Val;
UWord16 FBORPOR_Val;
uReg32 ConfAdr;

unsigned char BoardType;           //Used to identify the board type (MAIS or STRAIN)
unsigned int EnableLOG = 0;        //Used to enable error messages (see CMD_ENABLELOG parsing)


// Board Configuration image from EEPROM
struct s_eeprom BoardConfig = {0}; 


// CAN message buffer
#define CAN_MAX_MESSAGES_IN_BUFF 40
// Can message buffer pointer
int CAN_Messages_buff_ptr = 0;
// can messages buffer
struct canmsg_tag CAN_Messages[CAN_MAX_MESSAGES_IN_BUFF];

void __delay32(unsigned long);
   
void EEnqueue(unsigned x)
// Insert error in queue
{
// timestamp
// enqueue
// flag error
}

// error codes

#define CMD_BOARD_ERROR             1

#define CMD_ADDRESS_ERROR           2

#define CMD_START_ERROR             3

// CAN command unavailable 
#define COMMAND_UNAVAILABLE_WARNING 4

#define CMBUFINDEX_ERROR            5

#define BUFFERISFULL_ERROR          6

#define PAYLOAD_ERROR               7

// SW CAN Messages Rx buffer overflow
#define ERR_CAN_RXBUFF_OVERFLOW     8




void Init_Buffer(void)
//Initialize the buffer used to write the  flash memory
{
  int i;
  for(i=0; i < PM_ROWSIZE; i++)
    //Set the bits to '1' (equivalent to blank memory)
    Buffer.asWord32[i] = 0xFFFFFFFF;  
}

// Setup for I/O ports
void InitPorts(void)
{    
  unsigned char BoardId;
  // set pin 2 as input
  // active weak pull-up device connected to the pin CN18 (pin 2)
  // used to select a STRAIN or MAIS device
  TRISFbits.TRISF5 = 1;

  CNPU2bits.CN18PUE = 1;    
  
  //Detect the board type reading RF5 bit from RF port
  //and set a bit as out for a LED (according the board type)
  //It's necessary a double read for a correct reading after
  //the weak pull-up enabling
  BoardId = PORTFbits.RF5;
  BoardId = PORTFbits.RF5;

  if(BoardId)
  //STRAIN boaard (RF5 is unconnected with an weak pull-up)
  {
    BoardType = BOARD_TYPE_STRAIN;
    TRISBbits.TRISB12 = 0; // set B12 as out (LED)
  }
  else
  //MAIS board (RF5 is connected to pulldown)
  {
    BoardType = BOARD_TYPE_MAIS;
    TRISFbits.TRISF4 = 0; // set F4 as out (LED Yellow)
  }
}

void SwOffLED(void)
//Switch off the LED output according the board type
//For MAIS the LED is on RF port
//For STRAIN the LED is on RB port
{
  if(BoardType == BOARD_TYPE_MAIS)
    LATFbits.LATF4  = 0;
  else
    LATBbits.LATB12 = 0;
} 


void ToggleLED(void)
//Toggle LED output according the board type
//For MAIS the LED is on RF port
//For STRAIN the LED is on RB port
{
  if(BoardType == BOARD_TYPE_MAIS)
    LATFbits.LATF4  = ~LATFbits.LATF4;
  else
    LATBbits.LATB12 = ~LATBbits.LATB12;
} 

void FlashLED(void)
//Used to flash LED
{
  unsigned int i;
  for(i=1;i<=20;i++)
  {  
    ToggleLED();
    __delay32(800000);
  }
}


void SendCanMsg (unsigned int ID, unsigned char *Data, unsigned char DataLen)
//This procedure sends a CAN message according to the parameters
//THERE ISN'T A TX BUFFER
{
  // Wait till previous message is transmitted completely
  while(!CAN1IsTXReady(0))
    ;
  CAN1SendMessage((CAN_TX_SID(ID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, Data,DataLen,0); 
}


void comACK (unsigned char Dest, unsigned char Cmd )
//It sends command acknowledgement for 'Cmd' command to 'Dest' destination
//This ackowledgement is formed by two byte (TxData)
{
  unsigned int comACK_ID;
  unsigned char TxData[2];

  TxData[0] = Cmd;
  TxData[1] = 0x01;
  comACK_ID = CAN_MSG_CLASS_LOADER | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | ( Dest & 0xF );
  SendCanMsg(comACK_ID, TxData, 2);
}

void SendCanErrorMsg(unsigned char Dest, unsigned char ErrorCode)
{
  unsigned int comACK_ID;
  unsigned char TxData[2];

  if(EnableLOG)
  {
    TxData[0] = CMD_ERRORLOG;
    TxData[1] = ErrorCode;
    comACK_ID = CAN_MSG_CLASS_LOADER | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | ( Dest & 0xF );
    SendCanMsg(comACK_ID, TxData, 2);
  }
}

void HaltBootloader(unsigned char Dest, unsigned char ErrorCode)
{
  int i;
  while(1)
  {
    SendCanErrorMsg(Dest, ErrorCode);
    for(i=1; i<=100; i++)
      ;
  }
}

void InitBufParams(unsigned int RowSize, unsigned long OffsetAdr)
//Initialize globals variables StartAddr, BufIndex, using this parameters:
//RowSize is the row size for the memory area specified by prsAddress
//OffsetAdr is the first address for the specified memory area
{
  unsigned int RowNumber;
  //Calculate the correct index for the buffer
  RowNumber = floor((prsAddress.Val32-OffsetAdr)/(2*RowSize))+1;
  StartAddr = (RowNumber-1)*2*RowSize+OffsetAdr;
  EndAddr = StartAddr+(RowSize-1)*2;
  if(OffsetAdr == START_PM)
    BufIndex = (prsAddress.Val32-StartAddr)*2;
  else if(OffsetAdr == START_EE)
    BufIndex = (prsAddress.Val32-StartAddr);
}

void InitAdrIndexBuf(void)
//Calculate new row address range, buffer index for the new row
{
  switch(MemType)
  {
    case PM:
      InitBufParams(PM_ROWSIZE,START_PM);
    break;
    case EE:
      InitBufParams(EE_ROWSIZE,START_EE);
    break;
    case CM:
      InitBufParams(CM_ROWSIZE,START_CM);
    break;
  }
}

int BufferIsFull(void)
//Return 1 if Buffer array is full according to the memory type (MemType).
// - For PM the limit (bytes number) is PM_ROWSIZE*4
// - For EE the limit (bytes number) is EE_ROWSIZE*2
// - For CM the limit (bytes number) is CM_ROWSIZE*2
//Return 0 in other case.
{
  switch(MemType)
  {
    case PM:
      if(BufIndex >= PM_ROWSIZE*4)
        return 1;
      else
        return 0;
    break;
    case EE:
      if(BufIndex >= EE_ROWSIZE*2)
        return 1;
      else
        return 0;
    break;
    case CM:
      if(BufIndex >= CM_ROWSIZE*2)
        return 1;
      else
        return 0;
    break;
    default:
      return -1;
  }
}

void WriteBuffer(void)
//Write Buffer on flash memory
{
//  LATBbits.LATB12 = ~LATBbits.LATB12; // tolggle led
  ToggleLED();
  switch(MemType)
  {
    case PM:
      //Write buffer in program memory
      _write_flash24(StartAddr, Buffer.asWord32);
    break;
    case EE:
      //If the EEPROM has to be programmed
      if(UpdateEE)
      {
        _write_eedata_row(StartAddr, Buffer.asWord16);
        _wait_eedata();
      }
    break;
    case CM:
      ;  //CM is programmed at the end, see WriteCMBuffer call
    break;
    default:
      //todo: ERROR
      ;
  }
}

void WriteCMBuffer(void)
//Reserved configuration bits must be programmed as 1
//Unimplemented bits must be programmed with 0
{
  int i;

  SwOffLED();
  for(i=0; i < CMBufIndex; i++)
  {
    switch(CMBuffer[i].Addr.Val32)
    {
      case FOSC:
//        CMBuffer[i].Data.Val32 &= FOSC_MASK;
        //Set the bootloader default configuration (the value in HEX file isn't used)
        CMBuffer[i].Data.Val32 = (CSW_FSCM_OFF & ECIO_PLL8) & FOSC_MASK;
        //When change FOSC configuration register it's necessary a reset
      break;

      case FWDT:
        CMBuffer[i].Data.Val32 &= FWDT_MASK;
      break;

      case FBORPOR:
        CMBuffer[i].Data.Val32 &= FBORPOR_MASK;
      break;
      
      case FGS:
//        CMBuffer[i].Data.Val32 &= FGS_MASK;  Value received is not used
        //FGS register has always a fixed value to avoid dangerous configuration:
        //  Bit 0            GWRP = 1 General segment program memory is not write-protected
        //  Bit 1            GCP  = 1 General segment program memory is not  code-protected
        //  Bit 2            Reserved (must be programmed as '1')
        //  Bit 3.. Bit 15   Unimplememted (must be programmed with '0')
        //
        //  FGS register = 0x0007
        CMBuffer[i].Data.Val32 = 0x0007;
      break;

      case FBS:
        //Is a reserved register (reserved bits must be 1, unimplemented bits must be 0)
        CMBuffer[i].Data.Val32 = 0x310F;
      break;

      case FSS:
        //Is a reserved register (reserved bits must be 1, unimplemented bits must be 0)
        CMBuffer[i].Data.Val32 = 0x330F;
      break;

      case FICD:
        CMBuffer[i].Data.Val32 &= FICD_MASK;
      break;
    }
    //------------------  IMPORTANT --------------------------------
    //For Configuration memory use WriteLatchCM (write only low word)
    //
    WriteLatchCM(CMBuffer[i].Addr.Word.HW,CMBuffer[i].Addr.Word.LW,CMBuffer[i].Data.Word.LW);
    WriteMem(CONFIG_WORD_WRITE);
  }
}

void ErasePM_EE(void)
{
  _prog_addressT FLASH_addr;

  FLASH_addr = START_PM;   
  //Erases the free program memory
  while(FLASH_addr <= ((BOOTLDR_ADDR-2)-(2*PM_ROWSIZE)))
  {
    _erase_flash(FLASH_addr);
    FLASH_addr += PM_ROWSIZE*2;
  }
  if(UpdateEE)
  //If the EEPROM has to be programmed
  {
    _erase_eedata_all();  // erases the entire range of eedata memory
    _wait_eedata();
  }
}



unsigned int ParseCommand(canmsg_t *msg) //, unsigned int SID)
//
// parse command messages and renspond to the sender 
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
// Return 1 if receive a correct command (succes for parser) or in warning case
// Return 0 if there is an error in the command
{
  unsigned char Txdata[8]; 
  unsigned char datalen;
  unsigned char Cmd;
  unsigned int ID;
  unsigned int i;
  int retval;

  //Every CAN message has at least 1 byte in payload
  if(msg->CAN_DLC >= 1)
    Cmd = msg->CAN_Msg_PayLoad[0];
  else
  {
    //Enter in error loop and send error packet
    HaltBootloader(msg->CAN_Msg_Source, PAYLOAD_ERROR);
    return 0;
  }
  

  switch (Cmd)
  {
    case CMD_BOARD:
      if(msg->CAN_DLC >= FIELDLEN_BOARD)
      {
        UpdateEE = msg->CAN_Msg_PayLoad[1];
        comACK (msg->CAN_Msg_Source, Cmd);
        comContinue = 1;
		LoadProgram = 1; //Enable to parse next CMD_ADDRESS and CMD_DATA commands
        return 1;
      }
      else
      {
        //Enter in error loop and send error packet
        HaltBootloader(msg->CAN_Msg_Source, CMD_BOARD_ERROR);
        return 0;
      }
   	break;

    case CMD_ADDRESS:
      //If CAN packet length is correct
      if((msg->CAN_DLC >= FIELDLEN_ADDRESS) && LoadProgram)
      {
        prsLength = msg->CAN_Msg_PayLoad[1];
        prsAddress.Val[0] = msg->CAN_Msg_PayLoad[2];
        prsAddress.Val[1] = msg->CAN_Msg_PayLoad[3];
        prsAddress.Val[2] = msg->CAN_Msg_PayLoad[5];
        prsAddress.Val[3] = msg->CAN_Msg_PayLoad[6];
        //The Hex format address must be divided by 2
        prsAddress.Val32 = prsAddress.Val32 >> 1;

        if(!msg->CAN_Msg_PayLoad[4])
          prsMemoryType = 0;
        else
		  prsMemoryType = 1;	
        //If it's first time
        if(!prsErase)
        {
          //It's the first HEX INT32 address so initialize variables for the first PM row
          //Row address range and the buffer index
          StartAddr = START_PM;
          EndAddr = StartAddr + (PM_ROWSIZE-1)*2;
          MemType = PM;
          //Initializes program memory buffer with 0xFF
          Init_Buffer();
          BufIndex = 0;
          ByteCntr = 0;   //Counter for received bytes
        }
        //If the address is in row range
        if(prsAddress.Val32 >= StartAddr && prsAddress.Val32 <= EndAddr)
        {
          //Calculate the correct index (BufIndex) for the buffer
          if(MemType == PM)
            BufIndex = (prsAddress.Val32-StartAddr)*2;
          else
            if (MemType == EE || MemType == CM)
              BufIndex = (prsAddress.Val32-StartAddr);
        }
        //Else the prsAddress is in a new row
        else
        {
          //If Buffer is not empty
          if(BufIndex != 0)
          {
            WriteBuffer();
            Init_Buffer();
          }
          //Recognize the memory type (PM, EE or CM)
          if(prsAddress.Val32 >= START_PM && prsAddress.Val32 <= END_PM)
            MemType = PM;       //For address in program memory
          else
            if(prsAddress.Val32 >= START_EE && prsAddress.Val32 <= END_EE)
              MemType = EE;     //For address in EEPROM
            else
              if(prsAddress.Val32 >= START_CM && prsAddress.Val32 <= END_CM)
                MemType = CM;   //For address in configuration memory
          
          //Calculate new row address range, buffer index for the new row
          InitAdrIndexBuf();
        }  
        comContinue = 1;
        return 1;
      }
      else
      {
        if(LoadProgram)
        {
          //Enter in error loop and send error packet
          HaltBootloader(msg->CAN_Msg_Source, CMD_ADDRESS_ERROR);
          return 0;
        }
        else return 1; //LoadProgram is 0 so packet is not for this board (error doesn't care)
      }
	break;

    case CMD_END:
        //If Buffer isn't empty
        if(BufIndex)
          //Write buffer on flash
          WriteBuffer();
        comACK (msg->CAN_Msg_Source, Cmd);
        comContinue = 0;  //Set to 0 to interrupt comunication loop
        WriteCMBuffer();  //Write configuration memory buffer
		ClrWdt(); // Reset WDT
        ResetDevice();    //
        return 1;
	break;

    case CMD_DATA:
      if(LoadProgram)
      {
        i=1;
        do
        {
          //If are data for reset jump 
          if((prsAddress.Val32 == 0x00000000) && (BufIndex == 0))
          {
            Buffer.asWord32[BufIndex] = 0x00040000 + BOOTLDR_ADDR;
            BufIndex += 4;
            ByteCntr += 4;
            i += 4;
            prsLength -=4;
          }
          else
          {
            if(((ByteCntr < 4) && (MemType == PM)) ||
               ((ByteCntr < 2) && ((MemType == EE) ||(MemType == CM))))
            {
              if(MemType == CM)
              { 
                if(CMBufIndex >= 7)
                {
                  //Enter in error loop and send error packet
                  HaltBootloader(msg->CAN_Msg_Source, CMBUFINDEX_ERROR);
                  return 0;   //todo: Index over range error
                }
                CMBuffer[CMBufIndex].Data.Val[ByteCntr] = msg->CAN_Msg_PayLoad[i];
                //If the configuration register word is complete
                if(ByteCntr == 1)
                {
                  CMBuffer[CMBufIndex].Addr = prsAddress;  //store the address for config. register
                  CMBufIndex++;
                }
              }  
              else
                Buffer.asByte[BufIndex++] = msg->CAN_Msg_PayLoad[i];
            }
            i++;
            //Count the new byte
            ByteCntr++;
            //For each data read from CAN, decrease data length for HEX packet
            prsLength--;
          }
          //After 4 bytes increments address and reset the byte counter
          if(ByteCntr == 4)
          {
            prsAddress.Val32 += 2;
            ByteCntr = 0;
          }

          if((retval = BufferIsFull()))
          {
            if(retval == -1)
            {
              //Enter in error loop and send error packet
              HaltBootloader(msg->CAN_Msg_Source, BUFFERISFULL_ERROR);
              return 0;  //Return with error
            }
            else
            {
              //Write buffer on memory
              WriteBuffer();
              //Initialize Buffer
              Init_Buffer();
              //Initialize the variables for a new row
              InitAdrIndexBuf();
            }
          }
        }
        while(i < msg->CAN_DLC);
        if(prsLength == 0)
        {
          if(!prsErase)
          {
            ErasePM_EE();
            prsErase = 1;
          }
          //Send command ACK
          comACK (msg->CAN_Msg_Source, Cmd);
        }
        comContinue = 1;
      }
      return 1;
	break;

    case CMD_START:
      if((msg->CAN_DLC >= FIELDLEN_START)&& LoadProgram)
      {
        //If Buffer isn't empty
        if(BufIndex)
          //Write buffer on flash
          WriteBuffer();
        comACK (msg->CAN_Msg_Source, Cmd);
        comContinue = 1;  
        return 1;
      }
      else
      {
        if(LoadProgram)
        {
          //Enter in error loop and send error packet
          HaltBootloader(msg->CAN_Msg_Source, CMD_START_ERROR);
          return 0;
        }
        else
          return 1; //LoadProgram is 0 so packet is not for this board (error doesn't care)
      }
	break;

    case CMD_ENABLELOG:
      if(msg->CAN_DLC >= FIELDLEN_ENABLELOG)
      {
         comACK (msg->CAN_Msg_Source, Cmd);
         if(msg->CAN_Msg_PayLoad[1])
           EnableLOG = 1;
         else
           EnableLOG = 0;
//         comContinue = 1;
         return 1;
      }
      else
      {
        //Enter in error loop and send error packet
        HaltBootloader(msg->CAN_Msg_Source, CMD_ENABLELOG);
        return 0;
      }
 	break;

    case CMD_BROADCAST:
      //Create ID for CAN message
      ID = CAN_MSG_CLASS_LOADER | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | ( msg->CAN_Msg_Source & 0xF );
      
      Txdata[0] = CMD_BROADCAST;

//#ifdef STRAIN
      Txdata[1] = BoardType;  //Board type STRAIN
//#else
//      Txdata[1] = BOARD_TYPE_MAIS;    //Board type MAIS
//#endif

      Txdata[2] = VERSION;            //Firmware version number for BOOTLOADER
      Txdata[3] = BUILD;              //Firmware build number.
      datalen=4; 
      
      SendCanMsg(ID,Txdata,datalen);
      comContinue = 1;
      return 1;
    break;

    default:
    {  
      //If EnableLog is true send a message about an unvailable command warning
      SendCanErrorMsg(msg->CAN_Msg_Source, COMMAND_UNAVAILABLE_WARNING);
      return 1;  //Return no error (command unavailable is a warning)
    }
  }
}

void RunAppl(void)
{
  SwOffLED();  //switch off the LED

  //FOSC register doesn't change
  //ConfAdr.Val32 = FOSC;
  //WriteLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW,FOSC_Val);
  //WriteMem(CONFIG_WORD_WRITE);

  //Restore Configuration Registers: FWDT, FBORPOR
  ConfAdr.Val32 = FWDT;
  WriteLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW,FWDT_Val);
  WriteMem(CONFIG_WORD_WRITE);

  ConfAdr.Val32 = FBORPOR;
  WriteLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW,FBORPOR_Val);
  WriteMem(CONFIG_WORD_WRITE);
  
  //Restart from address 0x100 
  ResetDevice();
}


int main(void)
{ 
  // int i;
  unsigned int match_value;
  unsigned int T1Value;
 
  canmsg_t CAN_Msg;

  char FilterNo,tx_rx_no;
  
  _prog_addressT EE_addr;

  UWord16 ConfVal;
  
//  ConfAdr.Val32 = FOSC;
//  FOSC_Val = ReadLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW);  //Save FOSC configuration register
//  ConfVal = (CSW_FSCM_OFF & ECIO_PLL8) & FOSC_MASK;         //Update register value to run bootloader
//  WriteLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW,ConfVal);
//  WriteMem(CONFIG_WORD_WRITE);

 
  ConfAdr.Val32 = FWDT;
  FWDT_Val = ReadLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW);  //Save FWDT configuration register
  ConfVal = WDT_OFF & FWDT_MASK;                            //Update register value to run bootloader (WD disabled)
  WriteLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW,ConfVal);
  WriteMem(CONFIG_WORD_WRITE);

  ConfAdr.Val32 = FBORPOR;
  FBORPOR_Val = ReadLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW);        //Save FBORPOR configuration register
  ConfVal = (MCLR_EN & PWRT_64 & PBOR_ON & BORV_27) & FBORPOR_MASK;  //Update register value to run bootloader (WD disabled)
  WriteLatchCM(ConfAdr.Word.HW,ConfAdr.Word.LW,ConfVal);
  WriteMem(CONFIG_WORD_WRITE);

   
  InitPorts();
 
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

  //
  // EEPROM Data Recovery
  // 

  // Initialize BoardConfig variable in RAM with the Data EEPROM stored values 
  _init_prog_address(EE_addr, ee_data);
  _memcpy_p2d16(&BoardConfig, EE_addr, sizeof(BoardConfig));
  // todo: checksum verification


// TRISBbits.TRISB12 = 0; // set B12 as out (LED)
// LATBbits.LATB12 = 1;


  // flash led 
  FlashLED();

  //
  // Test STRAIN 
  //   get commands (11 bit ID) via CAN 
  //

  // risponde a un comando: class loader, any source, board address destination
  //  -------  -------  -------  -------  ------- 
  // | 3b     | 4b     | 4b     |                |
  // | class  | Source | Dest   |      ....      |
  //  -------  -------  -------  -------  ------- 
  //   command:
  //     CAN_MSG_CLASS_LOADER xxxx ee_data.EE_CAN_BoardAddress 
  //   mask:
  //     0b111 xxxx 0101 -> mask = 0x70F

  // Load Acceptance filter register
  FilterNo = 0;

//  CAN1SetFilter(FilterNo, CAN_FILTER_SID( (CAN_MSG_CLASS_BOARDTEST|BoardConfig.EE_CAN_BoardAddress) ) // 0x205
//    & CAN_RX_EID_DIS, CAN_FILTER_EID(0));

  CAN1SetFilter(FilterNo, CAN_FILTER_SID( (CAN_MSG_CLASS_LOADER|BoardConfig.EE_CAN_BoardAddress) ) // 0x705
    & CAN_RX_EID_DIS, CAN_FILTER_EID(0));

  // set acceptance mask
  // answer commands from any source
  CAN1SetMask(FilterNo, CAN_MASK_SID(0x70F) & CAN_MATCH_FILTER_TYPE, CAN_MASK_EID(0));

  FilterNo = 1;
  CAN1SetFilter(FilterNo, CAN_FILTER_SID( (CAN_MSG_CLASS_LOADER|0xF) ) // 0x70F
    & CAN_RX_EID_DIS, CAN_FILTER_EID(0));
  CAN1SetMask(FilterNo, CAN_MASK_SID(0x70F) & CAN_MATCH_FILTER_TYPE, CAN_MASK_EID(0));


  // abort any spurious Tx
  CAN1AbortAll();

  // Set Operation Mode  NORMAL
  CAN1SetOperationMode(CAN_IDLE_CON & CAN_CAPTURE_DIS &	CAN_MASTERCLOCK_1 & CAN_REQ_OPERMODE_NOR); 
  
  // Enable CAN IRQ
  //EnableIntCAN1;

  //
  // Timer 1 irq
  // Low frequency operation
  //
  //ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_OFF);
  WriteTimer1(0);
  // with clock set as 10MHz x8PLL 
  // (fosc*8/4)/(prescaler*match) -> 80/4MHz / (256*39062) = 0,5 sec 
  match_value = 39062;
  
  OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP & T1_PS_1_256 & 
    T1_SYNC_EXT_OFF & T1_SOURCE_INT, match_value);
  
  //Disables interrupt timer
  ConfigIntTimer1(T1_INT_OFF);

  while(1)
  {
    // Wait till receive buffer0 or buffer1 contain valid  message
    while(!CAN1IsRXReady(0) && !CAN1IsRXReady(1))
    {
      if(CANLoadWaiting)
      {
        T1Value = ReadTimer1();
        if(T1Value >= match_value)
        {
          WriteTimer1(0);
          ToggleLED();
          if(!(-- T1sec))
          {
            CANLoadWaiting = 0;
            CloseTimer1();  //Disables 16-bit timer interrupt and turns off the timer 
          }
        }
      }
      else if(!comContinue)
        RunAppl();
    }
    //It tests receive buffer 0
    if(CAN1IsRXReady(0))
    {
      // Read received data from receive buffer0 and store it into user defined dataarray
      CAN1ReceiveMessage(&CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_PayLoad[0], 8, 0);
      CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_AsBytes[0] = C1RX0SID; // get Dest, Source and Class 
      CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_AsBytes[1] = C1RX0DLC; // get data length and .... 
      // clear Rx buffer0 full flag
      C1RX0CONbits.RXFUL = 0;
      // increase buff pointer
      if (CAN_Messages_buff_ptr < (CAN_MAX_MESSAGES_IN_BUFF-1))
        CAN_Messages_buff_ptr++;
      else
        EEnqueue(ERR_CAN_RXBUFF_OVERFLOW);
    }
    //It tests receive buffer 1
    else if(CAN1IsRXReady(1))
    {
      // Read received data from receive buffer1 and store it into user defined dataarray
      CAN1ReceiveMessage(&CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_PayLoad[0], 8, 1);
      CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_AsBytes[0] = C1RX1SID; // get Dest, Source and Class 
      CAN_Messages[CAN_Messages_buff_ptr].CAN_Msg_AsBytes[2] = C1RX1DLC; // get data length and .... 

      // clear Rx buffer1 full flag
      C1RX1CONbits.RXFUL = 0;

      // increase buff pointer
      if (CAN_Messages_buff_ptr < (CAN_MAX_MESSAGES_IN_BUFF-1))
        CAN_Messages_buff_ptr++;
      else
        EEnqueue(ERR_CAN_RXBUFF_OVERFLOW);
    }
    memcpy(&CAN_Msg,&CAN_Messages[CAN_Messages_buff_ptr-1],sizeof(CAN_Msg));
    CAN_Messages_buff_ptr--;
    //If there aren't errors in the parsing
    if(ParseCommand(&CAN_Msg))
    {
      //If communication is terminated
      if(!comContinue && !CANLoadWaiting)
        RunAppl();
    }
 
/*
    if(!ParseCommand(&CAN_Msg))
    {
      ;
    }
    else if(!comContinue)
      RunAppl();
*/
  }
}
