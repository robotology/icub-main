#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H


extern char VERSION;   
extern char RELEASE;  
extern char BUILD;     
extern char BOARD_TYPE;

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
#define CAN_MAX_MESSAGES_IN_BUFF 10

//////////////////////////////////////////////////////////////
// CANLOADER MESSAGES, CLASS 0x02
// Incoming messages
#define CAN_CMD_NONE              0x0
#define CAN_CMD_SET_BOARD_ADX     0x32 // Set board CAN address
#define CAN_CMD_SET_IIR           0x1  // select IIR filters parameters 
#define CAN_CMD_SET_MATRIX_RC     0x3  // Set SG to TF trasformation matrix 
#define CAN_CMD_SET_CH_DAC        0x4  // set DAC for channel x
#define CAN_CMD_SELECT_ACTIVE_CH  0x5  // set active channels (activation mask) only active channels are transmitted
#define CAN_CMD_CALIBRATE_OFFSET  0x6  // set the calibration offset
#define CAN_CMD_SET_TXMODE        0x7  // set continuous/on demand transmission mode
#define CAN_CMD_SET_CANDATARATE   0x8  // set board CAN speed in milliseconds minimum, datarate 210ms  
#define CAN_CMD_SAVE2EE           0x9  // save Config to EE
#define CAN_CMD_GET_MATRIX_RC     0xA  // Get TF trasformation matrix 
#define CAN_CMD_GET_CH_DAC        0xB  // Get DAC for channel x
#define CAN_CMD_GET_CH_ADC        0xC  // Get ADC for channel x
#define CAN_CMD_FILTER_EN         0xD  // ENABLE/DISABLES FILTER
#define CAN_CMD_MUX_EN         	  0xE  // ENABLE/DISABLES MUX
#define CAN_CMD_MUX_NUM           0xF
#define CAN_CMD_NONE              0x0
#define CAN_CMD_SET_RESOLUTION    0x10 //set data resolution
#define CAN_CMD_SET_MATRIX_G      0x11 //set matrix gain
#define CAN_CMD_GET_MATRIX_G      0x12 //get matrix gain

//////////////////////////////////////////////////////////////
// BROADCAST MESSAGES, CLASS 0x03
// Transmitted Torque values 3*16 bit 
#define CAN_CMD_FORCE_VECTOR      0xA // 010 SOURCE DEST 0xA t1 t1 t2 t2 t3 t3
#define CAN_CMD_TORQUE_VECTOR     0xB // 010 SOURCE DEST 0xB f1 f1 f2 f2 f3 f3
// hall effect sensors from 0 to 3*16bits
#define CAN_CMD_HES0TO3           0xC // 010 SOURCE DEST 0xC h0 h0 h1 h1 h2 h2 h3 h3
// hall effect sensors from 4 to 7*16bits
#define CAN_CMD_HES4TO7           0xD // 010 SOURCE DEST 0xC h4 h4 h5 h5 h6 h6 h7 h7
// hall effect sensors from 8 to 11*16bits
#define CAN_CMD_HES8TO11          0xE // 010 SOURCE DEST 0xC h8 h8 h9 h9 h10 h10 h11 h11
// hall effect sensors from 12 to 14*16bits
#define CAN_CMD_HES12TO14         0xF // 010 SOURCE DEST 0xC h12 h12 h13 h13 h14 h14 h15 h15

// hall effect sensors from 0 to 7*8bits
#define CAN_CMD_HES0TO7           0xC // 010 SOURCE DEST 0xC h0 h1 h2 h3 h4 h5 h6 h7
// hall effect sensors from 8 to 14*8bits
#define CAN_CMD_HES8TO14          0xD // 010 SOURCE DEST 0xD h8 h9 h10 h11 h12 h13 h14 

// hall effect sensors from 0 to 4*12bits
#define CAN_CMD_HES0TO4           0xC // 010 SOURCE DEST 0xC h0 h1 h2 h3 h4 
// hall effect sensors from 5 to 9*12bits
#define CAN_CMD_HES5TO9          0xD // 010 SOURCE DEST 0xD h5 h6 h7 h8 h9 
// hall effect sensors from 10 to 14*12bits
#define CAN_CMD_HES10TO14          0xE // 010 SOURCE DEST 0xD h10 h11 h12 h13 h14 

//////////////////////////////////////////////////////////////
// CANLOADER MESSAGES, CLASS 0x07
#define CMD_BOARD                 0x00 //Jump to CAN loader code. Replay with 0x00, 0x01
#define CMD_ADDRESS               0x01 //Address packet Data[1] is the length; Data[2] and Data[3] are the address; Data[4], is the block type (0x00 = program, 0x01 = data)
#define CMD_START                 0x02 //Start the program ????? Replay with 0x02, 0x01
#define CMD_DATA                  0x03 //Data packet: 6 bytes of payload are flashed to memory. Replay with 0x03, 0x01
#define CMD_END                   0x04 //The program is terminated. Replay with 0x04, 0x01 ????
#define CMD_ERROR                 0x05 //Command not used
#define CMD_GET_ADDITIONAL_INFO   0x0C // Get Additional Info
#define CMD_SET_ADDITIONAL_INFO	  0x0D // Set Additional Info
#define CMD_BROADCAST             0xFF // Request for board type and firmware version


// can bus message structure
typedef struct canmsg_tag
{
  union
  {
    struct // for polling messages
    {
      unsigned CAN_Poll_RxIDE            :1;
      unsigned CAN_Poll_SRR              :1;
      unsigned CAN_Poll_Msg_Dest         :4;  // Destination
      unsigned CAN_Poll_Msg_Source       :4;  // Source
      unsigned CAN_Poll_Msg_Class        :3;  // Class of message
      unsigned Poll_NotUsed              :3;
      unsigned char CAN_Poll_Msg_CType;       // C+Type
      unsigned char CAN_Msg_Length;           // Message Length
      unsigned char CAN_Poll_Msg_PayLoad[8];  // Payload 
    };
    struct // for periodic messages
    {
  	  unsigned CAN_Per_RxIDE             :1;
	  unsigned CAN_Per_SRR               :1;
	  unsigned CAN_Per_Msg_Type          :4;  // Destination
	  unsigned CAN_Per_Msg_Source        :4;  // Source
	  unsigned CAN_Per_Msg_Class         :3;  // Class of message
	  unsigned Per_NotUsed               :3; 
	  unsigned char CAN_Poll_Msg_CType;       // C+Type
	  unsigned char CAN_Msg_Length;           // Message Length
	  unsigned char CAN_Per_Msg_PayLoad[8];   // Payload 
    };
    struct
    {
	  unsigned CAN_Poll_RxIDE          :1;
	  unsigned CAN_Poll_SRR            :1;
	  unsigned CAN_Msg_ID              :11; 
	  unsigned NotUsed                 :3;
	  unsigned char CAN_Poll_Msg_CType;       // C+Type
	  unsigned char CAN_Msg_Length;           // Message Length
	  unsigned char CAN_Msg_PayLoad[8];   // Payload  
    };
    //TODO: verificare se è sbagliato  CAN_Msg_AsBytes[9]. Facendo le somme dei precedenti sembra essere CAN_Msg_AsBytes[10]
    unsigned short CAN_Msg_AsBytes[12];   // Message data formatted as bytes 
  };
} canmsg_t;

//extern int CAN_Messages_buff_ptr;
extern struct canmsg_tag CAN_Messages[CAN_MAX_MESSAGES_IN_BUFF];

void InitCanInterface();
void SetBoardCanFilter();
unsigned char CAN1_send(unsigned int MessageID,unsigned char FrameType,unsigned char Length,unsigned char *Data);
void CAN1_interruptTx (void);
void CAN1_interruptRx (void);
void ParseCommand();
#endif

#define SendCanProblem() {  i = (msg->CAN_Per_Msg_Class << 8 ) | ( BoardConfig.EE_CAN_BoardAddress << 4 ) | ( msg->CAN_Poll_Msg_Source ); datalen = 4; Txdata[0]=msg->CAN_Per_Msg_PayLoad[0];Txdata[1] ='B';Txdata [2] ='U';Txdata[3] ='G';}
