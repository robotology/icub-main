#include <p30f4013.h>
#include <can.h>
#include <timer.h>
#include <string.h>
#include <libpic30.h>
#include <dsp.h>

#include "eeprom.h"
#include "errors.h"
#include "can_interface.h"
#include "IIR_filter.h"
#include "utils.h"

extern char filter_enable;
extern char can_enable;
extern char mux_enable;
extern char muxed_chans;
extern char CAN_ACK_EVERY_MESSAGE;

extern struct s_eeprom BoardConfig; 
extern struct s_eeprom /*_EEDATA(1)*/ ee_data;

extern struct s_eeprom BoardConfig; 

// Can RX BUFFER message buffer pointer
static int CAN_Messages_buff_ptr = 0;


/// CAN TX BUFFER 
static int canTxBufferIndex;
static int testcounter;
static canmsg_t canTxBuffer[CAN_TX_SOFTWARE_BUFFER_SIZE];

// can messages buffer
struct canmsg_tag CAN_Messages[CAN_MAX_MESSAGES_IN_BUFF];

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


void InitCanInterface()
{
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
  char tx_rx_no = 0;  // C1TX0CON
  //CAN1SetTXMode(tx_rx_no, CAN_TX_REQ & CAN_TX_PRIORITY_HIGH );
  CAN1SetTXMode(tx_rx_no, CAN_TX_PRIORITY_HIGH );
  CAN1SetRXMode(tx_rx_no, CAN_RXFUL_CLEAR & CAN_BUF0_DBLBUFFER_EN);

  // Configure CAN IRQs 
  ConfigIntCAN1(CAN_INDI_INVMESS_EN & CAN_INDI_WAK_DIS & CAN_INDI_ERR_DIS &
    CAN_INDI_TXB2_DIS & CAN_INDI_TXB1_DIS & CAN_INDI_TXB0_EN &  // CAN TX IRQ ON
    CAN_INDI_RXB1_EN & CAN_INDI_RXB0_EN ,                        // CANRX IRQ ON 
    CAN_INT_PRI_3 & CAN_INT_ENABLE);
    
  canTxBufferIndex=-1;   
  CAN_Messages_buff_ptr=0;
}

//
// CAN IRQ Service Routines
// 
void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void)
{   

 /* 
  // Tx IRQs should be disabled
  if(C1INTFbits.TX0IF)      //  Tx0 Interrupt
    C1INTFbits.TX0IF = 0;   
  else 
  {
    if(C1INTFbits.TX1IF)    //  Tx1 Interrupt
      C1INTFbits.TX1IF = 0; 
  }
  
  */
  	//ready to send a CAN message
    if (C1INTFbits.TX0IF || C1INTFbits.TX1IF || C1INTFbits.TX2IF)
    {
        CAN1_interruptTx();
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
  
  	if (C1INTFbits.WAKIF) C1INTFbits.WAKIF = 0; // Add wake-up handler code
	if (C1INTFbits.ERRIF) C1INTFbits.ERRIF = 0; // Add error handler code
	if (C1INTFbits.IVRIF) C1INTFbits.IVRIF = 0;
    
 	if ( (C1INTF & C1INTE) == 0 ) IFS1bits.C1IF = 0;  
    
  // toggled
  //LATBbits.LATB12 = ~LATBbits.LATB12;
}





//

unsigned char CAN1_send(unsigned int MessageID,unsigned char FrameType,unsigned char Length,unsigned char *Data)
{
    unsigned char i=0;

    //CAN1_TX_DIS; 
    DisableIntCAN1;  
    if (canTxBufferIndex<(CAN_TX_SOFTWARE_BUFFER_SIZE-1))
    {
        canTxBufferIndex++;
        canTxBuffer[canTxBufferIndex].CAN_Msg_ID=MessageID;
        canTxBuffer[canTxBufferIndex].CAN_Poll_Msg_CType=FrameType;
        
        canTxBuffer[canTxBufferIndex].CAN_Msg_Length=Length;
        for (i=0;i<Length;i++)
        {
            canTxBuffer[canTxBufferIndex].CAN_Msg_PayLoad[i]=Data[i];  
        }
        if (canTxBufferIndex==0)
        {
            while (!CAN1IsTXReady(0));
            C1INTFbits.TX0IF=1;
            //	C1TX0CONbits.TXREQ=1;
        }

        EnableIntCAN1;  
        return 0;
    } else
    {
	    
	    //CAN_TX_OVERRUN
        canTxBufferIndex=-1;
        MessageID= 0x100;
        MessageID |= (BoardConfig.EE_CAN_BoardAddress) << 4;
        while (!CAN1IsTXReady(0));    
        CAN1SendMessage((CAN_TX_SID(MessageID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ,
                        (CAN_TX_EID(0)) & CAN_NOR_TX_REQ, 
                        Data, 0,0);
        return -1;
    }
}

void CAN1_interruptTx (void)
{
    unsigned char buffer;
    //CAN1_TX_DIS;
    DisableIntCAN1;
    
    switch (C1CTRLbits.ICODE)
    {
    	case 4: 
    	{
    		C1INTFbits.TX0IF = 0; 
    		buffer=0;
    	}	
    	break;
		
		case 3: 
		{
			C1INTFbits.TX1IF = 0; 
			buffer=1;
		}	
		break;
		
		case 2: 
		{
			C1INTFbits.TX2IF = 0;
			buffer=2; 
		}
		break;
    }
    if (canTxBufferIndex!=-1)
    {
        testcounter++;
        if (testcounter>=255) testcounter=0;
       CAN1SendMessage((CAN_TX_SID(canTxBuffer[canTxBufferIndex].CAN_Msg_ID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ,
                        (CAN_TX_EID(0)) & CAN_NOR_TX_REQ,
                        canTxBuffer[canTxBufferIndex].CAN_Msg_PayLoad,
                        canTxBuffer[canTxBufferIndex].CAN_Msg_Length,
                        buffer);
  
        canTxBufferIndex--;
        if (canTxBufferIndex!=-1)
        //CAN1_TX_DIS;
        {
            //C1INTFbits.TX0IF;
            EnableIntCAN1;
        }
    }
    EnableIntCAN1;
}


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
void ParseCommand() //, unsigned int SID)


{
	 canmsg_t *msg;
	 canmsg_t CAN_Msg;
	 unsigned char Txdata[9]; 
 	 char datalen;
  	 //char tx_rx_no;
  	unsigned int i,match_value;
#ifdef MAIS
#else 
 	 unsigned int j,tmp,tout;
#endif

	if (CAN_Messages_buff_ptr != 0)
    // Something in the CAN message buffer
    {
      // Read received data from receive buffer and store it into user defined dataarray

      //mask CANRxIRQ for atomic buffer manipulation
      DisableIntCAN1;
      memcpy(&CAN_Msg,&CAN_Messages[CAN_Messages_buff_ptr-1],sizeof(CAN_Msg));
      msg=&CAN_Msg;
      CAN_Messages_buff_ptr--;
      EnableIntCAN1;
	  //tx_rx_no=0;
	  switch (msg->CAN_Per_Msg_Class)
	  {
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
/////                                          POLLING CLASS MESSAGES                                             ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case (CAN_MSG_CLASS_POLLING>>8):
		{
		  switch (msg->CAN_Per_Msg_PayLoad[0])
		  {
		    case CAN_CMD_SET_BOARD_ADX: // set board CAN address 
		    {     
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
		    }  
		    break;
		
		    case CAN_CMD_SET_IIR: // Set IIR Filter parameters: 0x205 len 4  data 1 i MSB LSB 
		    {
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
			}   
		    break;
	
			case CAN_CMD_GET_MATRIX_RC: //get MATRIX
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
	
			case CAN_CMD_GET_CH_ADC:  //get ADC channel
			{
	          if(msg->CAN_Per_Msg_PayLoad[1] < 6)
	          {
			    Txdata[0] = CAN_CMD_GET_CH_ADC; 
			    Txdata[1] = msg->CAN_Per_Msg_PayLoad[1];  
			    Txdata[2] = (BoardConfig.EE_AN_ChannelValue[msg->CAN_Per_Msg_PayLoad[1]]+0x7FFF) >> 8; 
		   	    Txdata[3] = (BoardConfig.EE_AN_ChannelValue[msg->CAN_Per_Msg_PayLoad[1]]+0x7FFF) & 0xFF; 
				/*
				Txdata[2] = BoardConfig.EE_AN_ChannelValue[msg->CAN_Per_Msg_PayLoad[1]] >> 8; 
				Txdata[3] = BoardConfig.EE_AN_ChannelValue[msg->CAN_Per_Msg_PayLoad[1]] & 0xFF; 
				*/
				datalen=4;	            
	          }
			}
			break;
		
		    case CAN_CMD_SET_MATRIX_RC: //set i,j value of transform. matrix:
		    { 
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
		    }  
		    break;
	
			case CAN_CMD_CALIBRATE_OFFSET: //set the calibration offset
			{	
		#ifdef MAIS
		    // 
	    	// MAIS
		    // 
		#endif
		
		#ifdef STRAIN
				unsigned int dat;
				dat = msg->CAN_Per_Msg_PayLoad[1]<<8 | msg->CAN_Per_Msg_PayLoad[2];  
				CalibrateOffset(dat);
		
		#endif				
			}
			break;
		
		    case CAN_CMD_SET_CH_DAC:  //  set DAC value 0x205 len 4  data 4 ch msb lsb
		    {	       
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
		     }
		    break;
		
		    case CAN_CMD_SET_TXMODE: // set continuous or on demand tx  0x205 len 2  data 7 0/1
		    {
		      if(msg->CAN_Per_Msg_PayLoad[1]==0)
		      { 
				can_enable=1;
		        EnableIntT2;
		        EnableIntT1;
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
		        DisableIntT1;
		        DisableIntT2;
		        DisableIntT3;
		        // ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_OFF);
		      }
		      datalen=0;
		    }  
		    break;
	
		    case CAN_CMD_FILTER_EN: //enabling filter
		    {
			      if(msg->CAN_Per_Msg_PayLoad[1]==0)
			      { 
			        filter_enable=0;
			      }
				  else
			      {
			        filter_enable=1;
			      }
			      datalen=0;
		    }  
		    break;
	
		    case CAN_CMD_MUX_EN:  //enabling multiplexer
		    {
	    #ifdef STRAIN
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
		 #endif 
		      datalen=0;
		 
		    }  
		    break;
	
		    case CAN_CMD_MUX_NUM: //set multiplexer number
		    {
		#ifdef STRAIN 
		      muxed_chans=msg->CAN_Per_Msg_PayLoad[1];
		#endif      
		      datalen=0;	      
		    }
		    break;
		    
		    case CAN_CMD_SET_CANDATARATE: // set datarate for transmission in milliseconds 
		    							  // 0x205 len 2  data 8 n
		    {
		    
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
		    }
		    break;
		    
		    case CAN_CMD_SET_RESOLUTION: // set data resolution   
		    							  // 0x205 len 2  data 0x10 n
		    {
		    
		      if (msg->CAN_Per_Msg_PayLoad[1]==HESDATA_IS_16_BIT)
		      {
		      	HESDATA_RESOLUTION=HESDATA_IS_16_BIT;
		      }
		      if (msg->CAN_Per_Msg_PayLoad[1]==HESDATA_IS_12_BIT)
		      {
		      	HESDATA_RESOLUTION=HESDATA_IS_12_BIT;
		      }
		      if (msg->CAN_Per_Msg_PayLoad[1]==HESDATA_IS_8_BIT)
		      {
		      	HESDATA_RESOLUTION=HESDATA_IS_8_BIT;
		      }
		     
		    }
		    break;
		    case CAN_CMD_SELECT_ACTIVE_CH:    	// select witch channel is sampled and CANsmitted
		   									    // 0x205 len 2  data 5 0bxx000001
		    {
		 
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
		  }    
		  break;
		
		    case CAN_CMD_SAVE2EE: // Save configuration data to EE
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
/////                                          CANLOADER CLASS MESSAGES                                           ///
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		case (CAN_MSG_CLASS_LOADER>>8):
		{
		//	DisableIntT1;
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
					/*	do 
						{
							tout++;
						}
						while (!CAN1IsTXReady(0) && tout<=2000) ;
						if (tout!=2000 )
						{
					*/
							Txdata[1]=tmp;
							for (j=0; j<4; j++)	Txdata[2+j] = BoardConfig.EE_AdditionalInfo[j+tmp*4]; 	
							CAN1_send(i,0,datalen,Txdata);	
						//	CAN1SendMessage((CAN_TX_SID(i)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, Txdata,datalen,0); 
					//	} 	
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
	    CAN1_send(i,0,datalen,Txdata);
//	    CAN1SendMessage((CAN_TX_SID(i)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ, (CAN_TX_EID(0x0)) & CAN_NOR_TX_REQ, Txdata,datalen,0); 
	  }
   }
}