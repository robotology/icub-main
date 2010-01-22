#include <can.h>
#include "can_interface.h"
#include "options.h"
#include "eeprom.h"
#include "timer.h"

/// CAN TX BUFFER 
static int canTxBufferIndex;
static int canRxBufferIndex;
static int testcounter;
static canmsg_t canTxBuffer[CAN_TX_SOFTWARE_BUFFER_SIZE];

extern void main();

extern struct s_eeprom BoardConfig; 


extern unsigned int _board_ID;
extern unsigned char board_MODE;
extern unsigned char new_board_MODE;
unsigned int board_TIMER;
extern unsigned int PW_CONTROL; // 0x1B0 for 120 decim
extern unsigned int TIMER_VALUE;
extern unsigned int CONFIG_TYPE;
extern char _additional_info[32];
extern unsigned int ConValue[2];
// can RX messages buffer

static canmsg_t CANRxBuffer[CAN_RX_SOFTWARE_BUFFER_SIZE];

void CAN_Init()
{
    char FilterNo = 0, tx_rx_no = 0;
    unsigned int i=0;
    unsigned int k=0;
    //Interrupt configuration CAN1;
    // RX0IE    RX1IE    TX0IE 
    // 
    ConfigIntCAN1(0x0007, CAN_INT_PRI_2 & CAN_INT_ENABLE);   
    canTxBufferIndex=-1;
    canRxBufferIndex=0;
    testcounter=0;

    for (i=0;i< CAN_RX_SOFTWARE_BUFFER_SIZE-1;i++)
    {
        for (k=0;k< 8;k++)
        {
            CANRxBuffer[i].CAN_data[k]=0;
        }
		CANRxBuffer[i].CAN_messID=_board_ID;
    }
/* Settaggio del Operation Mode in CONFIG (permette la configurazione) */
    CAN1SetOperationMode(   CAN_IDLE_CON &
                            CAN_MASTERCLOCK_1 &
                            CAN_REQ_OPERMODE_CONFIG &
                            CAN_CAPTURE_DIS);    

    while (C1CTRLbits.OPMODE <=3);       // Attende che le impostazioni del modulo di CAN siano caricate (CONFIG)

    /* Configurazione del modulo CAN */
    //1MB 1 3 3 1 or 1 5 3 1 Fosc 10x8 80 Mhz
    CAN1Initialize(             CAN_SYNC_JUMP_WIDTH1 &                     // 1 o 2 sembra non cambiare nulla
                                CAN_BAUD_PRE_SCALE(1),              // Adatta la baud rate al clock del dspic
                                CAN_WAKEUP_BY_FILTER_DIS  &         // Disabilita un filtro passa basso sull'ingresso
                                CAN_PHASE_SEG2_TQ(5) &              // Definisce il punto in cui
                                CAN_PHASE_SEG1_TQ(3) &              // si campiona il segnale
                                CAN_PROPAGATIONTIME_SEG_TQ(1) & // Setta un tempo di propagazione
                                CAN_SEG2_FREE_PROG &
                                CAN_SAMPLE1TIME);         

/* Configurazione dei filtri e delle maschere in questo caso RXF0 RXM0 */

    CAN1SetFilter(              FilterNo,                       // Numero del filtro
                                CAN_FILTER_SID(0) &             // Valore memorizzato nel filtro sid (standard data frame)
                                CAN_RX_EID_DIS,             // Abilita/disabilita il filtro
                                CAN_FILTER_EID(0));             // Valore memorizzato nel filtro eid (extended data frame)

    CAN1SetMask(                FilterNo,                       // Numero della maschera
                                CAN_MASK_SID(0) &                // Valore memorizzato nella maschera sid (standard data frame)
                                CAN_MATCH_FILTER_TYPE,  // Controlla il formato del pacchetto dell'informazione
                                CAN_MASK_EID(0));               // Valore memorizzato nella maschera eid (extended data frame)

/* Impostazione modulo trasmissione e ricezione */    

    CAN1SetTXMode(          tx_rx_no,                       // Setta il buffer di trasmissione
                            CAN_TX_STOP_REQ &               // Leva la richiesta di trasmissione         
                            CAN_TX_PRIORITY_HIGH );     // Imposta la priorita del nodo?

    CAN1SetRXMode(          tx_rx_no,                       // Setta il buffer di ricezione
                            CAN_RXFUL_CLEAR &           // Azzera il flag RXFUL          
                            CAN_BUF0_DBLBUFFER_EN);  // Attiva la dipendenza dei due buffer?  

    while (!CAN1IsTXReady(0));       // Funzione che ritorna lo stato del modulo di trasmissione

/* Settaggio del Operation Mode in NORMAL (trasmissione e ricezione) */

    CAN1SetOperationMode(   CAN_IDLE_CON & 
                            CAN_CAPTURE_DIS &
                            CAN_MASTERCLOCK_1 &
                            CAN_REQ_OPERMODE_NOR); 
    DisableIntCAN1;
}


/**
 * copy the message to the canTxBuffer. 
 *
 * @param MessageID is the ID of the message (aka arbitration ID).
 * @param FrameType is either DATA_FRAME of REMOTE_FRAME.
 * @param Length is the lenght of the message (0 to 8).
 * @param Data is the pointer to Data (8 bytes).
 */

unsigned char CAN1_send(unsigned int MessageID,unsigned char FrameType,unsigned char Length,unsigned char *Data)
{
    unsigned char i=0;

    //CAN1_TX_DIS; 
    DisableIntCAN1;  
    if (canTxBufferIndex<(CAN_TX_SOFTWARE_BUFFER_SIZE-1))
    {
        canTxBufferIndex++;
        canTxBuffer[canTxBufferIndex].CAN_messID=MessageID;
        canTxBuffer[canTxBufferIndex].CAN_frameType=FrameType;
        canTxBuffer[canTxBufferIndex].CAN_length=Length;
        for (i=0;i<Length;i++)
        {
            canTxBuffer[canTxBufferIndex].CAN_data[i]=Data[i];  
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
        canTxBufferIndex=-1;
        MessageID= 0x100;
        MessageID |= (CAN_BOARD_ID) << 4;
        MessageID |= CAN_BCAST_OVERFLOW;
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
        
    if ((C1CTRLbits.ICODE==0b100))
    {
        buffer=0;       
        C1INTFbits.TX0IF=0;
    }

    if (canTxBufferIndex!=-1)
    {
        CAN1SendMessage((CAN_TX_SID(canTxBuffer[canTxBufferIndex].CAN_messID)) & CAN_TX_EID_DIS & CAN_SUB_NOR_TX_REQ,
                        (CAN_TX_EID(0)) & CAN_NOR_TX_REQ,
                        canTxBuffer[canTxBufferIndex].CAN_data,
                        canTxBuffer[canTxBufferIndex].CAN_length,
                        buffer);

        canTxBufferIndex--;
    }

}

void CAN1_interruptRx (void)
{
    unsigned int rxbuffer=0;
    unsigned int i,datalen;
    if (C1INTFbits.RX0IF)
    {
        C1INTFbits.RX0IF = 0;   // clear Rx0 Interrupt
        rxbuffer=0;
    } else
    {
        if (C1INTFbits.RX1IF)
        {
            C1INTFbits.RX1IF = 0; 
            rxbuffer=1;
        }
    }
    //If the Interrupt is due to Receive1 of CAN1 Clear the Interrupt

    //Read received data from receive buffer and store it into user defined dataarray

    // increase buff pointer
    if (canRxBufferIndex < (CAN_RX_SOFTWARE_BUFFER_SIZE-1))
    {
        canRxBufferIndex++;
    }
    datalen=8;
    for (i=0;i<datalen;i++)
    {
        if (rxbuffer==0)
        {
            CANRxBuffer[canRxBufferIndex-1].CAN_data[i]=*((unsigned char *)&C1RX0B1 + i);
			CANRxBuffer[canRxBufferIndex-1].CAN_messID=C1RX0SIDbits.SID;//((C1RX0SID & 0xffc)>>2);
			CANRxBuffer[canRxBufferIndex-1].CAN_length= C1RX0DLCbits.DLC;
        } else
        {
		CANRxBuffer[canRxBufferIndex-1].CAN_data[i]=*((unsigned char *)&C1RX1B1 + i);
		CANRxBuffer[canRxBufferIndex-1].CAN_messID=C1RX1SIDbits.SID;//((C1RX1SID & 0xffc)>>2);
		CANRxBuffer[canRxBufferIndex-1].CAN_length=C1RX1DLCbits.DLC;
    
        }
    }
    // CAN1_send(7, 1, 4, CANRxBuffer[canRxBufferIndex-1].CAN_data);   
    // put the message in the buffer  
    // clear Rx buffer0 full flag
    C1RX0CONbits.RXFUL = 0;
    //ERROR;
}

int CAN1_handleRx (unsigned int board_id)
{
    unsigned int IdTx;
    unsigned int i;
    //unsigned char prova[8]={1,2,3,4,5,6,7,8};

    while (CAN1_getRxbufferIndex()>0)
    {

        //  bootloader messages, 
        // ID 0x700 (100 0000 0000b) message class = bootloader message
        if (((CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x700) == 0x700) ||((CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x200) == 0x200))
        {
	        DisableIntT1;  
            IdTx = (CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x0700);         
            IdTx |=  ((_board_ID) << 4);
            IdTx |= ((CANRxBuffer[canRxBufferIndex-1].CAN_messID >> 4) & 0x000F);     

            switch (CANRxBuffer[canRxBufferIndex-1].CAN_data[0])
            {
            //request for board type and firmware versione
            case CMD_BROADCAST:             

                CANRxBuffer[canRxBufferIndex-1].CAN_data[0] = 0xFF;
                CANRxBuffer[canRxBufferIndex-1].CAN_data[1] = CURRENT_BOARD_TYPE;  
                CANRxBuffer[canRxBufferIndex-1].CAN_data[2] = (_version & 0xff00) >> 8;  // firmware version.	
                CANRxBuffer[canRxBufferIndex-1].CAN_data[3] = _version & 0x00ff;       // firmware revision
                CANRxBuffer[canRxBufferIndex-1].CAN_data[4] = _build_number & 0x00ff;    // build number
                CAN1_send(IdTx, 1, 5, CANRxBuffer[canRxBufferIndex-1].CAN_data);                 

                // ping command	
            case 4:             

                if (_board_ID == (CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x000F))
                {
                    CANRxBuffer[canRxBufferIndex-1].CAN_data[0] = 4;
                    CANRxBuffer[canRxBufferIndex-1].CAN_data[1] = 1; 
                    CAN1_send(IdTx, 1, 2, CANRxBuffer[canRxBufferIndex-1].CAN_data);
                }
                break;

                // restart bootloader	
            case CMD_BOARD: //
            	    CANRxBuffer[canRxBufferIndex-1].CAN_data[0] = 0;
                    CANRxBuffer[canRxBufferIndex-1].CAN_data[1] = 1; 
                    CAN1_send(IdTx, 1, 2, CANRxBuffer[canRxBufferIndex-1].CAN_data);
    
                if (_board_ID == (CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x000F))
                    //
                    //          JUMP to bootloader
                    			asm ("reset");
                    //                    
                    break;
			case CAN_SET_BOARD_ID:  //SETTING THE BOARD ID
					
					BoardConfig.EE_CAN_BoardAddress= CANRxBuffer[canRxBufferIndex-1].CAN_data[1];
					_board_ID=BoardConfig.EE_CAN_BoardAddress;
					SaveEepromBoardConfig();
			break;	
                // set additional info					
            case CAN_SET_ADDITIONAL_INFO:
                if (_board_ID == (CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x000F))
                {
	              	static unsigned char addinfo_part=0;			
			      	for ( i=0; i<4; i++)
					{
						BoardConfig.EE_AdditionalInfo[CANRxBuffer[canRxBufferIndex-1].CAN_data[1]*4+i] = CANRxBuffer[canRxBufferIndex-1].CAN_data[2+i]; 
					}
					if (addinfo_part==7)
					{
	 					addinfo_part=0;
					    SaveEepromBoardConfig();
					}
					else
					{					
						addinfo_part++;
					}
                }
            break;

                // get additional info	
            case CAN_GET_ADDITIONAL_INFO:
                if (_board_ID == (CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x000F))
                {
	                for (i=0;i<32;i++)
                    {
                    _additional_info[i]=BoardConfig.EE_AdditionalInfo[i]; 
                    }
                    can_send_additional_info(); 
                }
                break;
            default:
                break;
            }    

        }//if 

        //  Polling message, 
        // ID 0x100 (001 0000 0000b) message class = polling message                
        if ((CANRxBuffer[canRxBufferIndex-1].CAN_messID & 0x70F) == (0x200 | BoardConfig.EE_CAN_BoardAddress))
		{
			// Board Setup or Calibration Commands    
    		switch (CANRxBuffer[canRxBufferIndex-1].CAN_data[0])
			{
				case CAN_TACT_SETUP:
				{
					
					// board_MODE
					//
					// SIXTEEN_BITS 0x0
					// EIGHT_BITS   0x1
					// TEN_BITS     0x2
					// CALIB        0x3
					//
					new_board_MODE=	CANRxBuffer[canRxBufferIndex-1].CAN_data[1]&0xF;        
					
					//CONFIG_TYPE
					//
					// CONFIG_SINGLE  0x00 (12 indipendent measurements)
					// CONFIG_THREE   0x10 (3 macro areas) 
					// CONFIG_ALL     0x20 (1 macro area) 
					
					CONFIG_TYPE=(CANRxBuffer[canRxBufferIndex-1].CAN_data[1]&0xF0)>>4;
					
					switch (CONFIG_TYPE)
					{ 
						case CONFIG_SINGLE:
						{
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x1)
							{	
								PW_CONTROL= 0xC0B0; // for 256 decim
								TIMER_VALUE=0x3A00; // for 256 decim
							}
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x2)
							{
								PW_CONTROL= 0xC1B0; // for 128 decim
								TIMER_VALUE=0x2D00; // for 128 decim
							}
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x3)
							{
								PW_CONTROL= 0xC2B0; // for 64 decim
								TIMER_VALUE=0x1A00; // for 64 decim
							}
						}	
						break;
						case CONFIG_THREE:
						{
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x1)
							{	
								PW_CONTROL= 0x20; // for 256 decim
					  			TIMER_VALUE=0x1A00; // for 256 decim
							}
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x2)
							{
								PW_CONTROL= 0x120; // for 128 decim
								TIMER_VALUE=0x1A00; // for 128 decim
							}
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x3)
							{
								PW_CONTROL= 0x220; // for 64 decim
								TIMER_VALUE=0x1A00; // for 64 decim
							}
						}	
						break;
						case CONFIG_ALL:
						{
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x1)
							{	
								PW_CONTROL= 0x00; // for 256 decim
								TIMER_VALUE=0x1A00; // for 256 decim
							}
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x2)
							{
								PW_CONTROL= 0x100; // for 128 decim
								TIMER_VALUE=0x1A00; // for 128 decim
							}
							if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[2]&0xf)==0x3)
							{
								PW_CONTROL= 0x200; // for 64 decim
								TIMER_VALUE=0x1A00; // for 64 decim
							}
						}	
						break;
					}
					if ((CANRxBuffer[canRxBufferIndex-1].CAN_data[3]&0xF)==0x3)
					{
						TIMER_VALUE=(CANRxBuffer[canRxBufferIndex-1].CAN_data[6]) | (CANRxBuffer[canRxBufferIndex-1].CAN_data[7]<<8);	
					}
		
						ConValue[0]=(CANRxBuffer[canRxBufferIndex-1].CAN_data[4]) | (CANRxBuffer[canRxBufferIndex-1].CAN_data[5]<<8);
						ConValue[1]=(CANRxBuffer[canRxBufferIndex-1].CAN_data[4]) | (CANRxBuffer[canRxBufferIndex-1].CAN_data[5]<<8);
					
					//main();
					board_MODE=CALIB;
					EnableIntT1;
				}
				break;

				case CAN_TACT_CALIB:
				{	
					DisableIntCAN1;
					main();
				}
				break;
			}	
 	   	
	
	    }
        DisableIntCAN1;
        canRxBufferIndex--;
        EnableIntCAN1;
    }
    return 0;
}

int  CAN1_getRxbufferIndex()
{
    return canRxBufferIndex;
}

void CAN1_RxbufferIndex_dec() //canRxBufferIndex --
{
    DisableIntCAN1;
    if (canRxBufferIndex>0)
    {
        canRxBufferIndex--;
    }
    EnableIntCAN1;

}

/***********************************************************************/
void can_send_additional_info()
{
    unsigned int j = 0;
    unsigned int counter = 0;
    unsigned char CAN_DATA [8]={0,0,0,0,0,0,0,0};
    unsigned int IdTx=0;
   // IdTx >>= 4; 
   // IdTx &= 0xffffff0f; 
    IdTx |= (_board_ID << 4); 
    IdTx |= 0x700;
    CAN_DATA[0]=CAN_GET_ADDITIONAL_INFO;
    for (counter = 0 ; counter < 8; counter++)
    {
        CAN_DATA[1]=counter;
        for (j=0; j<4; j++) CAN_DATA[2+j] = _additional_info[j+counter*4];      
        CAN1_send (IdTx, 1, 6, CAN_DATA);  
    }       
}
/***********************************************************************/
void can_receive_additional_info()
{
    unsigned int value = 0; 
    unsigned int j = 0;
    unsigned int CAN_DATA [8]={0,0,0,0,0,0,0,0};

    value = CAN_DATA[1];    
    for (j=0; j<4; j++) _additional_info[value*4+j] = CAN_DATA[2+j];       
}
