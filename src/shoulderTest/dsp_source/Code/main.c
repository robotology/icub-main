/*
 * firmware/controller application.
 *
 */

#include "dsp56f807.h"
#include "asc.h"
#include "SPI1.h"
#include "SSI1.h"
#include "EEPROM.h"
#include "pwmc0.h"
#include "pwmc1.h"

#include "can1.h"
//#include "qd0.h"
//#include "qd1.h"
#include "ti1.h"
//#include "ifsh1.h"
//#include "ad.h"

#include "controller.h"
#include "messages.h"

#define _version 0x1010 
/*
 * prototypes.
 */
void isrIRQA ();
//int get_flash_addr (void);
extern void bootStart(void);  /* Defined in linker.cmd file */ 

/*
 * test irs.
 */
#pragma interrupt saveall
void isrIRQA ()
{
	AS1_sendCharSafe('*');
}

/* stable global data */
bool _calibrated[JN] = { false, false };
bool _pad_enabled[JN] = { false, false };
bool _verbose = false;

//vars

unsigned char c=0;
Int16 _flash_addr = 0;


/* CAN bus communication global vars */
canmsg_t can_fifo[CAN_FIFO_LEN];
Int16 write_p = 0;
Int16 read_p = -1;					/* -1 means empty, last_read == fifo_ptr means full */
canmsg_t _canmsg;					/* buffer to prepare messages for send */

byte	_board_ID = DEFAULT_BOARD_ID;	/* */
byte	_general_board_error = ERROR_NONE;

volatile bool _wait = true;				/* wait on timer variable */

extern bool _ended[];					/* trajectory completed flag */
#define IS_DONE(jj) (_ended[jj])

/* Local prototypes */

void set_can_masks(void);
byte can_receive();
void ChipSelectInit();
void ChipSelectEnable(byte);
void ChipSelectDisable();

void set_can_masks()
{
	Int32 acceptance_code = 0x0;
	Int32 acceptance_code2 = 0x0;
	word temporary;
		

	CAN1_setAcceptanceMask (0x1f1e1f1e, 0x1f1e1f1e);
	//acceptance_code = 0x00100010;
	temporary = _board_ID >> 3;
	temporary |= (_board_ID << 13);
	temporary &= (0xff1f);
	acceptance_code = L_deposit_h (temporary);
	acceptance_code |= temporary;
	acceptance_code2 = 0xe0e10000;
	temporary = _board_ID >> 3;
	temporary |= (_board_ID << 13);
	temporary |= (0x00e0);
	acceptance_code2 |= temporary;	
	CAN1_setAcceptanceCode (acceptance_code, acceptance_code2);
	
}

dword BYTE_C(byte x4, byte x3, byte x2, byte x1)
{
	dword ret;
	word *p = (word *)&ret;
	*p++ = __shl(x3,8) | x4;
	*p++ = __shl(x1,8) | x2;
	return ret;
}


/* used only for displaying on the terminal */
byte channel = 0;

/* 
	This is the main controller loop.
	sequences of operations:
		- reads from CAN bus or serial port 1.
		- reads encoders (or ADC).
		- computes the control value (a PID in this version).
		- checks limits and other errors.
		- sets PWM
		- does extra functions (e.g. communicate with neighboring cards).
*/
void main(void)
{
	word temporary;
	UInt16 _counter,l;
	byte data[8];
	UInt16 SSI1_value[4]={0,0,0,0};
	byte ret;
	unsigned char frametype;
	unsigned char frameformat;
	unsigned char lenght;
	bool done = false;
	word i;
	dword IdTx;
	canmsg_t *p;
	canmsg_t canmsg_send;
	UInt16 _duty[4];
	UInt16 _percent[4];
	
	/* gets the address of flash memory from the linker */
//	_flash_addr = get_flash_addr();
	dword messageID=1000;	
	
	/* enable interrupts */
	setReg(SYS_CNTL, 0);
	setRegBits(IPR, 0xfe12);	/* enable all interrupts and IRQA, IRQB */
	__ENIGROUP (52, 4);
	__ENIGROUP (53, 4);
	__ENIGROUP (50, 4);
	__ENIGROUP (51, 4);
	__ENIGROUP (13, 4);
	__ENIGROUP (14, 7);
	__ENIGROUP (15, 7);
	__ENIGROUP (16, 7);
	__ENIGROUP (17, 7);
	__ENIGROUP (42, 4);
	__ENIGROUP (55, 4);
	__ENIGROUP (54, 4);
		
	AS1_init ();
	CAN1_init ();
	PWMC0_init ();
	PWMC1_init ();

	TI1_init ();
	__EI();
	SSI1_Init();	

	/* CAN masks/filters init, note the reverse order of mask comparison (see manual) */
	set_can_masks ();
	
	_percent[0]=50;
	_percent[1]=50;
	_percent[2]=50;
	_percent[3]=50;
	
	
	/* main control loop */
	for(_counter = 0;; _counter ++) 
	{	
		
	/* read commands from CAN or serial board */
	
	//	serial_interface();
	//**************************** SSI code ******************
		setRegBits(GPIO_E_DR,0x10);
		//clrRegBits(GPIO_B_DR,0x01);
		SSI1_value[0]=SSI1_GetVal(0x02);
		SSI1_value[1]=SSI1_GetVal(0x08);
		SSI1_value[2]=SSI1_GetVal(0x20);
		SSI1_value[3]=SSI1_GetVal(0x80);
			
			CAN_EI;
			while(read_p>-1)
			{	
				CAN_DI;
				p = can_fifo + read_p;
				
	
				_canmsg.CAN_data[0] = p->CAN_data[0];
				_canmsg.CAN_data[1] = p->CAN_data[1];
				_canmsg.CAN_data[2] = p->CAN_data[2];
				_canmsg.CAN_data[3] = p->CAN_data[3];
				_canmsg.CAN_data[4] = p->CAN_data[4];
				_canmsg.CAN_data[5] = p->CAN_data[5];
				_canmsg.CAN_data[6] = p->CAN_data[6];
				_canmsg.CAN_data[7] = p->CAN_data[7];
				_canmsg.CAN_messID = p->CAN_messID;
				_canmsg.CAN_frameType = p->CAN_frameType;
				_canmsg.CAN_frameFormat = p->CAN_frameFormat;
				_canmsg.CAN_length = p->CAN_length;
			
				
				if ((_canmsg.CAN_messID & 0x00000700) != 0x0700)
				{
					canmsg_send.CAN_data[0] = SSI1_value[0] & 0xFF;
					canmsg_send.CAN_data[1] = (SSI1_value[0] & 0xFF00)>>8;
					canmsg_send.CAN_data[2] = SSI1_value[1] & 0xFF;
					canmsg_send.CAN_data[3] = (SSI1_value[1] & 0xFF00)>>8;
					canmsg_send.CAN_data[4] = SSI1_value[2] & 0xFF;
					canmsg_send.CAN_data[5] = (SSI1_value[2] & 0xFF00)>>8;
					canmsg_send.CAN_data[6] = SSI1_value[3] & 0xFF;
					canmsg_send.CAN_data[7] = (SSI1_value[3] & 0xFF00)>>8;
					canmsg_send.CAN_messID = 16;
					canmsg_send.CAN_frameType = 0;
					canmsg_send.CAN_frameFormat =0;
					canmsg_send.CAN_length = 8;
					CAN1_sendFrame (1,canmsg_send.CAN_messID,canmsg_send.CAN_frameType,canmsg_send.CAN_length,canmsg_send.CAN_data);	
				}
				if (read_p == write_p)
				{
					read_p = -1;	/* empty */
				}
				else
				{
					read_p ++;
					if (read_p >= CAN_FIFO_LEN)
						read_p = 0;
				}
			_percent[0]=(_canmsg.CAN_data[1]<<8)|_canmsg.CAN_data[0];
			_percent[1]=(_canmsg.CAN_data[3]<<8)|_canmsg.CAN_data[2];
			_percent[2]=(_canmsg.CAN_data[5]<<8)|_canmsg.CAN_data[4];
			_percent[3]=(_canmsg.CAN_data[7]<<8)|_canmsg.CAN_data[6];
			PWMC0_outputPadEnable();
			PWMC1_outputPadEnable();
			/* special message for the can loader */ 
			if ((_canmsg.CAN_messID & 0x00000700) == 0x0700)
			{
				PWMC0_outputPadDisable();
				PWMC1_outputPadDisable();
				if ((_canmsg.CAN_length == 1)) 
				{
	 				IdTx = (_canmsg.CAN_messID & 0x0700);			
					IdTx |= (L_deposit_l (_board_ID) << 4);
					IdTx |= ((_canmsg.CAN_messID >> 4) & 0x000F);
					
			    	switch (_canmsg.CAN_data[0]) 
			    	{
						case 0xFF:
						
							_canmsg.CAN_data[0] = 0xFF;
							_canmsg.CAN_data[1] = 0;  // board type (always 0 for motor control card).
							_canmsg.CAN_data[2] = (_version & 0xff00) >> 8;  // firmware version.	
							_canmsg.CAN_data[3] = _version & 0x00ff; 		 // firmware revision.
							CAN1_sendFrame (1, IdTx, DATA_FRAME, 4, _canmsg.CAN_data);
							break;
							
					    case 4:
		    				if (_board_ID == (_canmsg.CAN_messID & 0x000F)) 
		    				{
								_canmsg.CAN_data[0] = 4;
								_canmsg.CAN_data[1] = 1; 
								CAN1_sendFrame (1, IdTx, DATA_FRAME, 2, _canmsg.CAN_data);
							}	
							break;
							  
					    case 0:
		    				if (_board_ID == (_canmsg.CAN_messID & 0x000F))
		    				    asm(jsr bootStart);	/// check whether this has to be a JMP rather than a JSR.
		    				break;
		    		}	
				}
			}
			}
			

			_duty[0]=(UInt16)(1333*(UInt32)(_percent[0])/100);
			_duty[1]=(UInt16)(1333*(UInt32)(_percent[1])/100);
			_duty[2]=(UInt16)(1333*(UInt32)(_percent[2])/100);
			_duty[3]=(UInt16)(1333*(UInt32)(_percent[3])/100);
			PWMC0_setDuty(0,_duty[0]);
			PWMC0_setDuty(2,_duty[1]);
			PWMC0_setDuty(4,_duty[2]);
			PWMC1_setDuty(0,_duty[3]);
			PWMC0_load();	
			PWMC1_load();	
		while (_wait);
		/* tells that the control cycle is completed */
		_wait = true;	
		
	} /* end for(;;) */
}



/* test/debug serial port interface (on AS1) */
byte serial_interface (void)
{
	Int32 acceptance_code;
	byte d = 0;
	byte send=0;
	byte ret=0;
	byte data[4];
	UInt16 count=0;
	UInt16 AMI=0;
	UInt16 j=0;
	byte ch;
	char duty[2]={0,0};
	UInt16 _duty=0;
	char p;	
	char buffer[SMALL_BUFFER_SIZE];
	int  iretval = 0;
	
	if (c == 0)
		AS1_getString (&p, 1);
		c=p;
	
	switch (c)
	{
		default:
			c = 0;
			break;
		
		case 'h':
		case 'H':
		case '\r':
			AS1_printStringEx ("Test Software BLL\r\n");
			AS1_printStringEx ("h, H: help\r\n");
			AS1_printStringEx ("l, Test LED DL1..DL4\r\n");
			AS1_printStringEx ("b, Test SPI\r\n");
			AS1_printStringEx ("c, Test EEPROM\r\n");
			AS1_printStringEx ("d, Test AMI1\r\n");
			AS1_printStringEx ("e, Test AMI2\r\n");
			AS1_printStringEx ("f, Test PWM channelA\r\n");
			AS1_printStringEx ("g, Test PWM channelB\r\n");
			AS1_printStringEx ("h, Test Quadrature Decoder \r\n");
			AS1_printStringEx ("h, Test analog ANA1 and ANB1 \r\n");
			AS1_printStringEx ("n, Test PWM fault \r\n");
			
				c = 0;
			break;
			
		case 'l':
		case 'L':
			
			AS1_printStringEx ("Test LED started\r\n");	
			/*        LED INIT       */			
			//DL1: GPIOA4
			setRegBits(GPIO_A_DDR,0x10);   
			clrRegBits(GPIO_A_PER,0x10); 
			//DL2: GPIOA5
			setRegBits(GPIO_A_DDR,0x20);   
			clrRegBits(GPIO_A_PER,0x20); 
			//DL3: GPIOA6
			setRegBits(GPIO_A_DDR,0x40);   
			clrRegBits(GPIO_A_PER,0x40); 
			//DL4: GPIOA7
			setRegBits(GPIO_A_DDR,0x80);   
			clrRegBits(GPIO_A_PER,0x80);
			
			/*  SWITCH ON-OFF for 10 times       */
			
			for(count=0;count<10;count++)
			{
				setRegBits(GPIO_A_DR,0x10);
				setRegBits(GPIO_A_DR,0x20);
				setRegBits(GPIO_A_DR,0x40);
				setRegBits(GPIO_A_DR,0x80);
				// Delay
				for(j=0;j<2000;j++)
				{
					asm(NOP);
					asm(NOP);
				} 	
				clrRegBits(GPIO_A_DR,0x10);
				clrRegBits(GPIO_A_DR,0x20);
				clrRegBits(GPIO_A_DR,0x40);
				clrRegBits(GPIO_A_DR,0x80);
				
			} 	
			AS1_printStringEx ("Test LED finished\r\n");
			c=0;
			break;
		
		case 'b':
			AS1_printStringEx ("Test SPI1 started\r\n");
			//SPI Init
			ChipSelectInit();
			SPI1_Init();
			ChipSelectEnable(0);
			send=0b11110000;
			ret=SPI1_SendChar(send);
			AS1_printStringEx ("ERROR CODE:");
			AS1_printByteAsChars (ret);
			c=0;
			ChipSelectDisable();
			AS1_printStringEx ("\r\n");
			AS1_printStringEx ("Test SPI1 finished\r\n");
			
			break;
		case 'c':
			AS1_printStringEx ("Test EEPROM started\r\n");
			EEPROM_Init();
			AS1_printStringEx ("Test 1 Status Register write: \r\n");
			d=0b10000001;
			ret=EEPROM_WRSR(d);
			AS1_printByteAsChars(d);
			AS1_printStringEx ("Error code: \r\n");
			AS1_printByteAsChars(ret);
			AS1_printStringEx ("Test 2 Status Register read: \r\n");
			ret=EEPROM_RDSR(&d);
			AS1_printByteAsChars(d);
			AS1_printStringEx ("Error code: \r\n");
			AS1_printByteAsChars(ret);
			AS1_printStringEx ("Test 3 Write in EEPROM (255 0 255 0): \r\n");
			data[0]=255;
			data[1]=0;
			data[2]=255;
			data[3]=0;
			ret=EEPROM_WRITE(data,0x0000,4);
			AS1_printStringEx ("Error code: \r\n");
			AS1_printByteAsChars(ret);
			AS1_printStringEx ("Test 3 Read in EEPROM (from Address 0 to 4): \r\n");
			ret=EEPROM_READ(data,0x0000,4);
			AS1_printStringEx ("position 0x0000:");
			AS1_printByteAsChars(data[0]);
			AS1_printStringEx ("\r\n");
			AS1_printStringEx ("position 0x0008:");
			AS1_printByteAsChars(data[1]);
			AS1_printStringEx ("\r\n");
			AS1_printStringEx ("position 0x0010:");
			AS1_printByteAsChars(data[2]);
			AS1_printStringEx ("\r\n");
			AS1_printStringEx ("position 0x0018:");
			AS1_printByteAsChars(data[3]);
			AS1_printStringEx ("\r\n");
			AS1_printStringEx ("Error code: \r\n");
			AS1_printByteAsChars(ret);
		
			AS1_printStringEx ("Test EEPROM finished\r\n");
			c=0;
			break;
			
			
		case 'd':
			AS1_printStringEx ("Test AMI1 started\r\n");
			SSI1_Init();
			AMI=SSI1_GetVal(0x01);
			AS1_printStringEx ("AMI1 value: ");
			AS1_printWord16AsChars (AMI);
			AS1_printStringEx ("\r\n");
			AS1_printStringEx ("Test AMI1 finished\r\n");
			c=0;
			break;
			
		case 'e':
			
			AS1_printStringEx ("Test AMI2 started\r\n");
			SSI1_Init();
			AMI=SSI1_GetVal(0x02);
			AS1_printStringEx ("AMI2 value: ");
			AS1_printWord16AsChars (AMI);
			AS1_printStringEx ("\r\n");
			AS1_printStringEx ("Test AMI2 finished\r\n");
			c=0;
			break;
		case 'f':
			AS1_printStringEx ("Test PWM channel A started\r\n");	
		//	PWMC0_init();
			AS1_printStringEx ("Insert PWM duty percent channel 0:\r\n");
			AS1_getString (duty, 2);
			_duty=(UInt16)(1333*(UInt32)(AS1_atoi(duty, 2))/100);
			AS1_printDWordAsCharsDec(_duty);
			PWMC0_setDuty(0,_duty);
			PWMC0_load();
			AS1_printStringEx ("Insert PWM duty percent channel 2:\r\n");
		    AS1_getString (duty, 2);
			_duty=(UInt16)(1333*(UInt32)(AS1_atoi(duty, 2))/100);
			AS1_printDWordAsCharsDec(_duty);
			PWMC0_setDuty(2,_duty);
			PWMC0_load();
			AS1_printStringEx ("Insert PWM duty percent channel 4:\r\n");
			AS1_getString (duty, 2);
			_duty=(UInt16)(1333*(UInt32)(AS1_atoi(duty, 2))/100);
			AS1_printDWordAsCharsDec(_duty);
			PWMC0_setDuty(4,_duty);
			PWMC0_load();
			PWMC0_outputPadEnable();
			ret=1;
			AS1_printStringEx ("Press any key to stop PWM:\r\n");
			while(ret!=0)
			{
				ret=AS1_recvChar((unsigned char *)duty);
			}
			
			PWMC0_setDutyPercent(0,0);
			PWMC0_setDutyPercent(2,0);
			PWMC0_setDutyPercent(4,0);
			PWMC0_outputPadDisable();
			AS1_printStringEx ("Test PWM channel A finished\r\n");
			c=0;
			break;
		case 'g':
			AS1_printStringEx ("Test PWM channel B started\r\n");	
			PWMC1_init();
			PWMC1_outputPadEnable();
			AS1_printStringEx ("Insert PWM duty percent channel 0:\r\n");
            AS1_getString (duty, 2);
			_duty=AS1_atoi (duty, 2);
			AS1_printWord16AsChars(_duty);
			PWMC1_setDutyPercent(0,_duty);
			PWMC1_load();
			AS1_printStringEx ("Insert PWM duty percent channel 2:\r\n");
		    AS1_getString (duty, 2);
			_duty=AS1_atoi (duty, 2);
			AS1_printWord16AsChars(_duty);
			PWMC1_setDutyPercent(2,_duty);
			PWMC1_load();
			AS1_printStringEx ("Insert PWM duty percent channel 4:\r\n");
			AS1_getString (duty, 2);
			_duty=AS1_atoi (duty, 2);
			AS1_printWord16AsChars(_duty);
			PWMC1_setDutyPercent(4,_duty);
			PWMC1_load();
			ret=1;
			AS1_printStringEx ("Press any key to stop PWM:\r\n");
			while(ret!=0)
			{
				ret=AS1_recvChar((unsigned char *)duty);
			}
			PWMC1_outputPadDisable();
			PWMC1_setDutyPercent(0,0);
			PWMC1_setDutyPercent(2,0);
			PWMC1_setDutyPercent(4,0);
			AS1_printStringEx ("Test PWM channel B finished\r\n");
			c=0;
			break;	
		case 'n':
			AS1_printStringEx ("pwm fault 0 = ");
			iretval = getReg (PWMA_PMFSA);
			AS1_printUWord16AsChars (iretval);
			
			AS1_printStringEx (" dismap 0 = ");
			iretval = getReg (PWMA_PMDISMAP1);
			AS1_printUWord16AsChars (iretval);
			
			AS1_printStringEx ("  pwm fault 1 = ");
			iretval = getReg (PWMB_PMFSA);
			AS1_printUWord16AsChars (iretval);

			AS1_printStringEx (" dismap 1 = ");
			iretval = getReg (PWMB_PMDISMAP1);
			AS1_printUWord16AsChars (iretval);
			
			AS1_printStringEx ("\r\n");
			c = 0;
			break;	
		
			
		
	
	}	/* end switch/case */
}

void ChipSelectInit()
{
	//E2PCS: GPIOE2
	setRegBits(GPIO_E_DDR,0x04);   
	clrRegBits(GPIO_E_PER,0x04); 
	//SPIEN1: GPIOA0	
	setRegBits(GPIO_A_DDR,0x01);   
	clrRegBits(GPIO_A_PER,0x01); 
	//SPIEN2: GPIOA1
	setRegBits(GPIO_A_DDR,0x02);   
	clrRegBits(GPIO_A_PER,0x02); 
	
	setRegBits(GPIO_E_DR,0x04);
	setRegBits(GPIO_A_DR,0x01);
	setRegBits(GPIO_A_DR,0x02);
}

// select the chip: b from 0 to 2
void ChipSelectEnable(byte b)
{
	if (b==0)
	{
		//EEPROM
		clrRegBits(GPIO_E_DR,0x04);
		setRegBits(GPIO_A_DR,0x01);
		setRegBits(GPIO_A_DR,0x02);
		return;
	}
	else 
	{
		if (b==1)
		{
			// AMI 1
			setRegBits(GPIO_E_DR,0x04);
	  		clrRegBits(GPIO_A_DR,0x01);
			setRegBits(GPIO_A_DR,0x02);
			return;
		}
	}
	
	// AMI 2
	setRegBits(GPIO_E_DR,0x04);
	setRegBits(GPIO_A_DR,0x01);
	clrRegBits(GPIO_A_DR,0x02);
	return;
}

void ChipSelectDisable()
{
	setRegBits(GPIO_E_DR,0x04);
	setRegBits(GPIO_A_DR,0x01);
	clrRegBits(GPIO_A_DR,0x02);
	return;
}

byte can_receive (void)
{
	bool done = false;
	word i;
	dword IdTx;
	CAN_EI;
	while (!done)
	{
		canmsg_t *p;
		CAN_DI;	
		p = can_fifo + read_p;
		/* makes a private copy of the message */
		_canmsg.CAN_data[0] = p->CAN_data[0];
		_canmsg.CAN_data[1] = p->CAN_data[1];
		_canmsg.CAN_data[2] = p->CAN_data[2];
		_canmsg.CAN_data[3] = p->CAN_data[3];
		_canmsg.CAN_data[4] = p->CAN_data[4];
		_canmsg.CAN_data[5] = p->CAN_data[5];
		_canmsg.CAN_data[6] = p->CAN_data[6];
		_canmsg.CAN_data[7] = p->CAN_data[7];
		_canmsg.CAN_messID = p->CAN_messID;
		_canmsg.CAN_frameType = p->CAN_frameType;
		_canmsg.CAN_frameFormat = p->CAN_frameFormat;
		_canmsg.CAN_length = p->CAN_length;
			
			if (read_p == write_p)
			{
				read_p = -1;	/* empty */
				done = true;
			}
			else
			{
				read_p ++;
				if (read_p >= CAN_FIFO_LEN)
					read_p = 0;
				//done = true;
			}
			CAN_EI;
		}
	
}