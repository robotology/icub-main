/*********************************************************************
 *                Microchip USB C18 Firmware Version 1.0
 *********************************************************************
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Rawin Rojvanit       11/19/04    Original.
 ********************************************************************/


/** I N C L U D E S **********************************************************/
#include <p18f2550.h>
#include <usart.h>
#include "system\typedefs.h"
#include "system\usb\usb.h"
#include "io_cfg.h"             // I/O pin mapping
#include "delays.h"

#include <string.h>
#include <float.h>
#include "..\usart_int.h"
#include "..\timerzero.h"


/** V A R I A B L E S ********************************************************/
#pragma udata


char buffUSBinput[16];
char counUSBinput;
static unsigned char data[10];
char output_buffer[12];


char usb_in_count;
char usb_out_ready;

byte debug;

extern far unsigned char TMR0fim;

unsigned char byte_count;


static unsigned char Ncar;
static unsigned char NChar; //NChar=0, waiting for the firstLetter 
							//NChar=1, waiting for the secondLetter
							//NChar=2, waiting for the thirdLetter			
							//NChar=10,waiting for the Initilize
							//NChar=20,waiting for the Reset
static unsigned char N1Char,N2Char,N3Char; // Store first, second and third letter
static unsigned char MouthH, MouthL, Left_Y, Right_Y, ServoH,ServoL;
			 	// Store the High and Low Hexadecimal values for Mouth, and Servos 
				//and the Low Hexadecimal value for the Left and right Eyes 
static unsigned char char_H,char_L; // Store the High and Low Hexadecimal values
static unsigned char SLetter[16]; //Buffer read from the USB port

static unsigned char Pwm_count; //Used to make the PWM 
static unsigned char Servo_PWM; //Defines the PWM Duty-Cycle
static int time_count;          //Used for delay
 
unsigned char MaxPWM;
unsigned char MinPWM;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
void 	InterruptHandlerHigh (void);

void FirstLetter(void);
void SecondLetter(void);
void ThirdLetter(void);
void Actuate(void);
void WriteUSB (unsigned char);

unsigned char EEPROM_READ(unsigned char);
void EEPROM_WRITE(unsigned char, unsigned char);

/** D E C L A R A T I O N S **************************************************/
#pragma code

/*
UserInit - Variables and registers initializations.
When the Microcontroller starts, it will run first this cycle.
*/
void UserInit(void)
{
	char i;     // Counter

	MaxPWM=EEPROM_READ(0x001); //read from EEPROM memory the Maximum PWM
	MinPWM=EEPROM_READ(0x002); //read from EEPROM memory the Maximum PWM

	Ncar=0;     // Cycle variables reset
	NChar=0;    //
	Pwm_count=0;// PWM count reset

	TRISA=0b00000000;     // Microcontroller Port configuration PORTA
	TRISB=0b00000000;     // Microcontroller Port configuration PORTB
	TRISC	&= 0b10111001;// Microcontroller Port configuration PORTC

	T0CON = 0b11001000;   // Timer 0 configuration

	/* Init Interruptions */
	PIE1	= 0b00000010;
	PIE1bits.SSPIE 	= 1;

	PIE2	= 0b00001000;
	INTCON3 = 0b00000000;
	INTCON2	= 0b10000100;
	INTCON 	= 0b11100000;


	T2CON=0b00000111;     // Timer2/PWM Configuration 
	CCP1CON= 0b00001111;
	PR2=248;              // PWM time configuration
	CCPR1L=0;             // PWM duty-cycle Reset


	/* Configure TIMER 0 */
	T0CON = 0b10000111;   // Timer 0 configuration

	usb_in_count = 0;     // USB configuration
	usb_out_ready = 0;
	TMR0fim = 1;
	memset (buffUSBinput, 0, 15);
	counUSBinput = 0;
	output_buffer[0] = 0;

	/*Servo, Ports, Mouth and Eyes Variables Reset*/

	ServoH='0';
	ServoL='0';
//	Servo_PWM=0;
	Servo_PWM=MaxPWM;
	PORTB=0;
	PORTA=0;
	MouthH='0';
	MouthL='0';
	Left_Y='0';
	Right_Y='0';
	time_count=0;

}//end UserInit

/*Main Cycle ProcessIO*/
/************************************************************
This function will be the main cycle.
Will check for new data in the USB buffer.
If data is detected it will process the information actuating
the microcontroller pins, servo duty-cycle or send requested
information to the USB port.
*************************************************************/

void ProcessIO(void)
{   
	static unsigned char State = 0;
	static unsigned char kt = 0;
	unsigned char j,i;

	output_buffer[0] = 0;

	/* USB Verification connected/disconneced*****************************/
	if((usb_device_state < CONFIGURED_STATE)||(UCONbits.SUSPND==1)) return;
	/* USB data Verification
	       True=new data -> PROCESS;
		   False=no data -> Dont do anything*/
	if(usb_in_count = getsUSBUSART(SLetter, 10))
	{
		if (NChar==0) FirstLetter();  //Test first Letter
		if (NChar==1) SecondLetter(); //Test second Letter
		if (NChar==2) ThirdLetter();  //Test Third Letter
		Ncar=0;	
	}
	if (NChar==3) //If NChar =3 a send or actuate is required
	{	
		Actuate(); // Process USB comand
		if (NChar==4)      // Send command
		{
			data[3]='\n';
			if(mUSBUSARTIsTxTrfReady())
			{
				mUSBUSARTTxRam ((byte *)data, 4);			
				NChar=0;
			}
		}
	}
	if (NChar==20)   //If NChar =4 a reset instruction was send
	{	
		data[0]='A';
		data[1]='C';
		data[2]='K';
		data[3]='\n';
		if(mUSBUSARTIsTxTrfReady())
		{	
			mUSBUSARTTxRam ((byte *)data, 4);			
			NChar=0;
		}
	}
	if (NChar==10) //If NChar =10 a initiation instruction was send
	{
		if (time_count==0)
		{
			time_count=1;
			PORTB=0xFF;
			PORTA=0xFF;
			Servo_PWM=MinPWM;
			
		}	
		if (time_count==1000)
		{
			data[0]='A';
			data[1]='C';
			data[2]='K';
			data[3]='\n';
			PORTB=0x00;
			PORTA=0x00;
			Servo_PWM=MaxPWM;
			ServoH='6';
			ServoL='4';
			MouthH='0';
			MouthL='0';
			Left_Y='0';
			Right_Y='0';

			if(mUSBUSARTIsTxTrfReady())
			{
				mUSBUSARTTxRam ((byte *)data, 4);			
				NChar=0;
			}
		}
	}
}

/***********************************************************
First Letter
Receive first letter:
S, L, R, M, X, I and Z return NChar=0,1,10 or 20
***********************************************************/
void FirstLetter(void) 
{
	unsigned char eeprom_value;
	switch (SLetter[0])
	{
		case 'S':
		{
			N1Char='S';	
			NChar=1;	
			break;
		}
		case 'L':
		{
			N1Char='L';	
			NChar=1;		
			break;
		}
		case 'R':
		{
			N1Char='R';	
			NChar=1;		
			break;
		}
		case 'M':
		{
			N1Char='M';	
			NChar=1;		
			break;
		}
		case 'X':
		{
			NChar=0;	
			break;
		}
		case 'I':
		{
			NChar=10;
			time_count=0;
			break;
		}
		case 'Z':
		{
			NChar=20;
			PORTB=0;
			PORTA=0;
			Servo_PWM=MaxPWM;

			ServoH='6';
			ServoL='4';
			MouthH='0';
			MouthL='0';
			Left_Y='0';
			Right_Y='0';
			break;
		}
				case 'E': //Write the new PWM maximum to the EEPROM
		{
			eeprom_value=(SLetter[1]-'0')*0x0A+(SLetter[2]-'0');		
			MaxPWM=eeprom_value;
			EEPROM_WRITE(0x001, eeprom_value);
	
			NChar=0;	
			break;
		}
		case 'U':  //Write the new PWM minimum to the EEPROM
		{
			eeprom_value=(SLetter[1]-'0')*0x0A+(SLetter[2]-'0');		
			MinPWM=eeprom_value;
			EEPROM_WRITE(0x002, eeprom_value);
			NChar=0;	
			break;
		}
		default:
		{
			NChar=0;
			break;
		}
	}
}

/***********************************************************
Second Letter
Receive first letter:
'0' to '9', 'A' to 'F' or 'X' return NChar=0 or 2
***********************************************************/

void SecondLetter(void)
{

	if (SLetter[1]=='X')
	{
		NChar=0;
	}
	else if (SLetter[1]>='0' && SLetter[1]<='9')
	{
		N2Char=0x10*(SLetter[1]-'0');		
		NChar=2;
		char_H=SLetter[1];
	}
	else if (SLetter[1]>='A' && SLetter[1]<='F')
	{
		N2Char=0x10*(SLetter[1]-'A'+10);		
		NChar=2;
		char_H=SLetter[1];
	}
	else
	{
		NChar=0;
	}
}

/***********************************************************
Third Letter
Receive first letter:
'0' to '9', 'A' to 'F' or 'X' return NChar=0 or 2
***********************************************************/

void ThirdLetter(void)
{	

	if (SLetter[2]=='X')
	{
		NChar=0;
	}
	else if (SLetter[2]>='0' && SLetter[2]<='9')
	{
		N3Char=SLetter[2]-'0';		
		NChar=3;
		char_L=SLetter[2];
	}
	else if (SLetter[2]>='A' && SLetter[2]<='F')
	{
		char_L=SLetter[2];
		N3Char=(SLetter[2]-'A'+10);		
		NChar=3;

	}
	else
	{
		NChar=0;
	}
}
/***********************************************************
Actuate
If NChar = 3
if (N2Char>128) then send data to USB
else actuate (Leds Mouth, eyes or servos)
***********************************************************/

void Actuate(void)
{
	unsigned char auxsum;
	unsigned char PORTAaux, PORTBaux;
	unsigned char bitsaux, bitsaux1, bitsaux2;
	unsigned char auxvar;
	bitsaux=0;
 	bitsaux1=0;
	bitsaux2=0;
	if ((N2Char & 0b10000000)!=0)
	{
	/*Format data to send to USB*/
		switch (N1Char) 
		{
			case 'S':
			{
				data[0]='s';
				data[1]=ServoH;
				data[2]=ServoL;
				NChar=4;	
				
				break;
			}
			case 'L':
			{
				data[0]='l';
				data[1]='0';
				data[2]=Left_Y;

				NChar=4;		
				break;
			}
			case 'R':
			{
				data[0]='r';
				data[1]='0';
				data[2]=Right_Y;
				NChar=4;	
				break;
			}
			case 'M':
			{
				data[0]='m';
				data[1]=MouthH;
				data[2]=MouthL;
				NChar=4;		
				break;
			}
			default:
			{
				NChar=4;	
				break;	
			}
		}
	}
	else
	{
		/*Actuate Leds or servo*/
		switch (N1Char)
		{
			case 'S':
			{
				ServoH=char_H;
				ServoL=char_L;
				auxsum=N2Char+N3Char;			
				NChar=0;	
				auxvar=MaxPWM;
				if (auxsum>auxvar)
				{
 					auxsum=MaxPWM;		
				}
				else if (auxsum<MinPWM)
				{
 					auxsum=MinPWM;		
				}
				Servo_PWM=auxsum;
				// Actuate servo PWM()
				break;
			}
			case 'L':
			{
				Left_Y=char_L;
				bitsaux=0;

				if ((N3Char & 0b00000001)==1)  bitsaux=bitsaux | 0b00001000;
				if ((N3Char & 0b00000010)==2)  bitsaux=bitsaux | 0b00000100;
				if ((N3Char & 0b00000100)==4)  bitsaux=bitsaux | 0b00000010;
				if ((N3Char & 0b00001000)==8)  bitsaux=bitsaux | 0b00000001;
				PORTAaux=LATA & 0b11110000;
				PORTAaux=PORTAaux | bitsaux;
				PORTA=PORTAaux;
				NChar=0;
				break;
			}
			case 'R':
			{
				bitsaux1=0;
				bitsaux2=0;
				Right_Y=char_L;
				if ((N3Char & 0b00000001)==1)  bitsaux2=bitsaux2 | 0b00000010;
				if ((N3Char & 0b00000010)==2)  bitsaux2=bitsaux2 | 0b00000001;
				if ((N3Char & 0b00000100)==4)  bitsaux1=bitsaux1 | 0b00100000;
				if ((N3Char & 0b00001000)==8)  bitsaux1=bitsaux1 | 0b00010000;
				PORTAaux=LATA & 0b00001111;
				PORTAaux=PORTAaux | bitsaux1;
				PORTA=PORTAaux;
				PORTBaux=LATB & 0b11111100;
				PORTBaux=PORTBaux | bitsaux2;
				PORTB=PORTBaux;
				NChar=0;
				break;
			}
			case 'M':
			{
				MouthH=char_H;
				MouthL=char_L;
				auxsum=0x04*(N2Char+N3Char);	
				PORTBaux=LATB & 0b00000011;
				PORTBaux=PORTBaux | auxsum;
				PORTB=PORTBaux;
	
				NChar=0;
				break;
			}
			default:
			{
				
				break;
			}	
		}
		NChar=0;
	}
}



//----------------------------------------------------------------------------
// High priority interrupt vector

#pragma code InterruptVectorHigh = 0x08
void
InterruptVectorHigh (void)
{
  _asm
    goto InterruptHandlerHigh //jump to interrupt routine
  _endasm
}

//----------------------------------------------------------------------------
// High priority interrupt routine

#pragma code
#pragma interrupt InterruptHandlerHigh

void
InterruptHandlerHigh ()
{

	/* Timer 0 */
	if (INTCONbits.TMR0IF)
	{
		/*Timer0 interrupt used for delays*/
		INTCONbits.TMR0IF = 0;
		TMR0fim = 1;
		T0CONbits.TMR0ON = 0;
	}	
	else if (PIR1bits.TMR2IF)
	{
		/*Create Servo PWM*/
		if (time_count!=0) 
		{
			time_count++;
		}
		if (Pwm_count==0)
		{
		 	CCPR1L=0xFF;
			Pwm_count++;
		}
		else if (Pwm_count==1)
		{

			//CCPR1L=(-2.48*Servo_PWM)+248.0;
			CCPR1L=+2.48*Servo_PWM;
			Pwm_count++;
		}
		else 
		{
			CCPR1L=0;
			Pwm_count++;
			if (Pwm_count==20) Pwm_count=0;
		}
			
		PIR1bits.TMR2IF=0;
	}
}


//Read from EEPROM memory

unsigned char EEPROM_READ(unsigned char EEEnd)
{
	EEADR=EEEnd;
	EECON1bits.EEPGD=0;
	EECON1bits.CFGS=0;
	EECON1bits.RD=1;
	return(EEDATA);
}

//Write to the EEPROM memory
void EEPROM_WRITE(unsigned char EEEnd, unsigned char dataEE)
{

	INTCONbits.GIE=0;

	EECON1bits.EEPGD=0;
	EECON1bits.WREN=1;
	EEADR=EEEnd;
	EEDATA=dataEE;	
	EECON2=0x55;
	EECON2=0x0AA;
	EECON1bits.WR=1;
	while( !PIR2bits.EEIF);
	PIR2bits.EEIF=0;

	INTCONbits.GIE=1;
}



/** EOF user.c ***************************************************************/
