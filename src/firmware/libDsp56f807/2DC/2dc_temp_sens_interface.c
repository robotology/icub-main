
#include "temp_sens_interface.h"

volatile UInt16 rise_time_1 =1;
volatile UInt16 fall_time_1 =0;
volatile UInt16 current_time_1 = 0;
volatile UInt16 timer_extension_1 =0;

float           duty_cycle_1=0;
UInt16  temperature=0;


//temp
UInt8 data_register=0; 
 
#define STATUS_IDLE               0x00
#define STATUS_RUNNING            0x01
#define STATUS_OK                 0xF0
#define STATUS_ERR                0xFF
UInt8   temp_sens_status=0;

void delay(Int16 del);

#define TEMP_PIN 0x80
#warning "REMEMBER: sensore di temperatura sul pin GPIO_B7, attualmente non funziona"
#warning "REMEMBER: NOT IMPLEMENTED for 2DC Library"

//********************************************************* 

void SendBit  (bool bit);
bool ReadBit  (void);
void SendString (UInt8* data, UInt8 size);
void SendWord   (UInt16 data);
void SendByte   (UInt8 data);

UInt16 ReceiveWord   (void);
UInt8  ReceiveByte   (void);

//********************************************************* 
void init_temp_sens (void)
{
//@@ DEBUG
/*
 //sensor status
 temp_sens_status = STATUS_IDLE;
 
 // ---------- init port interrupt ----------
 //porto di uscita SSI
 setRegBits(GPIO_B_DDR,TEMP_PIN);   
 clrRegBits(GPIO_B_PER,TEMP_PIN);	 
 setRegBits(GPIO_B_DR, TEMP_PIN); 	
	
 //disable interrupt
 //clrRegBits (GPIO_B_IENR,  0x20);
   
 // ---------- init timer B3 -----------
    // Set the Load register  
 setReg (TMRB3_LOAD, 0);          
  // Set the Counter register
 setReg (TMRB3_CNTR, 0);             
 // Set the Status and control register
 // overflow interrupt active TOFIE bit12 	
 setReg (TMRB3_SCR,  0x800);     
         
 // Set the Control register (timer stopped)  
 setReg (TMRB3_CTRL, 0);     
 
 __EI();
*/ 
}

//*********************************************************
bool ResetTempSens(void)
{
  
 //low output pin
 clrRegBits(GPIO_B_PER,TEMP_PIN);
 setRegBits(GPIO_B_DDR,TEMP_PIN);  	 
 clrRegBits(GPIO_B_DR, TEMP_PIN); 	
		
 // wait 480us minimum
 delay (500);
		
 //now pin is in input mode
 clrRegBits(GPIO_B_DDR,TEMP_PIN);

 // wait 70us
 delay (70);
 
 //acquire presence pulse on input pin       
 data_register=getReg(GPIO_B_DR);
 
 // wait 410us
 delay (430);
        
 if (data_register & TEMP_PIN)
  {
    //LEVEL IS 1=HIGH,RETURN 0=ERROR
 	return 0;
  } 
 else
  {
    //LEVEL IS 0=LOW,RETURN 1=OK
    return 1;
  }
}

//*********************************************************
void MeasureTempSens(void)
{

 UInt16 i=0;

 switch (temp_sens_status)
 {
 	case STATUS_OK:
 	case STATUS_ERR:
 		return;
 		
  	case STATUS_IDLE:
  		 // begin a new conversion
	 	temp_sens_status=STATUS_RUNNING;
 	
  	case STATUS_RUNNING:
 		 // generate reset pulse
		 if(!ResetTempSens())
 		{
 			temp_sens_status = STATUS_ERR;
 			return;
		}
		temp_sens_status++;
	 	return;
	 	
 	case STATUS_RUNNING+1:
 		// skip address identification command
	 	SendByte(0xCC);
	 	temp_sens_status++;
	 	return;
	
 	case STATUS_RUNNING+2:
 	 	// begin temperature conversion command	
 		SendByte(0x44);
	  	temp_sens_status++;
	 	return;
	 	
 	case STATUS_RUNNING+3:
 	    // check if temperature conversion is finished
 	    if (ReadBit())
 	    {
 	    	temp_sens_status++;
 	    	return;
 	    }
 	    
  	case STATUS_RUNNING+4:
 		// generate reset pulse
		 if(!ResetTempSens())
 		{
 			temp_sens_status = STATUS_ERR;
 			return;
		}
		temp_sens_status++;
	 	return;
	 	
	case STATUS_RUNNING+5:
 		// skip address identification command
 		SendByte(0xCC);
	  	temp_sens_status++;
	 	return;
	 	
	case STATUS_RUNNING+6:
 		// read data command	
 		SendByte(0xBE);
	  	temp_sens_status++;
	 	return;
	 	
	case STATUS_RUNNING+7:
		//receives temperature data from the sensor
		temperature=ReceiveWord();

		//data ready for reading
		temp_sens_status=STATUS_OK; 
		return;	
 }
}

//*********************************************************
// 1 < del < 1679 : delay in microseconds
void delay(Int16 del)
{
 UInt16 val = del*39;
 
 //reset timer
 setReg(TMRB3_CNTR,0); 
 
 /* TMRA1_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
 setReg(TMRB3_SCR,0);             
 
 //start timer
 /* TMRA1_CTRL: CM=1,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 */
 setReg (TMRB3_CTRL, 0b0011000000000000);
 
 //wait
 while (getReg(TMRB3_CNTR)<val);
 
 //stop timer	
 /* TMRA1_CTRL: CM=0,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 */
 setReg (TMRB3_CTRL, 0b0001000000000000);         	
}


//*********************************************************
void SendBit  (bool bit)
{
	if (bit)
	{
		//low output pin
		clrRegBits(GPIO_B_PER,TEMP_PIN);
		setRegBits(GPIO_B_DDR,TEMP_PIN);  	 
        clrRegBits(GPIO_B_DR, TEMP_PIN); 	
 
		//wait 15us
		delay (12);
		
		//release output pin
		clrRegBits(GPIO_B_DDR,TEMP_PIN);	 
       	
		//wait 45us
		delay (48);
		
		//wait 1us minimum
	    //recovery time
		delay (1);
	}
	else
	{
		//low output pin
		clrRegBits(GPIO_B_PER,TEMP_PIN);
		setRegBits(GPIO_B_DDR,TEMP_PIN);  	 
        clrRegBits(GPIO_B_DR, TEMP_PIN);
         
		//wait 60us
		delay (60);
		
		//release output pin
		clrRegBits(GPIO_B_DDR,TEMP_PIN);
		
		//wait 1us minimum
		//recovery time
		delay (1);
	}
}

//*********************************************************
bool ReadBit (void)
{

	bool result = 0;
	
	//low output pin
	clrRegBits(GPIO_B_PER,TEMP_PIN);
	setRegBits(GPIO_B_DDR,TEMP_PIN);  	 
    clrRegBits(GPIO_B_DR, TEMP_PIN); 	
 	
	//wait 1us minimum
	delay (2);
	
	//release output pin
	clrRegBits(GPIO_B_DDR,TEMP_PIN);  
	
	//wait 15us minimum, to get safe data
	delay (15);
	
	//read data
	data_register=getReg(GPIO_B_DR);
	 
	//wait 45us
	delay (45);
	
	//wait 1us minimum
	//recovery time
	delay (1);
	
	if (data_register & TEMP_PIN) return 1;
	
	else return 0;
}

//*********************************************************
void SendWord (UInt16 data)
{
	
}

void SendByte (UInt8 data)
{
	UInt8 i = 0;
	for (i=0; i<8; i++)
	{
		if (data & 0x01)
		{
			SendBit(1);
		}
		else
		{
			SendBit(0);
		}
		data >>= 1;
	}
	
	//now pin is in input mode
    clrRegBits(GPIO_B_DDR,TEMP_PIN);
 
}

//*********************************************************
UInt16  ReceiveWord   (void)
{
	UInt8 i = 0;
	UInt16 data = 0;
	bool  data_bit=0;
	for (i=0; i<16; i++)
	{
	  data |= (ReadBit() << i);
	}
	return data;
}
UInt8  ReceiveByte   (void)
{
	UInt8 i = 0;
	UInt8 data = 0;
	bool  data_bit=0;
	for (i=0; i<8; i++)
	{
	  data |= (ReadBit() << i);
	}
	return data;
}

//*********************************************************
Int16 GetTempSens(void)
{
 switch (temp_sens_status)
 {
 	case STATUS_IDLE:
 	return temperature>>1;
 	
 	case STATUS_OK:
 	temp_sens_status=STATUS_IDLE;
 	return temperature>>1;
 	
 	case STATUS_ERR:
 	temp_sens_status=STATUS_IDLE;
 	return -1;
 	break;
 
 	default:	
 	case STATUS_RUNNING: 
  	return temperature>>1;
 }
}


//*********************************************************
#pragma interrupt saveall
void intSENS (void)
{ 
}