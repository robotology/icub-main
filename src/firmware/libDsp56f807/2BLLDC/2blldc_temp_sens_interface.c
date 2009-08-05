
#include "temp_sens_interface.h"
#include "can1.h"

Int16  temperature[2]   	= {0,0};
Int16  temperature_old[2]   = {0,0};

UInt8   data_register[2] = {0,0}; 
 
UInt8   temp_sens_status = 0;
bool    error_flag[2] = {0,0};

void 	delay(UInt16 del);

#define TEMP_PIN1 GPIO_D6
#define TEMP_PIN2 GPIO_D7

//********************************************************* 

void SendBit  	(bool bit);
bool ReadBit  	(bool* bits);
bool SendByte   (UInt8 data);

bool ReceiveTemperatureWord   (void);

bool ResetTempSens_part1(void);
bool ResetTempSens_part2(void);

void output_bit_1   (void);
void output_bit_0   (void);
void output_bit_off (void);

//********************************************************* 
void output_bit_1   (void)
{
 clrRegBits(GPIO_D_PER,TEMP_PIN1);	 
 setRegBits(GPIO_D_DDR,TEMP_PIN1);   
 setRegBits(GPIO_D_DR, TEMP_PIN1); 

 clrRegBits(GPIO_D_PER,TEMP_PIN2);	 
 setRegBits(GPIO_D_DDR,TEMP_PIN2);   
 setRegBits(GPIO_D_DR, TEMP_PIN2); 	
}
//********************************************************* 
void output_bit_0   (void)
{
 clrRegBits(GPIO_D_PER,TEMP_PIN1);
 setRegBits(GPIO_D_DDR,TEMP_PIN1);  	 
 clrRegBits(GPIO_D_DR, TEMP_PIN1); 	
 
 clrRegBits(GPIO_D_PER,TEMP_PIN2);
 setRegBits(GPIO_D_DDR,TEMP_PIN2);  	 
 clrRegBits(GPIO_D_DR, TEMP_PIN2); 	
}
//********************************************************* 
void output_bit_off (void)
{
 clrRegBits(GPIO_D_DDR,TEMP_PIN1);	 
 clrRegBits(GPIO_D_DDR,TEMP_PIN2);	 
}

//********************************************************* 
void init_temp_sens (void)
{

 //sensor status
 temp_sens_status = TEMPERATURE_STATUS_IDLE;
 
 // ---------- init port interrupt ----------
 output_bit_1();
  
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
 
}


//*********************************************************
bool ResetTempSens_part1(void)
{
 //low output pin
 output_bit_0();
 		
 // now wait 480us minimum!
 //delay (500);
 return true;
}

//*********************************************************
bool ResetTempSens_part2(void)
{
 //now pin is in input mode
 output_bit_off ();
 
 // wait 70us
 delay (70);
 
 //acquire presence pulse on input pin       
 data_register[0]=getReg(GPIO_D_DR);
 data_register[1]=getReg(GPIO_D_DR);
 
 error_flag[0]=false;
 error_flag[1]=false;
     
 if (data_register[0] & TEMP_PIN1) 
 	error_flag[0]=true;
 	
 if (data_register[1] & TEMP_PIN2)
 	error_flag[1]=true;
 
 // now wait 410us minimum!
 // delay (430); 
 return true;
}
  		

//*********************************************************
byte MeasureTempSens(void)
{

 UInt16 i=0;
 bool check_bits [2] =  {0,0};
 	
 switch (temp_sens_status)
 {
 	case TEMPERATURE_STATUS_OK:
 	case TEMPERATURE_STATUS_ERR:
 		return temp_sens_status;
 		
  	case TEMPERATURE_STATUS_IDLE:
  		 // begin a new conversion
	 	temp_sens_status=TEMPERATURE_STATUS_RUNNING;
 	
  	case TEMPERATURE_STATUS_RUNNING:
 		 // generate reset pulse
		 if(!ResetTempSens_part1())
 		{
 			temp_sens_status = TEMPERATURE_STATUS_ERR;
 			return temp_sens_status;
		}
		temp_sens_status++;
	 	return temp_sens_status;
	 	
  	case TEMPERATURE_STATUS_RUNNING+1:
 		 // generate reset pulse, second half
		 if(!ResetTempSens_part2())
 		{
 			temp_sens_status = TEMPERATURE_STATUS_ERR;
 			return temp_sens_status;
		}
		temp_sens_status++;
	 	return temp_sens_status;
	 	
 	case TEMPERATURE_STATUS_RUNNING+2:
 		// skip address identification command
	 	if (SendByte(0xCC))	temp_sens_status++;
	 	return temp_sens_status;
	
 	case TEMPERATURE_STATUS_RUNNING+3:
 	 	// begin temperature conversion command	
 		if (SendByte(0x44))	temp_sens_status++;
	 	return temp_sens_status;
	 	
 	case TEMPERATURE_STATUS_RUNNING+4:
 	    // check if temperature conversion is finished
 	    if (ReadBit(check_bits))
 	    {
 	    	temp_sens_status++;
 	    	return temp_sens_status;
 	    }
 	    
  	case TEMPERATURE_STATUS_RUNNING+5:
 		// generate reset pulse
		 if(!ResetTempSens_part1())
 		{
 			temp_sens_status = TEMPERATURE_STATUS_ERR;
 			return temp_sens_status;
		}
		temp_sens_status++;
	 	return temp_sens_status;

	case TEMPERATURE_STATUS_RUNNING+6:
 		// generate reset pulse, second half
		 if(!ResetTempSens_part2())
 		{
 			temp_sens_status = TEMPERATURE_STATUS_ERR;
 			return temp_sens_status;
		}
		temp_sens_status++;
	 	return temp_sens_status;
	 	 	
	case TEMPERATURE_STATUS_RUNNING+7:
 		// skip address identification command
 		if (SendByte(0xCC)) temp_sens_status++;;
	 	return temp_sens_status;
	 	
	case TEMPERATURE_STATUS_RUNNING+8:
 		// read data command	
 		if (SendByte(0xBE)) temp_sens_status++;;
	 	return temp_sens_status;
	 	
	case TEMPERATURE_STATUS_RUNNING+9:
		//receives temperature data from the sensor
		if(!ReceiveTemperatureWord())
			{
			return temp_sens_status;	
			}
		else
			{
			//data ready for reading
			temp_sens_status=TEMPERATURE_STATUS_OK; 
			return temp_sens_status;	
			}
	
 }
 return temp_sens_status;
}

//*********************************************************
// 1 < del < 1679 : delay in microseconds
void delay(UInt16 del)
{
 UInt16 val = 0;
 
 //reset timer
 setReg(TMRB3_CNTR,0); 
 
 /* TMRB3_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
 setReg(TMRB3_SCR,0);             
 
 //start timer
 /* TMRB3_CTRL: CM=1,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 */
 setReg (TMRB3_CTRL, 0x3000);
 
 //wait
 val = del*39;
 while (getReg(TMRB3_CNTR)<val)
 {
 	val = del*39;
 };
 
 //stop timer	
 /* TMRB3_CTRL: CM=0,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 */
 setReg (TMRB3_CTRL, 0x1000); 
  
}


//*********************************************************
void SendBit  (bool bit)
{
		
	
	if (bit)
	{	
		__DI();
		//low output pin
		output_bit_0();
		
		//wait 15us
		// 19/07/07  era 12
		delay (10);
		
		//release output pin
		output_bit_off ();	 
       	
		//wait 45us
		//delay (48);
		
		//wait 1us minimum
	    //recovery time
		//delay (1);
		__EI();
	}
	else
	{
		__DI();
		//low output pin
		output_bit_0();
		 
		//wait 60us
		delay (70);
		
		//release output pin
		output_bit_off ();	 
		
		//wait 1us minimum
		//recovery time
		//delay (1);
		__EI();
	}
	
}

//*********************************************************
bool ReadBit (bool* bits)
{
	bool result = 0;

    __DI();	
	//low output pin
	output_bit_0();		 	
 	
	//wait 1us minimum
	delay (2);
	
	//release output pin
	output_bit_off ();
	
	//wait 15us maxmimum, to get safe data
	// 19/07/07  era 13
	delay (10);
	
	//read data
	data_register[0]=getReg(GPIO_D_DR);
	data_register[1]=getReg(GPIO_D_DR);	
	 
	__EI();
	//wait 45us
	//delay (45);
	
	//wait 1us minimum
	//recovery time
	//delay (1);
	
	bits[0] = (data_register[0] & TEMP_PIN1)==0?0:1;
	bits[1] = (data_register[1] & TEMP_PIN2)==0?0:1;
	
	return (bits[0]);
	
}

//*********************************************************
bool SendByte (UInt8 data)
{
	static UInt8 i = 0;
	
	if (i<8)
	{
		data >>= i;
		if (data & 0x01)
		{
			SendBit(1);
		}
		else
		{
			SendBit(0);
		}
		i++;		
		return false;
	}
	else
	{
		i=0;
		//now pin is in input mode
    	output_bit_off ();	 
    	return true;
	}
}

//*********************************************************
bool  ReceiveTemperatureWord   ()
{
	static UInt8 i = 0;
	static UInt16 data[2] = {0,0};
	
	bool read_bits[2] = {0,0};
	if (i<16)
	{
		if (i==0) 
		{
			data[0] = 0;	
			data[1] = 0;	
		}
		ReadBit(read_bits);
		data[0] |= (read_bits[0] << i);
		data[1] |= (read_bits[1] << i);
		i++;		
		return false;
	}
	else
	{
		i=0;
		
		temperature_old[0]=temperature[0];
		temperature_old[1]=temperature[1];
		
		temperature[0] = data[0];
		temperature[1] = data[1];
		
		if (temperature[0]-temperature_old[0] > 20 ||
			temperature[0]-temperature_old[0] < -20)
			can_printf("********SPIKE TEMP!!! ax%d: %d******",0,temperature[0]);

		if (temperature[1]-temperature_old[1] > 20 ||
			temperature[1]-temperature_old[1] < -20)
			can_printf("********SPIKE TEMP!!! ax%d: %d******",1,temperature[1]);
							
    	return true;
	}
}


//*********************************************************
Int16 GetTempSens(byte sens_num)
{
 // the shift >> 1 is used to get 1 degree C of accuracy instead
 // of the 0.5 provided by the sensor
 if (sens_num>1) return 0;
 
 switch (temp_sens_status)
 {
 	case TEMPERATURE_STATUS_IDLE:
 	if (error_flag[sens_num]==true)
 		 return 0;
 	else
 		 return (temperature[sens_num]&0xFF)>>1; 
 	
 	case TEMPERATURE_STATUS_OK:
 	temp_sens_status=TEMPERATURE_STATUS_IDLE;
 	if (error_flag[sens_num]==true) 
 		return 0;
 	else 
 		return (temperature[sens_num]&0xFF)>>1;
 	
 	case TEMPERATURE_STATUS_ERR:
 	temp_sens_status=TEMPERATURE_STATUS_IDLE;
 	return 0;
 	break;
 
 	default:	
 	case TEMPERATURE_STATUS_RUNNING: 
 	if (error_flag[sens_num]==true) 
 		return 0;
  	else 
  		return (temperature[sens_num]&0xFF)>>1;
 }
}


