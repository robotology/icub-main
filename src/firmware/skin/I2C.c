#include<p30fxxxx.h>
#include "I2C.h"
//---------------------------------
//Function prototypes
//---------------------------------

//External functions
//==================
extern void Wait(unsigned int value);
extern const unsigned char AD7147_ADD[4];

static unsigned char ReceivedByte[4]={0,0,0,0};
//---------------------------------
//Function definitions
//---------------------------------


/********************************************************************************************************/
/****************** High level functions, need to be customised according to the device *****************/
/********************************************************************************************************/

void I2C_Init(unsigned char Channel)
{
	OC1CON= 0; 
	OC2CON= 0; 
	IC1CON= 0; 
	IC2CON= 0; 
	

	switch (Channel)
	{
		case CH0:
    	{
			MCE_0output
    		DE_0output
		}
		break;
		case CH1:
		{
			MCE_1output
    		DE_1output	
		}
  		break;
	}
}


void I2C_test(unsigned char Channel)
{
	switch (Channel)
	{
		case CH0:
 	    {
			DE_0output   
		    MCO_0=1;
 		  Wait(I2Cbit);// Wait(I2Cbit);
  			MCO_0=0;
   			DO_0on;
    		Wait(I2Cbit);//Wait(I2Cbit);
    		MCO_0=1;
    		DO_0on;
    		Wait(I2Cbit);//Wait(I2Cbit);
    		MCO_0=0;
    		DO_0off;
    		Wait(I2Cbit);//Wait(I2Cbit);
		}
		break;
		case CH1:
		{
			DE_1output   
		    MCO_1=1;
 		    Wait(I2Cbit);//Wait(I2Cbit);
  			MCO_1=0;
   			DO_1on;
    		Wait(I2Cbit);//Wait(I2Cbit);
    		MCO_1=1;
    		DO_1on;
    		Wait(I2Cbit);//Wait(I2Cbit);
    		MCO_1=0;
    		DO_1off;
    		Wait(I2Cbit);//Wait(I2Cbit);	
		}
		break;
 	}   
}   


//---------------------------------
//WriteAD7147ViaI2C()
//---------------------------------
//Function that writes to the AD7147 via the I2C port. It sends first the device
//address including the write bit, then the register address and finally the
//data. The function returns "1" if successfull otherwise "0".
//--------------------------------------------------------------------------------
unsigned char WriteToAD7147ViaI2C(unsigned char Channel,unsigned char DeviceAddress, const unsigned int RegisterStartAddress, const unsigned char NumberOfRegistersToWrite, unsigned int *DataBuffer, const unsigned int OffsetInBuffer)
{
    unsigned int  DataToWrite;
    unsigned char LowByteAddress, HighByteAddress;
    unsigned char LowByteData, HighByteData;
    unsigned char r, AcknError;
    unsigned char DeviceAddressHeader;

    AcknError=1; //No error on initialisation

    //Add the write bit to the device address
    DeviceAddressHeader = DeviceAddress<<1 | I2C_WR;
    //Split the address in two bytes 
    HighByteAddress = (RegisterStartAddress & 0xFF00) >>8;
    LowByteAddress = RegisterStartAddress & 0x00FF;

    //Start the I2C transfer
    InitialiseI2CMaster(Channel);
    StartI2CMaster(Channel);
    //Send device address
    if (!SendByteI2CMaster(Channel,DeviceAddressHeader))
    {
        //Send register address if the acknowledgement is there
        if (!SendByteI2CMaster(Channel,HighByteAddress))
        {
            if (!SendByteI2CMaster(Channel,LowByteAddress))
            {
                //Perform block write
                for (r=0;r<NumberOfRegistersToWrite;r++)
                {
                    DataToWrite = DataBuffer[OffsetInBuffer+r];
                    LowByteData = DataToWrite & 0x00FF;
                    HighByteData = (DataToWrite & 0xFF00)>>8;
                    if (!SendByteI2CMaster(Channel,HighByteData))
                    {
                        SendByteI2CMaster(Channel,LowByteData);
                    } else //No acknowledgement was found therefore send the stop condition
                    {
                        StopI2CMaster(Channel);
                        AcknError=0;
                    }
                }
                //Stop transfer
                StopI2CMaster(Channel);
            } else //No acknowledgement was found therefore send the stop condition
            {
                StopI2CMaster(Channel);
                AcknError=0;
            }
        } else //No acknowledgement was found therefore send the stop condition
        {
            StopI2CMaster(Channel);
            AcknError=0;
        }
    } else //No acknowledgement was found therefore send the stop condition
    {
        StopI2CMaster(Channel);
        AcknError=0;
    }
    return(AcknError);
}


//---------------------------------
//ReadFromAD7147ViaI2C()
//---------------------------------
//Function that reads from the AD7147 via the I2C port. It sends first the device
//address including the write bit, then the register address and finally reads data
//back. The function returns "1" if successfull otherwise "0". If an error occurs,
//Then the stop condition is sent.
//--------------------------------------------------------------------------------
unsigned char ReadFromAD7147ViaI2C(unsigned char Channel, unsigned char DeviceAddress, const unsigned int RegisterStartAddress, const unsigned char NumberOfRegistersToRead, unsigned int *DataBuffer1,unsigned int *DataBuffer2,unsigned int *DataBuffer3,unsigned int *DataBuffer4, const unsigned int OffsetInBuffer)
{
    unsigned int ReadData;
    unsigned char LowByteAddress, HighByteAddress;
    unsigned char LowByteData[4], HighByteData[4];
    unsigned char r, AcknError;
    unsigned char DeviceAddressHeader;

    AcknError=1; //No error on initialisation

    //Add the write bit to the device address
    DeviceAddressHeader=DeviceAddress<<1 | I2C_WR; //qui ci vuole I2C_WR o RD secondo me RD
    //Split the address in two bytes 
    HighByteAddress = (RegisterStartAddress & 0xFF00) >>8;
    LowByteAddress = RegisterStartAddress & 0x00FF;

    //Start the I2C transfer
    InitialiseI2CMaster(Channel);
    StartI2CMaster(Channel);

    //Send device address
    if (!SendByteI2CMaster(Channel,DeviceAddressHeader))
    {
        //Send register address
        if (!SendByteI2CMaster(Channel,HighByteAddress))
        {
            if (!SendByteI2CMaster(Channel,LowByteAddress))
            {
                //Send the repeated start
                StartI2CMaster(Channel);
                //Send device address again changing the Rd/Wr bit
                DeviceAddressHeader = DeviceAddress<<1 | I2C_RD;
                if (!SendByteI2CMaster(Channel,DeviceAddressHeader))
                {
                    //Perform block read, but first,we need to know if we must send an ACKN or a NACK
                    if (NumberOfRegistersToRead==1)
                    {
                        ReceiveByteI2CMaster(Channel,ACK);
                        HighByteData[0]=ReceivedByte[0];
                        HighByteData[1]=ReceivedByte[1];
                        HighByteData[2]=ReceivedByte[2];
                        HighByteData[3]=ReceivedByte[3];

                        ReceiveByteI2CMaster(Channel,NACK);
                        LowByteData[0]=ReceivedByte[0];
                        LowByteData[1]=ReceivedByte[1];
                        LowByteData[2]=ReceivedByte[2];
                        LowByteData[3]=ReceivedByte[3];

                        ReadData=((HighByteData[0] & 0xFF)<<8) | LowByteData[0];
                        DataBuffer1[OffsetInBuffer]=ReadData;
                        ReadData=((HighByteData[1] & 0xFF)<<8) | LowByteData[1];
                        DataBuffer2[OffsetInBuffer]=ReadData;
                        ReadData=((HighByteData[2] & 0xFF)<<8) | LowByteData[2];
                        DataBuffer3[OffsetInBuffer]=ReadData;
                        ReadData=((HighByteData[3] & 0xFF)<<8) | LowByteData[3];
                        DataBuffer4[OffsetInBuffer]=ReadData;

                    } else
                    {
                        for (r=0;r<(NumberOfRegistersToRead-1);r++)
                        {
                            ReceiveByteI2CMaster(Channel,ACK);
                            HighByteData[0]=ReceivedByte[0];
                            HighByteData[1]=ReceivedByte[1];
                            HighByteData[2]=ReceivedByte[2];
                            HighByteData[3]=ReceivedByte[3];

                            ReceiveByteI2CMaster(Channel,ACK);
                            LowByteData[0]=ReceivedByte[0];
                            LowByteData[1]=ReceivedByte[1];
                            LowByteData[2]=ReceivedByte[2];
                            LowByteData[3]=ReceivedByte[3];

                            ReadData=((HighByteData[0] & 0xFF)<<8) | LowByteData[0];
                            DataBuffer1[OffsetInBuffer+r]=ReadData;
                            ReadData=((HighByteData[1] & 0xFF)<<8) | LowByteData[1];
                            DataBuffer2[OffsetInBuffer+r]=ReadData;
                            ReadData=((HighByteData[2] & 0xFF)<<8) | LowByteData[2];
                            DataBuffer3[OffsetInBuffer+r]=ReadData;
                            ReadData=((HighByteData[3] & 0xFF)<<8) | LowByteData[3];
                            DataBuffer4[OffsetInBuffer+r]=ReadData;
                        }
                        //Do the last read sending the NACK
                        ReceiveByteI2CMaster(Channel,ACK);
                        HighByteData[0]=ReceivedByte[0];
                        HighByteData[1]=ReceivedByte[1];
                        HighByteData[2]=ReceivedByte[2];
                        HighByteData[3]=ReceivedByte[3];

                        ReceiveByteI2CMaster(Channel,NACK);
                        LowByteData[0]=ReceivedByte[0];
                        LowByteData[1]=ReceivedByte[1];
                        LowByteData[2]=ReceivedByte[2];
                        LowByteData[3]=ReceivedByte[3];

                        ReadData=((HighByteData[0] & 0xFF)<<8) | LowByteData[0];
                        DataBuffer1[OffsetInBuffer+NumberOfRegistersToRead-1]=ReadData;
                        ReadData=((HighByteData[1] & 0xFF)<<8) | LowByteData[1];
                        DataBuffer2[OffsetInBuffer+NumberOfRegistersToRead-1]=ReadData;
                        ReadData=((HighByteData[2] & 0xFF)<<8) | LowByteData[2];
                        DataBuffer3[OffsetInBuffer+NumberOfRegistersToRead-1]=ReadData;
                        ReadData=((HighByteData[3] & 0xFF)<<8) | LowByteData[3];
                        DataBuffer4[OffsetInBuffer+NumberOfRegistersToRead-1]=ReadData;

                    }
                    //Stop transfer
                    StopI2CMaster(Channel);
                } else //No acknowledgement was found therefore send the stop condition
                {
                    StopI2CMaster(Channel);
                    AcknError=0;
                }
            } else //No acknowledgement was found therefore send the stop condition
            {
                StopI2CMaster(Channel);
                AcknError=0;
            }
        } else //No acknowledgement was found therefore send the stop condition
        {
            StopI2CMaster(Channel);
            AcknError=0;
        }
    } else //No acknowledgement was found therefore send the stop condition
    {
        StopI2CMaster(Channel);
        AcknError=0;
    }
    return(AcknError);
}



/********************************************************************************************************/
/*** Low level functions, do not change anything below this line, however check the valid clock level ***/
/********************************************************************************************************/

//---------------------------------
//InitialiseI2CMaster();
//---------------------------------
//Function that configures the I2C port of the ADuC841 in master mode.
//--------------------------------------------------------------------------------
void InitialiseI2CMaster(unsigned char Channel)
{
	switch (Channel)
	{
	case CH0:	
    {
		MCE_0output
	}
	break;
	case CH1:
	{
		MCE_1output;
	}
	break;
	}
/*modificated
    TRISFbits.TRISF6=0; //Write a "0" to CLK so that it becomes an output.
*/
    //I2CCON = 0xA8;		//Master mode
}


//---------------------------------
//StartI2CMaster();
//---------------------------------
//Function that implements the start condition of the I2C protocol. The start
//condition consists in a falling edge on SDA when SCL is high.
//--------------------------------------------------------------------------------
void StartI2CMaster(unsigned char Channel)
{
	switch (Channel)
	{
	case CH0:
	{	
		DE_0output    //SDA as output	
		DO_0off        //SDA high
	    MCO_0 = 0;    //SCL low
    	Wait(I2Cbit);
	    DO_0off        //SDA high
	    Wait(I2Cbit);
	    MCO_0 = 1;    //SCL high
	    Wait(I2Cbit);
	    DO_0on       //SDA goes low before the clock
	    Wait(I2Cbit);
	    Wait(I2Cbit);
	    DO_0off
	    Wait(I2Cbit);
	    MCO_0 = 0;    //SCL low
	    Wait(I2Cbit);	
	}    
    /*{
		DE_0output    //SDA as output	
	    MCO_0 = 0;    //SCL low
		Nop();
		Nop();
		Nop();
		Nop();
	    DO_0on        //SDA high
	    MCO_0 = 1;    //SCL high
	    Wait(I2Cbit);
	    DO_0off       //SDA goes low before the clock
	    Wait(I2Cbit);
	    MCO_0 = 0;    //SCL low
	    Wait(I2Cbit);
	}
	*/
	break;
	case CH1:
	{
		DE_1output    //SDA as output	
	    MCO_1 = 0;    //SCL low
	    DO_1on        //SDA high
	    MCO_1 = 1;    //SCL high
	    Wait(I2Cbit);
	    DO_1off       //SDA goes low before the clock
	    Wait(I2Cbit);
	    MCO_1 = 0;    //SCL low
	    Wait(I2Cbit);
	}
	break;
	}
  
}


//---------------------------------
//StopI2CMaster();
//---------------------------------
//Function that implements the stop condition of the I2C protocol. The stop
//condition consists in a rising edge on SDA when SCL is high.
//--------------------------------------------------------------------------------
void StopI2CMaster(unsigned char Channel)
{
	switch (Channel)
	{
	case CH0:	
    {
	    DE_0output  //SDA as output	
	    DO_0off  //SDA low
	    Wait(I2Cbit);Wait(I2Cbit);
	    MCO_0 = 1;    //SCL high
	    Wait(I2Cbit);//Wait(I2Cbit);
	    DO_0on   //SDA goes from low to high when SCL is already high,
	    Wait(I2Cbit);//Wait(I2Cbit);
	}
	break;
	case CH1:
	{   
	    DE_1output  //SDA as output	
	    DO_1off  //SDA low
	    Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_1 = 1;    //SCL high
	    Wait(I2Cbit);//Wait(I2Cbit);
	    DO_1on   //SDA goes from low to high when SCL is already high,
	    Wait(I2Cbit);//Wait(I2Cbit);
	}	
	break;
	}
}


//---------------------------------
//SendByteI2CMaster();
//---------------------------------
//Function that sends a byte to the I2C port and then read the acknowledgement 
//bit. If the acknowledgement is found, then the function returns "1" otherwise, 
//it returns "0".
//--------------------------------------------------------------------------------
unsigned char SendByteI2CMaster(unsigned char Channel,unsigned char ByteToSend)
{

    unsigned char i;
    unsigned char noack;
	
	switch (Channel)
	{
	case CH0:
	{
	    DE_0output  //SDAs as output
	    for (i=8; i>0; i--)
	    {
	        MCO_0 = 0;                //Reset SCL		
	        /*@@@@@@@@@@@@
	            DO = ByteToSend >> 7;	//Send data to SDA pin
	        */
	        if (ByteToSend >> 7)
	        {
	            DO_0on;
	        } 
			else
	        {
	            DO_0off;
	        }
			Wait(0);
		
	     //   Wait(I2Cbit);//Wait(I2Cbit);
	        MCO_0 = 1;                //Set SCL
	         Wait(I2Cbit);//Wait(I2Cbit);
	        MCO_0 = 0;                //Reset SCL
	   //      Wait(I2Cbit);//Wait(I2Cbit);
	        ByteToSend<<=1;         //Rotate data
	    }
	    DO_0off
	    DE_0input                //SDA becomes an input
	    MCO_0 = 0;                //Reset SCL
	
	    Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_0 = 1;                //Set SCL

	//	if  ((DO1+DO2+DO3+DO4)<4) noack=0;
	//	else noack=1;

#warning DEBUG 
		//noack=0;  
	    //noack =(unsigned char) ((DO1 | DO2<<1 | DO3<<2 | DO4<<3) & 0xFF)?1:0);//(unsigned char) (PORTDbits.RD0 );//& PORTDbits.RD1 & PORTDbits.RD2 & PORTDbits.RD3);//PORTDbits.RD0; // 
	    DO_0off;
	
	     Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_0 = 0;
	    //DE = 1;	//Enable SDA output
	    DE_0output   //SDA becomes an output
	}
	break;
	case CH1:
	{
	    DE_1output  //SDAs as output
	    for (i=8; i>0; i--)
	    {
	        MCO_1 = 0;                //Reset SCL		
	        if (ByteToSend >> 7)
	        {
	            DO_1on;
	        } 
			else
	        {
	            DO_1off;
	        }	
	         Wait(I2Cbit);//Wait(I2Cbit);
	        MCO_1 = 1;                //Set SCL
	         Wait(I2Cbit);//Wait(I2Cbit);
	        MCO_1 = 0;                //Reset SCL
	         Wait(I2Cbit);//Wait(I2Cbit);
	        ByteToSend<<=1;         //Rotate data
	    }
	    DO_1off
	    DE_1input                //SDA becomes an input
	    MCO_1 = 0;                //Reset SCL
	
	     Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_1 = 1;                //Set SCL
#warning "no chech for the ack"

//		if  ((DO5+DO6+DO7+DO8)<4) noack=0;
//		else noack=1;
		noack=1;
		
		
	    //noack =(unsigned char) ((DO5+DO6+DO7+DO8)<4)?0:1);//(PORTDbits.RD0 );//& PORTDbits.RD1 & PORTDbits.RD2 & PORTDbits.RD3);//PORTDbits.RD0; // 
	    DO_1off;
	
	     Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_1 = 0;
	    DE_1output   //SDA becomes an output
	}
	break;
	}
    return(noack);
}



//---------------------------------
//ReceiveByteI2CMaster();
//---------------------------------
//Function that reads one byte from the I2C port. If we do continuous read, 
//then the acknowledgement must be "0" excepted for the last read sequence which
//it must be "1".
//--------------------------------------------------------------------------------
void ReceiveByteI2CMaster(unsigned char Channel,unsigned char ackn)     // changed bit with unsigned char
{	
    unsigned char i;
	unsigned char D1,D2,D3,D4;
	switch (Channel)
	{
	case CH0:
	{
		DE_0input     //SDA becomes an input
	    MCO_0 = 0;    //Reset SCL
	    for (i=8; i>0; i--)
	    {
			
	        Wait(I2Cbit);//Wait(I2Cbit);
	  
			ReceivedByte[0] <<= 1;      //Rotate data
	        ReceivedByte[1] <<= 1;      //Rotate data
	        ReceivedByte[2] <<= 1;      //Rotate data
	        ReceivedByte[3] <<= 1;      //Rotate data
	        MCO_0 = 1;                //Set SCL
	        
			D1=PORTDbits.RD0;
			D2=PORTDbits.RD2;
			D3=PORTDbits.RD3;
			D4=PORTDbits.RD1;
		

	
	        //test	
	        //	ReceivedByte[0]=i;			
	        ReceivedByte[0] |= D1;   //Read SDA -> data 
 	        ReceivedByte[1] |= D2;   //Read SDA -> data   
	        ReceivedByte[2] |= D3;   //Read SDA -> data   
	        ReceivedByte[3] |= D4;   //Read SDA -> data   
	      //   Wait(I2Cbit);//Wait(I2Cbit);
	        MCO_0 = 0;                //Reset SCL
	    //     Wait(I2Cbit);//Wait(I2Cbit);
	    }

	    DE_0output               //SDA becomes an output
	    if (ackn==1)
	    {
	        DO_0on
	    } 
		else DO_0off

	     Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_0 = 1;    //Set SCL
	     Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_0 = 0;    //Reset SCL
	     Wait(I2Cbit);//Wait(I2Cbit);
	}
	break;
	case CH1:
	{
		DE_1input     //SDA becomes an input
	    MCO_1 = 0;    //Reset SCL
	    for (i=8; i>0; i--)
	    {
	         Wait(I2Cbit);//Wait(I2Cbit);
	        MCO_1 = 1;                //Set SCL
	        ReceivedByte[0] <<= 1;      //Rotate data
	        ReceivedByte[1] <<= 1;      //Rotate data
	        ReceivedByte[2] <<= 1;      //Rotate data
	        ReceivedByte[3] <<= 1;      //Rotate data
	
	        //test	
	        //	ReceivedByte[0]=i;			
	        ReceivedByte[0] |= DO5;   //Read SDA -> data    MDI
	        ReceivedByte[1] |= DO6;   //Read SDA -> data    MDI
	        ReceivedByte[2] |= DO7;   //Read SDA -> data    MDI
	        ReceivedByte[3] |= DO8;   //Read SDA -> data    MDI
	         Wait(I2Cbit);//Wait(I2Cbit);
	        MCO_1 = 0;                //Reset SCL
	         Wait(I2Cbit);//Wait(I2Cbit);
	    }

	    DE_1output               //SDA becomes an output
	    if (ackn==1)
	    {
	        DO_1on
	    } 
		else DO_1off

	     Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_1 = 1;    //Set SCL
	     Wait(I2Cbit);//Wait(I2Cbit);
	    MCO_1 = 0;    //Reset SCL
	     Wait(I2Cbit);//Wait(I2Cbit);
	}
	break;
	}
    
}

