#include "AD7147RegMap.h"
#include "I2C.h"



//----------------------
//Function declarations
//----------------------     
//External functions
//==================
//unsigned char WriteToAD7147ViaI2C(unsigned char Channel, unsigned char DeviceAddress, const unsigned int RegisterStartAddress, const unsigned char NumberOfRegistersToWrite, unsigned int *DataBuffer, const unsigned int OffsetInBuffer);
//unsigned char ReadFromAD7147ViaI2C(unsigned char Channel, unsigned char DeviceAddress, const unsigned int RegisterStartAddress, const unsigned char NumberOfRegistersToRead, unsigned int *DataBuffer, const unsigned int OffsetInBuffer);
//Local functions
//===============
void ConfigAD7147(unsigned char Channel,    unsigned int i, unsigned int pw_control_val, unsigned int * convalue); //i is the number of the triangle
void ConfigAD7147_THREE(unsigned char Channel,unsigned int i,unsigned int pw_control_val, unsigned int *convalue);
void ConfigAD7147_ALL(unsigned char Channel,unsigned int i, unsigned int pw_control_val, unsigned int * convalue); //i is the number of the triangle


//Recal of variables from other C files
//-------------------------------------
extern const unsigned char AD7147_ADD[32];

//---------------------
//Function definitions
//---------------------

void ConfigAD7147(unsigned char Channel, unsigned int i,unsigned int pw_control_val, unsigned int * convalue)
{
	unsigned int ConfigBuffer[12];
	//=============================================
	//= Stage 0 - CIN0 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFFE;//0xAAAA;//0xFFFF;
	ConfigBuffer[1]=0x3FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE0_CONNECTION, 8, ConfigBuffer, 0);	//Stage 0 connection registers - Start address in RAM 0x80
	
	//=============================================
	//= Stage 1 - CIN1 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFFB;//0xAAAA;//0xFFFF;
	ConfigBuffer[1]=0x3FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE1_CONNECTION, 8, ConfigBuffer, 0);	//Stage 1 connection registers - Start address in RAM 0x88
	
	//=============================================
	//= Stage 2 - CIN2 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFEF;//0xAAAA;//0xFFFF;
	ConfigBuffer[1]=0x3FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE2_CONNECTION, 8, ConfigBuffer, 0);	//Stage 2 connection registers - Start address in RAM 0x90
	
	//=============================================
	//= Stage 3 - CIN3 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFBF;//0xAAAA;//0xFFFF;
	ConfigBuffer[1]=0x3FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE3_CONNECTION, 8, ConfigBuffer, 0);	//Stage 3 connection registers - Start address in RAM 0x98
	
	//=============================================
	//= Stage 4 - CIN4 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFEFF;//0xAAAA;//0xFFFF;
	ConfigBuffer[1]=0x3FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE4_CONNECTION, 8, ConfigBuffer, 0);	//Stage 4 connection registers - Start address in RAM 0xA0
	
	//=============================================
	//= Stage 5 - CIN5 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFBFF;//0xAAAA;//0xFFFF;
	ConfigBuffer[1]=0x3FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE5_CONNECTION, 8, ConfigBuffer, 0);	//Stage 5 connection registers - Start address in RAM 0xA8
	
	//=============================================
	//= Stage 6 - CIN6 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xEFFF;//0xFFFF;
	ConfigBuffer[1]=0x3FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE6_CONNECTION, 8, ConfigBuffer, 0);	//Stage 6 connection registers - Start address in RAM 0xB0
	
	//=============================================
	//= Stage 7 - CIN7 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFFF;//0xFFFF;
	ConfigBuffer[1]=0x3FFE;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE7_CONNECTION, 8, ConfigBuffer, 0);	//Stage 7 connection registers - Start address in RAM 0xB8
	
	//=============================================
	//= Stage 8 - CIN8 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFFF;//0xFFFF;
	ConfigBuffer[1]=0x3FFB;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE8_CONNECTION, 8, ConfigBuffer, 0);	//Stage 8 connection registers - Start address in RAM 0xC0
	
	//=============================================
	//= Stage 9 - CIN9 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFFF;//0xFFFF;
	ConfigBuffer[1]=0x3FEF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE9_CONNECTION, 8, ConfigBuffer, 0);	//Stage 9 connection registers - Start address in RAM 0xC8
	
	//=============================================
	//= Stage 10 - CIN10 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFFF;//0xFFFF;
	ConfigBuffer[1]=0x3FBF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE10_CONNECTION, 8, ConfigBuffer, 0);//Stage 10 connection registers - Start address in RAM 0xD0
	
	//=============================================
	//= Stage 11 - CIN11 Single-Ended(+) =
	//=============================================
	ConfigBuffer[0]=0xFFFF;//0xFFFF;
	ConfigBuffer[1]=0x3EFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE11_CONNECTION, 8, ConfigBuffer, 0);//Stage 11 connection registers - Start address in RAM 0xD8
	
	//=============================================
	//= Configure 1st register bank
	//=============================================
	//Initialisation of the first register bank but not the AMBCOMPCTL_REG0
	ConfigBuffer[PWR_CONTROL]=pw_control_val;	// Full power mode enabled at 32ms - 12 sequences - 256 decimation factor
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],PWR_CONTROL, 1, ConfigBuffer, PWR_CONTROL);
	// Run data path for all sequences
	ConfigBuffer[STAGE_CAL_EN]=0x0;//0x0FFF;
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE_CAL_EN, 1, ConfigBuffer, STAGE_CAL_EN);

	//Calibration configuration
	ConfigBuffer[AMB_COMP_CTRL0]=0x0;;//0x220;//0x220;
	ConfigBuffer[AMB_COMP_CTRL1]=0x14C8;//0x14C8;
	ConfigBuffer[AMB_COMP_CTRL2]=0x0832;//0xBFF;//0x0832;
	//Interrupt configuration
	ConfigBuffer[STAGE_LOW_INT_EN]=0x0000;
	ConfigBuffer[STAGE_HIGH_INT_EN]=0x0000;
	ConfigBuffer[STAGE_COMPLETE_INT_EN]=0x800;//0x800;//0x0001;//0x0001;
	ConfigBuffer[STAGE_LOW_LIMIT_INT]=0x0000;
	ConfigBuffer[STAGE_HIGH_LIMIT_INT]=0x0000;
	ConfigBuffer[STAGE_COMPLETE_LIMIT_INT]=0x000;//0x0FFF;
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],AMB_COMP_CTRL0, 9, ConfigBuffer, AMB_COMP_CTRL0);
		WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],PWR_CONTROL, 11, ConfigBuffer, PWR_CONTROL);
	// Run data path for all sequences
//	ConfigBuffer[STAGE_CAL_EN]=0x0;//0x0FFF;
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE_CAL_EN, 1, ConfigBuffer, STAGE_CAL_EN);
}

void ConfigAD7147_ALL(unsigned char Channel,unsigned int i,unsigned int pw_control_val, unsigned int *convalue)
{
	unsigned int ConfigBuffer[12];
	
	//=============================================
	//= Stage 0 - Connected to Vbias
	//=============================================
	ConfigBuffer[0]=0xEAAA;//0xFFFF;//0xFFFF; 9_10_11_12 (CIN8,CIN9,CIN10,CIN11)
	ConfigBuffer[1]=0x1EAA;//0x1EAB;//0x3EAB;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE0_CONNECTION, 8, ConfigBuffer, 0);	//Stage 9 connection registers - Start address in RAM 0xC8
	
	//=============================================
	//= Configure 1st register bank
	//=============================================
	//Initialisation of the first register bank but not the AMBCOMPCTL_REG0
	ConfigBuffer[PWR_CONTROL]=pw_control_val;	// Full power mode enabled at 32ms - 4 sequences - 256 decimation factor
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],PWR_CONTROL, 1, ConfigBuffer, PWR_CONTROL);
	// Run data path for all sequences

//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE_CAL_EN, 1, ConfigBuffer, STAGE_CAL_EN);
	ConfigBuffer[STAGE_CAL_EN]=0x0;//0x0FFF;	
	//Calibration configuration
	ConfigBuffer[AMB_COMP_CTRL0]=0x3230;//0x220;
	ConfigBuffer[AMB_COMP_CTRL1]=0x14C8;//0x14c8
	ConfigBuffer[AMB_COMP_CTRL2]=0x0832;//0xBFF;//0x0832;
	//Interrupt configuration
	ConfigBuffer[STAGE_LOW_INT_EN]=0x0000;
	ConfigBuffer[STAGE_HIGH_INT_EN]=0x0000;
	ConfigBuffer[STAGE_COMPLETE_INT_EN]=0x800;//0x0001;
//	ConfigBuffer[STAGE_LOW_LIMIT_INT]=0x0000;
//	ConfigBuffer[STAGE_HIGH_LIMIT_INT]=0x0000;
//	ConfigBuffer[STAGE_COMPLETE_LIMIT_INT]=0xFFF;//0x0FFF;
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],AMB_COMP_CTRL0, 9, ConfigBuffer, AMB_COMP_CTRL0);
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],AMB_COMP_CTRL0, 6, ConfigBuffer, AMB_COMP_CTRL0);

	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],PWR_CONTROL, 8, ConfigBuffer, PWR_CONTROL);
	// Run data path for all sequences
//	ConfigBuffer[STAGE_CAL_EN]=0x0;//0x0FFF;
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE_CAL_EN, 1, ConfigBuffer, STAGE_CAL_EN);
}

void ConfigAD7147_THREE(unsigned char Channel,unsigned int i,unsigned int pw_control_val, unsigned int *convalue)
{
	unsigned int ConfigBuffer[12];
	
	//=============================================
	//= Stage 0 - Connected to Vbias
	//=============================================
	ConfigBuffer[0]=0xFFAA;//0xFFFF; //1_2_3_4 (CIN0,CIN1,CIN2,CIN3)
	ConfigBuffer[1]=0x1FFF;//0x1FFF;//0x3FFF;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE0_CONNECTION, 8, ConfigBuffer, 0);	//Stage 9 connection registers - Start address in RAM 0xC8
	
	//=============================================
	//= Stage 1 - Connected to Vbias
	//=============================================
	ConfigBuffer[0]=0x2AFF;//0xEAFF; 5_6_7_8 (CIN4,CIN5,CIN6,CIN7)
	ConfigBuffer[1]=0x1FFe;//0x3FF2;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE1_CONNECTION, 8, ConfigBuffer, 0);//Stage 10 connection registers - Start address in RAM 0xD0
	
	//=============================================
	//= Stage 2 - Connected to Vbias
	//=============================================
	ConfigBuffer[0]=0xFFFF;//0xFFFF; 9_10_11_12 (CIN8,CIN9,CIN10,CIN11)
	ConfigBuffer[1]=0x1EAB;//0x3EAB;
	ConfigBuffer[2]=convalue[0];
	ConfigBuffer[3]=0x2626;
	ConfigBuffer[4]=50;
	ConfigBuffer[5]=50;
	ConfigBuffer[6]=100;
	ConfigBuffer[7]=100;
	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE2_CONNECTION, 8, ConfigBuffer, 0);//Stage 11 connection registers - Start address in RAM 0xD8
	
	//=============================================
	//= Configure 1st register bank
	//=============================================
	//Initialisation of the first register bank but not the AMBCOMPCTL_REG0
	ConfigBuffer[PWR_CONTROL]=pw_control_val;	// Full power mode enabled at 32ms - 4 sequences - 256 decimation factor
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],PWR_CONTROL, 1, ConfigBuffer, PWR_CONTROL);
	// Run data path for all sequences

//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE_CAL_EN, 1, ConfigBuffer, STAGE_CAL_EN);
	ConfigBuffer[STAGE_CAL_EN]=0x0;//0x0FFF;	
	//Calibration configuration
	ConfigBuffer[AMB_COMP_CTRL0]=0x3230;//0x220;
	ConfigBuffer[AMB_COMP_CTRL1]=0x14C8;//0x14c8
	ConfigBuffer[AMB_COMP_CTRL2]=0x0832;//0xBFF;//0x0832;
	//Interrupt configuration
	ConfigBuffer[STAGE_LOW_INT_EN]=0x0000;
	ConfigBuffer[STAGE_HIGH_INT_EN]=0x0000;
	ConfigBuffer[STAGE_COMPLETE_INT_EN]=0x800;//0x0001;
//	ConfigBuffer[STAGE_LOW_LIMIT_INT]=0x0000;
//	ConfigBuffer[STAGE_HIGH_LIMIT_INT]=0x0000;
//	ConfigBuffer[STAGE_COMPLETE_LIMIT_INT]=0xFFF;//0x0FFF;
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],AMB_COMP_CTRL0, 9, ConfigBuffer, AMB_COMP_CTRL0);
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],AMB_COMP_CTRL0, 6, ConfigBuffer, AMB_COMP_CTRL0);

	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],PWR_CONTROL, 8, ConfigBuffer, PWR_CONTROL);
	// Run data path for all sequences
//	ConfigBuffer[STAGE_CAL_EN]=0x0;//0x0FFF;
//	WriteToAD7147ViaI2C(Channel,AD7147_ADD[i],STAGE_CAL_EN, 1, ConfigBuffer, STAGE_CAL_EN);
}
