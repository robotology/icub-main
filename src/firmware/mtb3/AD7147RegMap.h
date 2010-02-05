//========================================================================================
//File AD7147RegMap.h
//Data of creation: 12th May 2005
//Author: ADI Limerick
//
//Description 
//===========
//This file contains the AD7147 register and ram maps.
//========================================================================================
   

// Register map
//=============
#define	PWR_CONTROL					0x00	// RW	Power & conversion control

#define	STAGE_CAL_EN				0x01	// RW	Calibration enable register
#define	AMB_COMP_CTRL0				0x02	// RW	Ambient compensation control register 0
#define	AMB_COMP_CTRL1				0x03	// RW	Ambient compensation control register 1
#define	AMB_COMP_CTRL2				0x04	// RW	Ambient compensation control register 2

#define	STAGE_LOW_INT_EN			0x05	// RW	Low Limit Interrupt enable register
#define	STAGE_HIGH_INT_EN			0x06	// RW	High Limit Interrupt enable register
#define	STAGE_COMPLETE_INT_EN		0x07	// RW	Conversion Interrupt enable register
#define	STAGE_LOW_LIMIT_INT			0x08	// R	Low limit interrupt status register
#define	STAGE_HIGH_LIMIT_INT		0x09	// R	High limit interrupt status register
#define	STAGE_COMPLETE_LIMIT_INT	0x0A	// R	Conversion Interrupt status register

// I have changed it in order to start from 0

#define	ADCRESULT_S0			     0x00 //0x0E0//	// R	ADC stage 0 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S1			 	 0x01 //0x104// // R	ADC stage 1 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S2				 0x02 //0x128//	// R	ADC stage 2 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S3				 0x03 //0x14C//	// R	ADC stage 3 result (uncompensated) actually located in SRAM

#define	ADCRESULT_S4				 0x04 //0x170//	// R	ADC stage 4 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S5				 0x05 //0x194// // R	ADC stage 5 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S6				 0x06 //0x1B8//	// R	ADC stage 6 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S7			     0x07 //0x1DC//	// R	ADC stage 7 result (uncompensated) actually located in SRAM

#define	ADCRESULT_S8				 0x08 //0x200//	// R	ADC stage 8 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S9				 0x09 //0x224//	// R	ADC stage 9 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S10				 0x0A //0x248//	// R	ADC stage 10 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S11				 0x0B //0x26C//	// R	ADC stage 11 result (uncompensated) actually located in SRAM

#define	DEVID					   	0x17	// R	I.D. Register

#define	THRES_STAT_REG0				0x40	// R	Current threshold status register 0
#define	THRES_STAT_REG1				0x41	// R	Current threshold status register 1
#define	PROX_STAT_REG				0x42	// R	Current proximity status register 2


// Ram map - these registers are defined as we go along
//=====================================================
#define STAGE0_CONNECTION	0x80
#define STAGE1_CONNECTION	0x88
#define STAGE2_CONNECTION	0x90
#define STAGE3_CONNECTION	0x98
#define STAGE4_CONNECTION	0xA0
#define STAGE5_CONNECTION	0xA8
#define STAGE6_CONNECTION	0xB0
#define STAGE7_CONNECTION	0xB8
#define STAGE8_CONNECTION	0xC0
#define STAGE9_CONNECTION	0xC8
#define STAGE10_CONNECTION	0xD0
#define STAGE11_CONNECTION	0xD8


#define STAGE0				0xE0
#define STAGE0_AMBIENT		0xF1
#define STAGE0_MAX_AVG		0xF9
#define STAGE0_UPP_THRES	0xFA
#define STAGE0_MIN_AVG		0x100
#define STAGE0_LWR_THRES	0x101

#define STAGE1				0x104
#define STAGE1_AMBIENT		0x115
#define STAGE1_MAX_AVG		0x11D
#define STAGE1_UPP_THRES	0x11E
#define STAGE1_MIN_AVG		0x124
#define STAGE1_LWR_THRES	0x125

#define STAGE2				0x128
#define STAGE2_AMBIENT		0x139
#define STAGE2_MAX_AVG		0x141
#define STAGE2_UPP_THRES	0x142
#define STAGE2_MIN_AVG		0x148
#define STAGE2_LWR_THRES	0x149

#define STAGE3				0x14C
#define STAGE3_AMBIENT		0x15D
#define STAGE3_MAX_AVG		0x165
#define STAGE3_UPP_THRES	0x166
#define STAGE3_MIN_AVG		0x16C
#define STAGE3_LWR_THRES	0x16D

#define STAGE4				0x170
#define STAGE4_AMBIENT		0x181
#define STAGE4_MAX_AVG		0x189
#define STAGE4_UPP_THRES	0x18A
#define STAGE4_MIN_AVG		0x190
#define STAGE4_LWR_THRES	0x191

#define STAGE5				0x194
#define STAGE5_AMBIENT		0x1A5
#define STAGE5_MAX_AVG		0x1AD
#define STAGE5_UPP_THRES	0x1AE
#define STAGE5_MIN_AVG		0x1B4
#define STAGE5_LWR_THRES	0x1B5

#define STAGE6				0x1B8
#define STAGE6_AMBIENT		0x1C9
#define STAGE6_MAX_AVG		0x1D1
#define STAGE6_UPP_THRES	0x1D2
#define STAGE6_MIN_AVG		0x1D8
#define STAGE6_LWR_THRES	0x1D9

#define STAGE7				0x1DC
#define STAGE7_AMBIENT		0x1ED
#define STAGE7_MAX_AVG		0x1F5
#define STAGE7_UPP_THRES	0x1F6
#define STAGE7_MIN_AVG		0x1FC
#define STAGE7_LWR_THRES	0x1FD

#define STAGE8				0x200
#define STAGE8_AMBIENT		0x211
#define STAGE8_MAX_AVG		0x219
#define STAGE8_UPP_THRES	0x21A
#define STAGE8_MIN_AVG		0x220
#define STAGE8_LWR_THRES	0x221

#define STAGE9				0x224
#define STAGE9_AMBIENT		0x235
#define STAGE9_MAX_AVG		0x23D
#define STAGE9_UPP_THRES	0x23E
#define STAGE9_MIN_AVG		0x244
#define STAGE9_LWR_THRES	0x245

#define STAGE10				0x248
#define STAGE10_AMBIENT		0x259
#define STAGE10_MAX_AVG		0x261
#define STAGE10_UPP_THRES	0x262
#define STAGE10_MIN_AVG		0x268
#define STAGE10_LWR_THRES	0x269

#define STAGE11				0x26C
#define STAGE11_AMBIENT		0x27D
#define STAGE11_MAX_AVG		0x285
#define STAGE11_UPP_THRES	0x286
#define STAGE11_MIN_AVG		0x28C
#define STAGE11_LWR_THRES	0x28D
