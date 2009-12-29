/*--------------------------------------------------------------------------*/
// fglove.h
//
// 5DT Data Glove driver SDK
// Version 1.02
//
// Copyright (C) 2000, 5DT <Fifth Dimension Technologies>
// http://www.5dt.com/
/*--------------------------------------------------------------------------*/
#ifndef _FGLOVE_H_
#define _FGLOVE_H_
/*--------------------------------------------------------------------------*/
enum EfdGloveHand
{
	FD_HAND_LEFT,   // left-handed glove
	FD_HAND_RIGHT	// right-handed glove
};

enum EfdGloveTypes
{
	FD_GLOVENONE,   // no glove
	FD_GLOVE7,      // 7-sensor
	FD_GLOVE7W,     // 7-sensor, wireless
	FD_GLOVE16,     // 16-sensor
	FD_GLOVE16W     // 16-sensor, wireless
};

enum EfdSensors
{
	FD_THUMBNEAR=0,
	FD_THUMBFAR,
	FD_THUMBINDEX,
	FD_INDEXNEAR,
	FD_INDEXFAR,
	FD_INDEXMIDDLE,
	FD_MIDDLENEAR,
	FD_MIDDLEFAR,
	FD_MIDDLERING,
	FD_RINGNEAR,
	FD_RINGFAR,
	FD_RINGLITTLE,
	FD_LITTLENEAR,
	FD_LITTLEFAR,
	FD_THUMBPALM,
	FD_WRISTBEND,
	FD_PITCH,
	FD_ROLL
};
/*--------------------------------------------------------------------------*/
typedef struct
{
	// The contents of this struct are platform-dependent and subject to
	// change. You should not manipulate the contents of this struct directly.
	void           *m_pStuff;
} fdGlove;
/*--------------------------------------------------------------------------*/
fdGlove *fdOpen(char *pPort);
int   fdClose(fdGlove *pFG);
int   fdGetGloveHand(fdGlove *pFG);
int   fdGetGloveType(fdGlove *pFG);
int   fdGetNumSensors(fdGlove *pFG);
void  fdGetSensorRawAll(fdGlove *pFG, unsigned short *pData);
unsigned short fdGetSensorRaw(fdGlove *pFG, int nSensor);
void  fdSetSensorRawAll(fdGlove *pFG, unsigned short *pData);
void  fdSetSensorRaw(fdGlove *pFG, int nSensor, unsigned short nRaw);
void  fdGetSensorScaledAll(fdGlove *pFG, float *pData);
float fdGetSensorScaled(fdGlove *pFG, int nSensor);
int   fdGetNumGestures(fdGlove *pFG);
int   fdGetGesture(fdGlove *pFG);
void  fdGetCalibrationAll(fdGlove *pFG, unsigned short *pUpper, unsigned short *pLower);
void  fdGetCalibration(fdGlove *pFG, int nSensor, unsigned short *pUpper, unsigned short *pLower);
void  fdSetCalibrationAll(fdGlove *pFG, unsigned short *pUpper, unsigned short *pLower);
void  fdSetCalibration(fdGlove *pFG, int nSensor, unsigned short nUpper, unsigned short nLower);
void  fdResetCalibration(fdGlove *pFG);
void  fdGetSensorMaxAll(fdGlove *pFG, float *pMax);
float fdGetSensorMax(fdGlove *pFG, int nSensor);
void  fdSetSensorMaxAll(fdGlove *pFG, float *pMax);
void  fdSetSensorMax(fdGlove *pFG, int nSensor, float fMax);
void  fdGetThresholdAll(fdGlove *pFG, float *pUpper, float *pLower);
void  fdGetThreshold(fdGlove *pFG, int nSensor, float *pUpper, float *pLower);
void  fdSetThresholdAll(fdGlove *pFG, float *pUpper, float *pLower);
void  fdSetThreshold(fdGlove *pFG, int nSensor, float fUpper, float fLower);
void  fdGetGloveInfo(fdGlove *pFG, unsigned char *pData);
void  fdGetDriverInfo(fdGlove *pFG, unsigned char *pData);
/*--------------------------------------------------------------------------*/
#endif // #ifndef _FGLOVE_H_
/*--------------------------------------------------------------------------*/
