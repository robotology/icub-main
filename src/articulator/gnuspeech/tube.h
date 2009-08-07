// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#ifndef __TUBE_H
#define __TUBE_H

#include <stdio.h> // For FILE
#include "input.h" // For INPUT
#include "structs.h" // For TRMSampleRateConverter

/*  FUNCTION RETURN CONSTANTS  */
#define ERROR                     (-1)
#define SUCCESS                   0

/*  WAVEFORM TYPES  */
#define PULSE                     0
#define SINE                      1

/*  MATH CONSTANTS  */
#define PI                        3.14159265358979
#define TWO_PI                    (2.0 * PI)

//extern int controlPeriod;
//extern int sampleRate;
//extern double actualTubeLength;

#ifdef __cplusplus
extern "C" {
#endif

TRMTubeModel *TRMTubeModelCreate(TRMInputParameters *inputParameters);
void TRMTubeModelFree(TRMTubeModel *model);

void synthesize(TRMTubeModel *tubeModel, TRMData *data);


#ifdef __cplusplus
};
#endif


#endif
