// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#ifndef __OUTPUT_H
#define __OUTPUT_H

#include "structs.h" // For TRMData

/*  OUTPUT FILE FORMAT CONSTANTS  */
#define AU_FILE_FORMAT            0
#define AIFF_FILE_FORMAT          1
#define WAVE_FILE_FORMAT          2

/*  FINAL OUTPUT SCALING, SO THAT .SND FILES APPROX. MATCH DSP OUTPUT  */
//#define OUTPUT_SCALE              0.25
#define OUTPUT_SCALE              1.0

/*  MAXIMUM SAMPLE VALUE  */
#define RANGE_MAX                 32767.0

/*  SIZE IN BITS PER OUTPUT SAMPLE  */
#define BITS_PER_SAMPLE           16


#ifdef __cplusplus
extern "C" {
#endif

void writeOutputToFile(TRMSampleRateConverter *sampleRateConverter, TRMData *data, const char *fileName);

void writeAuFileHeader(int channels, long int numberSamples, float outputRate, FILE *outputFile);
size_t fwriteShortMsb(int data, FILE *stream);

#ifdef __cplusplus
};
#endif


#endif
