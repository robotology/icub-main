// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
 * Based on GNUSPEECH code, which is: 
 *
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "tube.h"
#include "input.h"
#include "output.h"
#include "structs.h"
#include "listen.h"

int remote = 0;
int verbose = 0;

void printInfo2(struct _TRMData *data, char *inputFile)
{
#if 1
    int i;

    /*  PRINT INPUT FILE NAME  */
    printf("input file:\t\t%s\n\n", inputFile);

    /*  ECHO INPUT PARAMETERS  */
    printf("outputFileFormat:\t");
    if (data->inputParameters.outputFileFormat == AU_FILE_FORMAT)
        printf("AU\n");
    else if (data->inputParameters.outputFileFormat == AIFF_FILE_FORMAT)
        printf("AIFF\n");
    else if (data->inputParameters.outputFileFormat == WAVE_FILE_FORMAT)
        printf("WAVE\n");

    printf("outputRate:\t\t%.1f Hz\n", data->inputParameters.outputRate);
    printf("controlRate:\t\t%.2f Hz\n\n", data->inputParameters.controlRate);

    printf("volume:\t\t\t%.2f dB\n", data->inputParameters.volume);
    printf("channels:\t\t%-d\n", data->inputParameters.channels);
    printf("balance:\t\t%+1.2f\n\n", data->inputParameters.balance);

    printf("waveform:\t\t");
    if (data->inputParameters.waveform == PULSE)
	printf("pulse\n");
    else if (data->inputParameters.waveform == SINE)
	printf("sine\n");
    printf("tp:\t\t\t%.2f%%\n", data->inputParameters.tp);
    printf("tnMin:\t\t\t%.2f%%\n", data->inputParameters.tnMin);
    printf("tnMax:\t\t\t%.2f%%\n", data->inputParameters.tnMax);
    printf("breathiness:\t\t%.2f%%\n\n", data->inputParameters.breathiness);

    printf("nominal tube length:\t%.2f cm\n", data->inputParameters.length);
    printf("temperature:\t\t%.2f degrees C\n", data->inputParameters.temperature);
    printf("lossFactor:\t\t%.2f%%\n\n", data->inputParameters.lossFactor);

    printf("apScale:\t\t%.2f cm\n", data->inputParameters.apScale);
    printf("mouthCoef:\t\t%.1f Hz\n", data->inputParameters.mouthCoef);
    printf("noseCoef:\t\t%.1f Hz\n\n", data->inputParameters.noseCoef);

    for (i = 1; i < TOTAL_NASAL_SECTIONS; i++)
	printf("n%-d:\t\t\t%.2f cm\n", i, data->inputParameters.noseRadius[i]);

    printf("\nthroatCutoff:\t\t%.1f Hz\n", data->inputParameters.throatCutoff);
    printf("throatVol:\t\t%.2f dB\n\n", data->inputParameters.throatVol);

    printf("modulation:\t\t");
    if (data->inputParameters.modulation)
	printf("on\n");
    else
	printf("off\n");
    printf("mixOffset:\t\t%.2f dB\n\n", data->inputParameters.mixOffset);

    /*  PRINT OUT DERIVED VALUES  */
    //printf("\nactual tube length:\t%.4f cm\n", actualTubeLength);
    //printf("internal sample rate:\t%-d Hz\n", sampleRate);
    //printf("control period:\t\t%-d samples (%.4f seconds)\n\n",
    //controlPeriod, (float)controlPeriod/(float)sampleRate);

#if DEBUG
    /*  PRINT OUT WAVE TABLE VALUES  */
    printf("\n");
    for (i = 0; i < TABLE_LENGTH; i++)
	printf("table[%-d] = %.4f\n", i, wavetable[i]);
#endif

    //printControlRateInputTable(data);
#endif
}

int main(int argc, char *argv[]) {
    char *inputFile = "emergency.txt";
    TRMData *inputData;
    TRMTubeModel *tube;
	remote = true;

    TRMData data;
    data.inputParameters.outputFileFormat = 2;
    data.inputParameters.outputRate = 22050.0;
    data.inputParameters.controlRate = 250.0;
    data.inputParameters.volume = 60;
    data.inputParameters.channels = 2;
    data.inputParameters.balance = 0.0;
    data.inputParameters.waveform = 0;
    data.inputParameters.tp = 40.0;
    data.inputParameters.tnMin = 16.0;
    data.inputParameters.tnMax = 23.0;
    data.inputParameters.breathiness = 1.0;
    data.inputParameters.length = 17.5;
    data.inputParameters.temperature = 25;
    data.inputParameters.lossFactor = 0.5;
    data.inputParameters.apScale = 3.05;
    data.inputParameters.mouthCoef = 5000.0;
    data.inputParameters.noseCoef = 500.0;
    data.inputParameters.noseRadius[0] = 1.35;
    data.inputParameters.noseRadius[1] = 1.96;
    data.inputParameters.noseRadius[2] = 1.91;
    data.inputParameters.noseRadius[3] = 1.3;
    data.inputParameters.noseRadius[4] = 0.73;
    data.inputParameters.throatCutoff = 1500.0;
    data.inputParameters.throatVol = 6.0;
    data.inputParameters.modulation = 1;
    data.inputParameters.mixOffset = 54.0;

    data.inputHead = NULL;
    data.inputTail = NULL;

    int numberInputTables = 0;
    for (int i=0; i<100; i++) {
        double radius[TOTAL_REGIONS] =
            { 0.800, 0.890, 0.990, 0.810, 
              0.760, 1.050, 1.230, 0.010 };
        addInput(&data, -17.243, 0.000, 0.000, 0.000, 5.500, 
                 2500.000, 500.000, radius, 0.100);
        numberInputTables++;
    }
    addInput(&data, glotPitchAt(data.inputTail), glotVolAt(data.inputTail),
             aspVolAt(data.inputTail), fricVolAt(data.inputTail),
             fricPosAt(data.inputTail), fricCFAt(data.inputTail),
             fricBWAt(data.inputTail), radiiAt(data.inputTail),
             velumAt(data.inputTail));

    inputData = (TRMData *)malloc(sizeof(TRMData));
    if (inputData == NULL) {
        fprintf(stderr, "Couldn't allocate TRMData.\n");
        return 1;
    }
    memcpy(inputData, &data, sizeof(TRMData));


    tube = TRMTubeModelCreate(&(inputData->inputParameters));
    if (tube == NULL) {
        fprintf(stderr, "Cannot create tube model\n");
        exit(-1);
    }

    printInfo2(inputData, inputFile);


	init_listen("/gnuspeech");

    synthesize(tube, inputData);

    TRMTubeModelFree(tube);

    return 0;
}
