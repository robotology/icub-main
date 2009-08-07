// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "input.h"

/*  VARIABLES FOR INPUT TABLE STORAGE  */

static INPUT *newInputTable(void);
static int inputTableLength(INPUT *ptr);

/******************************************************************************
*
*       function:       parseInputFile
*
*       purpose:        Parses the input file and assigns values to global
*                       variables.
*
*       arguments:      inputFile
*
*       internal
*       functions:      addInput, glotPitchAt, glotVolAt, aspVolAt, fricVolAt,
*                       fricPosAt, fricCFAt, fricBWAt, radiiAt, velumAt
*
*       library
*       functions:      fopen, fprintf, fgets, strtol, strod, fclose
*
******************************************************************************/

TRMData *parseInputFile(const char *inputFile)
{
    int i;
    FILE *fp;
    char line[128];
    int numberInputTables = 0;
    TRMData data, *result;


    /*  OPEN THE INPUT FILE  */
    if ((fp = fopen(inputFile, "r")) == NULL) {
        fprintf(stderr, "Can't open input file \"%s\".\n", inputFile);
        return NULL;
    }


    /*  GET THE OUTPUT FILE FORMAT  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read output file format.\n");
        return NULL;
    } else
        data.inputParameters.outputFileFormat = strtol(line, NULL, 10);

    /*  GET THE OUTPUT SAMPLE RATE  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read output sample rate.\n");
        return NULL;
    } else
        data.inputParameters.outputRate = strtod(line, NULL);

    /*  GET THE INPUT CONTROL RATE  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read input control rate.\n");
        return NULL;
    } else
        data.inputParameters.controlRate = strtod(line, NULL);


    /*  GET THE MASTER VOLUME  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read master volume.\n");
        return NULL;
    } else
        data.inputParameters.volume = strtod(line, NULL);

    /*  GET THE NUMBER OF SOUND OUTPUT CHANNELS  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read number of sound output channels.\n");
        return NULL;
    } else
        data.inputParameters.channels = strtol(line, NULL, 10);

    /*  GET THE STEREO BALANCE  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read stereo balance.\n");
        return NULL;
    } else
        data.inputParameters.balance = strtod(line, NULL);


    /*  GET THE GLOTTAL SOURCE WAVEFORM TYPE  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read glottal source waveform type.\n");
        return NULL;
    } else
        data.inputParameters.waveform = strtol(line, NULL, 10);

    /*  GET THE GLOTTAL PULSE RISE TIME (tp)  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read glottal pulse rise time (tp).\n");
        return NULL;
    } else
        data.inputParameters.tp = strtod(line, NULL);

    /*  GET THE GLOTTAL PULSE FALL TIME MINIMUM (tnMin)  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read glottal pulse fall time minimum (tnMin).\n");
        return NULL;
    } else
        data.inputParameters.tnMin = strtod(line, NULL);

    /*  GET THE GLOTTAL PULSE FALL TIME MAXIMUM (tnMax)  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read glottal pulse fall time maximum (tnMax).\n");
        return NULL;
    } else
        data.inputParameters.tnMax = strtod(line, NULL);

    /*  GET THE GLOTTAL SOURCE BREATHINESS  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read glottal source breathiness.\n");
        return NULL;
    } else
        data.inputParameters.breathiness = strtod(line, NULL);


    /*  GET THE NOMINAL TUBE LENGTH  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read nominal tube length.\n");
        return NULL;
    } else
        data.inputParameters.length = strtod(line, NULL);

    /*  GET THE TUBE TEMPERATURE  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read tube temperature.\n");
        return NULL;
    } else
        data.inputParameters.temperature = strtod(line, NULL);

    /*  GET THE JUNCTION LOSS FACTOR  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read junction loss factor.\n");
        return NULL;
    } else
        data.inputParameters.lossFactor = strtod(line, NULL);


    /*  GET THE APERTURE SCALING RADIUS  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read aperture scaling radius.\n");
        return NULL;
    } else
        data.inputParameters.apScale = strtod(line, NULL);

    /*  GET THE MOUTH APERTURE COEFFICIENT  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read mouth aperture coefficient\n");
        return NULL;
    } else
        data.inputParameters.mouthCoef = strtod(line, NULL);

    /*  GET THE NOSE APERTURE COEFFICIENT  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read nose aperture coefficient\n");
        return NULL;
    } else
        data.inputParameters.noseCoef = strtod(line, NULL);


    /*  GET THE NOSE RADII  */
    for (i = 1; i < TOTAL_NASAL_SECTIONS; i++) {
        if (fgets(line, 128, fp) == NULL) {
            fprintf(stderr, "Can't read nose radius %-d.\n", i);
            return NULL;
        } else
            data.inputParameters.noseRadius[i] = strtod(line, NULL);
    }


    /*  GET THE THROAT LOWPASS FREQUENCY CUTOFF  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read throat lowpass filter cutoff.\n");
        return NULL;
    } else
        data.inputParameters.throatCutoff = strtod(line, NULL);

    /*  GET THE THROAT VOLUME  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read throat volume.\n");
        return NULL;
    } else
        data.inputParameters.throatVol = strtod(line, NULL);


    /*  GET THE PULSE MODULATION OF NOISE FLAG  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read pulse modulation of noise flag.\n");
        return NULL;
    } else
        data.inputParameters.modulation = strtol(line, NULL, 10);

    /*  GET THE NOISE CROSSMIX OFFSET  */
    if (fgets(line, 128, fp) == NULL) {
        fprintf(stderr, "Can't read noise crossmix offset.\n");
        return NULL;
    } else
        data.inputParameters.mixOffset = strtod(line, NULL);


    data.inputHead = NULL;
    data.inputTail = NULL;

    /*  GET THE INPUT TABLE VALUES  */
    while (fgets(line, 128, fp)) {
        double glotPitch, glotVol, radius[TOTAL_REGIONS], velum, aspVol;
        double fricVol, fricPos, fricCF, fricBW;
        char *ptr = line;

        /*  GET EACH PARAMETER  */
        glotPitch = strtod(ptr, &ptr);
        glotVol = strtod(ptr, &ptr);
        aspVol = strtod(ptr, &ptr);
        fricVol = strtod(ptr, &ptr);
        fricPos = strtod(ptr, &ptr);
        fricCF = strtod(ptr, &ptr);
        fricBW = strtod(ptr, &ptr);
        for (i = 0; i < TOTAL_REGIONS; i++)
            radius[i] = strtod(ptr, &ptr);
        velum = strtod(ptr, &ptr);

        /*  ADD THE PARAMETERS TO THE INPUT LIST  */
        addInput(&data, glotPitch, glotVol, aspVol, fricVol, fricPos, fricCF, fricBW, radius, velum);
        numberInputTables++;
    }
#if 0
    /*  DOUBLE UP THE LAST INPUT TABLE, TO HELP INTERPOLATION CALCULATIONS  */
    if (numberInputTables > 0) {
        addInput(&data, glotPitchAt(data.inputTail), glotVolAt(data.inputTail),
                 aspVolAt(data.inputTail), fricVolAt(data.inputTail),
                 fricPosAt(data.inputTail), fricCFAt(data.inputTail),
                 fricBWAt(data.inputTail), radiiAt(data.inputTail),
                 velumAt(data.inputTail));
    }
#endif
    /*  CLOSE THE INPUT FILE  */
    fclose(fp);

    result = (TRMData *)malloc(sizeof(TRMData));
    if (result == NULL) {
        fprintf(stderr, "Couldn't malloc() TRMData.\n");
        return NULL;
    }

    memcpy(result, &data, sizeof(TRMData));

    return result;
}

/******************************************************************************
*
*       function:       addInput
*
*       purpose:        Adds table control data to the end of a linked list.
*
*       arguments:      glotPitch, glotVol, radius, velum, aspVol,
*                       fricVol, fricPos,
*                       fricCF, fricBW
*
*       internal
*       functions:      newInputTable
*
*       library
*       functions:      none
*
******************************************************************************/

void addInput(TRMData *data, double glotPitch, double glotVol, double aspVol, double fricVol,
              double fricPos, double fricCF, double fricBW, double *radius,
              double velum)
{
    int i;
    INPUT *tempPtr;
#if 0
    printf("addInput(%p, %8.4g %8.4g %8.4g %8.4g %8.4g %8.4g %8.4g [%8.4g %8.4g %8.4g %8.4g %8.4g %8.4g %8.4g %8.4g] %8.4g)\n", data,
           glotPitch, glotVol, aspVol, fricVol, fricPos, fricCF, fricBW,
           radius[0], radius[1], radius[2], radius[3], radius[4], radius[5], radius[6], radius[7],
           velum);
#endif
    if (data->inputHead == NULL) {
        data->inputTail = data->inputHead = newInputTable();
        data->inputTail->previous = NULL;
    } else {
        tempPtr = data->inputTail;
        data->inputTail = tempPtr->next = newInputTable();
        data->inputTail->previous = tempPtr;
    }

    /*  SET NULL POINTER TO NEXT, SINCE END OF LIST  */
    data->inputTail->next = NULL;

    /*  ADD GLOTTAL PITCH AND VOLUME  */
    data->inputTail->parameters.glotPitch = glotPitch;
    data->inputTail->parameters.glotVol = glotVol;

    /*  ADD ASPIRATION  */
    data->inputTail->parameters.aspVol = aspVol;

    /*  ADD FRICATION PARAMETERS  */
    data->inputTail->parameters.fricVol = fricVol;
    data->inputTail->parameters.fricPos = fricPos;
    data->inputTail->parameters.fricCF = fricCF;
    data->inputTail->parameters.fricBW = fricBW;

    /*  ADD TUBE REGION RADII  */
    for (i = 0; i < TOTAL_REGIONS; i++)
        data->inputTail->parameters.radius[i] = radius[i];

    /*  ADD VELUM RADIUS  */
    data->inputTail->parameters.velum = velum;
}



/******************************************************************************
*
*       function:       newInputTable
*
*       purpose:        Allocates memory for a new input table.
*
*       arguments:      none
*
*       internal
*       functions:      none
*
*       library
*       functions:      malloc
*
******************************************************************************/

INPUT *newInputTable(void)
{
    return ((INPUT *)malloc(sizeof(INPUT)));
}

// Returns the pitch stored in the table 'ptr'.
double glotPitchAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.glotPitch;

    return 0.0;
}

// Returns the glotVol stored in the table 'ptr'.
double glotVolAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.glotVol;

    return 0.0;
}

// Returns the variable tube radii stored in the table 'ptr'.
double *radiiAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.radius;

    return NULL;
}

// Returns the radius for 'region', from the table 'ptr'.
double radiusAtRegion(INPUT *ptr, int region)
{
    if (ptr)
        return ptr->parameters.radius[region];

    return 0.0;
}

// Returns the velum radius from the table 'ptr'.
double velumAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.velum;

    return 0.0;
}

// Returns the aspiration factor from the table 'ptr'.
double aspVolAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.aspVol;

    return 0.0;
}

// Returns the frication volume from the table 'ptr'.
double fricVolAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.fricVol;

    return 0.0;
}

// Returns the frication position from the table 'ptr'.
double fricPosAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.fricPos;

    return 0.0;
}

// Returns the frication center frequency from the table 'ptr'.
double fricCFAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.fricCF;

    return 0.0;
}

// Returns the frication bandwidth from the table 'ptr'.
double fricBWAt(INPUT *ptr)
{
    if (ptr)
        return ptr->parameters.fricBW;

    return 0.0;
}

int inputTableLength(INPUT *ptr)
{
    int count = 0;

    while (ptr) {
        count++;
        ptr = ptr->next;
    }

    return count;
}

void printControlRateInputTable(TRMData *data)
{
    INPUT *ptr;
    int index;

    /*  ECHO TABLE VALUES  */
    printf("\n%-d control rate input tables:\n\n", inputTableLength(data->inputHead));

    /*  HEADER  */
    printf("glPitch");
    printf("\tglotVol");
    printf("\taspVol");
    printf("\tfricVol");
    printf("\tfricPos");
    printf("\tfricCF");
    printf("\tfricBW");
    for (index = 0; index < TOTAL_REGIONS; index++)
        printf("\tr%-d", index + 1);
    printf("\tvelum\n");

    /*  ACTUAL VALUES  */
    ptr = data->inputHead;
    while (ptr != NULL) {
        TRMParameters *parameters;

        parameters = &(ptr->parameters);
        printf("%.2f", parameters->glotPitch);
        printf("\t%.2f", parameters->glotVol);
        printf("\t%.2f", parameters->aspVol);
        printf("\t%.2f", parameters->fricVol);
        printf("\t%.2f", parameters->fricPos);
        printf("\t%.2f", parameters->fricCF);
        printf("\t%.2f", parameters->fricBW);
        for (index = 0; index < TOTAL_REGIONS; index++)
            printf("\t%.2f", parameters->radius[index]);
        printf("\t%.2f\n", parameters->velum);
        ptr = ptr->next;
    }
    printf("\n");
}
