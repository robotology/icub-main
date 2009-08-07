// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#include <stdlib.h>
#include <math.h>
#include "fir.h"
#include "tube.h"
#include "wavetable.h"

#include "local.h"


//  Glottal source oscillator table variables
#define TABLE_LENGTH              512
#define TABLE_MODULUS             (TABLE_LENGTH-1)

static double mod0(double value);
static void TRMWavetableIncrementPosition(TRMWavetable *wavetable, double frequency);

// Returns the modulus of 'value', keeping it in the range 0 -> TABLE_MODULUS.
static double mod0(double value)
{
    if (value > TABLE_MODULUS)
        value -= TABLE_LENGTH;

    return value;
}

// Calculates the initial glottal pulse and stores it in the wavetable, for use in the oscillator.
TRMWavetable *TRMWavetableCreate(int waveform, double tp, double tnMin, double tnMax, double sampleRate)
{
    TRMWavetable *newWavetable;
    int i, j;

    newWavetable = (TRMWavetable *)malloc(sizeof(TRMWavetable));
    if (newWavetable == NULL) {
        fprintf(stderr, "Failed to allocate space for new TRMWavetable in TRMWavetableCreate.\n");
        return NULL;
    }

    newWavetable->FIRFilter = TRMFIRFilterCreate(FIR_BETA, FIR_GAMMA, FIR_CUTOFF);

    //  Allocate memory for wavetable
    newWavetable->wavetable = (double *)calloc(TABLE_LENGTH, sizeof(double));
    if (newWavetable->wavetable == NULL) {
        fprintf(stderr, "Failed to allocate space for wavetable in TRMWavetableCreate.\n");
        free(newWavetable);
        return NULL;
    }

    //  Calculate wave table parameters
    newWavetable->tableDiv1 = rint(TABLE_LENGTH * (tp / 100.0));
    newWavetable->tableDiv2 = rint(TABLE_LENGTH * ((tp + tnMax) / 100.0));
    newWavetable->tnLength = newWavetable->tableDiv2 - newWavetable->tableDiv1;
    newWavetable->tnDelta = rint(TABLE_LENGTH * ((tnMax - tnMin) / 100.0));
    newWavetable->basicIncrement = (double)TABLE_LENGTH / sampleRate;
    newWavetable->currentPosition = 0;

    //  Initialize the wavetable with either a glottal pulse or sine tone
    if (waveform == PULSE) {
        //  Calculate rise portion of wave table
        for (i = 0; i < newWavetable->tableDiv1; i++) {
            double x = (double)i / (double)newWavetable->tableDiv1;
            double x2 = x * x;
            double x3 = x2 * x;
            newWavetable->wavetable[i] = (3.0 * x2) - (2.0 * x3);
        }

        //  Calculate fall portion of wave table
        for (i = newWavetable->tableDiv1, j = 0; i < newWavetable->tableDiv2; i++, j++) {
            double x = (double)j / newWavetable->tnLength;
            newWavetable->wavetable[i] = 1.0 - (x * x);
        }

        //  Set closed portion of wave table
        for (i = newWavetable->tableDiv2; i < TABLE_LENGTH; i++)
            newWavetable->wavetable[i] = 0.0;
    } else {
        //  Sine wave
        for (i = 0; i < TABLE_LENGTH; i++) {
            newWavetable->wavetable[i] = sin( ((double)i / (double)TABLE_LENGTH) * 2.0 * PI );
        }
    }

    return newWavetable;
}

void TRMWavetableFree(TRMWavetable *wavetable)
{
    if (wavetable == NULL)
        return;

    if (FIRFilter != NULL) {
        TRMFIRFilterFree(wavetable->FIRFilter);
        wavetable->FIRFilter = NULL;
    }

    if (wavetable->wavetable != NULL) {
        free(wavetable->wavetable);
        wavetable->wavetable = NULL;
    }

    free(wavetable);
}


// Rewrites the changeable part of the glottal pulse according to the amplitude.
void TRMWavetableUpdate(TRMWavetable *wavetable, double amplitude)
{
    int i;

    //  Calculate new closure point, based on amplitude
    double newDiv2 = wavetable->tableDiv2 - rint(amplitude * wavetable->tnDelta);
    double newTnLength = newDiv2 - wavetable->tableDiv1;
    double j;

    //  Recalculate the falling portion of the glottal pulse
#ifdef USE_VECLIB
    {
        double aj[TABLE_LENGTH], ajj[TABLE_LENGTH], one[TABLE_LENGTH];
        double scale = 1.0 / (newTnLength * newTnLength);
        int len;

        len = newTnLength;
        for (i = 0, j = 0.0; i < len; i++, j += 1.0) {
            aj[i] = j;
            one[i] = 1.0;
        }

        vsqD(aj, 1, ajj, 1, len);
        vsmulD(ajj, 1, &scale, aj, 1, len);
        vsubD(aj, 1, one, 1, &(wavetable->wavetable[wavetable->tableDiv1]), 1, len); // The docs seem to be wrong about which one gets subtracted...
    }
#else
    {
        for (i = wavetable->tableDiv1, j = 0.0; i < newDiv2; i++, j++) {
            double x = j / newTnLength;
            wavetable->wavetable[i] = 1.0 - (x * x);
        }
    }

#endif

    //  Fill in with closed portion of glottal pulse
#if 1
    for (i = (int)newDiv2; i < (int)(wavetable->tableDiv2); i++)
        wavetable->wavetable[i] = 0.0;
#else
    i = newDiv2;
    if (wavetable->tableDiv2 > i)
        memset(&(wavetable[i]), 0, (wavetable->tableDiv2 - i) * sizeof(double)); // This seems to be crashy... possibly with 0 sizes?
#endif
}



// Increments the position in the wavetable according to the desired frequency.
static void TRMWavetableIncrementPosition(TRMWavetable *wavetable, double frequency)
{
    wavetable->currentPosition = mod0(wavetable->currentPosition + (frequency * wavetable->basicIncrement));
}

// A 2X oversampling interpolating wavetable oscillator.

#if OVERSAMPLING_OSCILLATOR
double TRMWavetableOscillator(TRMWavetable *wavetable, double frequency)  //  2X oversampling oscillator
{
    int i, lowerPosition, upperPosition;
    double interpolatedValue, output;


    for (i = 0; i < 2; i++) {
        //  First increment the table position, depending on frequency
        TRMWavetableIncrementPosition(wavetable, frequency / 2.0);

        //  Find surrounding integer table positions
        lowerPosition = (int)wavetable->currentPosition;
        upperPosition = (int)mod0(lowerPosition + 1);

        //  Calculate interpolated table value
        interpolatedValue = (wavetable->wavetable[lowerPosition] +
                             ((wavetable->currentPosition - lowerPosition) *
                              (wavetable->wavetable[upperPosition] - wavetable->wavetable[lowerPosition])));

        //  Put value through FIR filter
        output = FIRFilter(wavetable->FIRFilter, interpolatedValue, i);
    }

    //  Since we decimate, take only the second output value
    return output;
}
#else
double TRMWavetableOscillator(TRMWavetable *wavetable, double frequency)  //  Plain oscillator
{
    int lowerPosition, upperPosition;


    //  First increment the table position, depending on frequency
    TRMWavetableIncrementPosition(wavetable, frequency);

    //  Find surrounding integer table positions
    lowerPosition = (int)wavetable->currentPosition;
    upperPosition = mod0(lowerPosition + 1);

    //  Return interpolated table value
    return (wavetable->wavetable[lowerPosition] +
            ((wavetable->currentPosition - lowerPosition) *
             (wavetable->wavetable[upperPosition] - wavetable->wavetable[lowerPosition])));
}
#endif
