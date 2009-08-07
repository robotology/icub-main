// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#ifndef __WAVETABLE_H
#define __WAVETABLE_H

#include "fir.h"

//  Compile with oversampling or plain oscillator
#define OVERSAMPLING_OSCILLATOR   1

typedef struct _TRMWavetable {
    TRMFIRFilter *FIRFilter;
    double *wavetable;

    int tableDiv1;
    int tableDiv2;
    double tnLength;
    double tnDelta;

    double basicIncrement;
    double currentPosition;
} TRMWavetable;

TRMWavetable *TRMWavetableCreate(int waveform, double tp, double tnMin, double tnMax, double sampleRate);
void TRMWavetableFree(TRMWavetable *wavetable);

void TRMWavetableUpdate(TRMWavetable *wavetable, double amplitude);
double TRMWavetableOscillator(TRMWavetable *wavetable, double frequency);

#endif
