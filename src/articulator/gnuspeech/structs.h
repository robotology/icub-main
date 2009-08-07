// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#ifndef __STRUCTS_H
#define __STRUCTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h> // For FILE
#include "ring_buffer.h"
#include "wavetable.h"

/*  OROPHARYNX REGIONS  */
#define TRM_R1                        0      /*  S1  */
#define TRM_R2                        1      /*  S2  */
#define TRM_R3                        2      /*  S3  */
#define TRM_R4                        3      /*  S4 & S5  */
#define TRM_R5                        4      /*  S6 & S7  */
#define TRM_R6                        5      /*  S8  */
#define TRM_R7                        6      /*  S9  */
#define TRM_R8                        7      /*  S10  */
#define TOTAL_REGIONS             8

/*  NASAL TRACT SECTIONS  */
#define TRM_N1                        0
#define TRM_VELUM                     TRM_N1
#define TRM_N2                        1
#define TRM_N3                        2
#define TRM_N4                        3
#define TRM_N5                        4
#define TRM_N6                        5
#define TOTAL_NASAL_SECTIONS      6


/*  SAMPLE RATE CONVERSION CONSTANTS  */
#define ZERO_CROSSINGS            13                 /*  SRC CUTOFF FRQ      */
#define LP_CUTOFF                 (11.0/13.0)        /*  (0.846 OF NYQUIST)  */

#define N_BITS                    16
#define L_BITS                    8
#define L_RANGE                   256                  /*  must be 2^L_BITS  */
#define M_BITS                    8
#define M_RANGE                   256                  /*  must be 2^M_BITS  */
#define FRACTION_BITS             (L_BITS + M_BITS)
#define FRACTION_RANGE            65536         /*  must be 2^FRACTION_BITS  */
#define FILTER_LENGTH             (ZERO_CROSSINGS * L_RANGE)
#define FILTER_LIMIT              (FILTER_LENGTH - 1)

#define N_MASK                    0xFFFF0000
#define L_MASK                    0x0000FF00
#define M_MASK                    0x000000FF
#define FRACTION_MASK             0x0000FFFF

#define nValue(x)                 (((x) & N_MASK) >> FRACTION_BITS)
#define lValue(x)                 (((x) & L_MASK) >> M_BITS)
#define mValue(x)                 ((x) & M_MASK)
#define fractionValue(x)          ((x) & FRACTION_MASK)

#define OUTPUT_SRATE_LOW          22050.0
#define OUTPUT_SRATE_HIGH         44100.0

typedef struct _TRMParameters {
    double glotPitch;
    double glotVol;
    double aspVol;
    double fricVol;
    double fricPos;
    double fricCF;
    double fricBW;
    double radius[TOTAL_REGIONS];
    double velum;
} TRMParameters;

/*  VARIABLES FOR INPUT TABLES  */
typedef struct _INPUT {
    struct _INPUT *previous;
    struct _INPUT *next;

    TRMParameters parameters;
} INPUT;

extern int remote;
extern INPUT *remoteInput(INPUT *input);
extern int isRemote();
extern void remoteOutput(double signal);


typedef struct _TRMInputParameters {
    int    outputFileFormat;            /*  file format (0=AU, 1=AIFF, 2=WAVE)  */
    float  outputRate;                  /*  output sample rate (22.05, 44.1 KHz)  */
    float  controlRate;                 /*  1.0-1000.0 input tables/second (Hz)  */

    double volume;                      /*  master volume (0 - 60 dB)  */
    int    channels;                    /*  # of sound output channels (1, 2)  */
    double balance;                     /*  stereo balance (-1 to +1)  */

    int    waveform;                    /*  GS waveform type (0=PULSE, 1=SINE)  */
    double tp;                          /*  % glottal pulse rise time  */
    double tnMin;                       /*  % glottal pulse fall time minimum  */
    double tnMax;                       /*  % glottal pulse fall time maximum  */
    double breathiness;                 /*  % glottal source breathiness  */

    double length;                      /*  nominal tube length (10 - 20 cm)  */
    double temperature;                 /*  tube temperature (25 - 40 C)  */
    double lossFactor;                  /*  junction loss factor in (0 - 5 %)  */

    double apScale;                     /*  aperture scl. radius (3.05 - 12 cm)  */
    double mouthCoef;                   /*  mouth aperture coefficient  */
    double noseCoef;                    /*  nose aperture coefficient  */

    double noseRadius[TOTAL_NASAL_SECTIONS];  /*  fixed nose radii (0 - 3 cm)  */

    double throatCutoff;                /*  throat lp cutoff (50 - nyquist Hz)  */
    double throatVol;                   /*  throat volume (0 - 48 dB) */

    int    modulation;                  /*  pulse mod. of noise (0=OFF, 1=ON)  */
    double mixOffset;                   /*  noise crossmix offset (30 - 60 dB)  */
} TRMInputParameters;

typedef struct _TRMData {
    TRMInputParameters inputParameters;

    INPUT *inputHead;
    INPUT *inputTail;
} TRMData;


/*  VARIABLES FOR SAMPLE RATE CONVERSION  */
typedef struct _TRMSampleRateConverter {
    double sampleRateRatio;
    double h[FILTER_LENGTH], deltaH[FILTER_LENGTH];
    unsigned int timeRegisterIncrement, filterIncrement, phaseIncrement;
    unsigned int timeRegister;

    // Temporary sample storage values
    double maximumSampleValue;
    long int numberSamples;
    FILE *tempFilePtr;
} TRMSampleRateConverter;

/*  OROPHARYNX SCATTERING JUNCTION COEFFICIENTS (BETWEEN EACH REGION)  */
#define C1                        TRM_R1     /*  R1-R2 (S1-S2)  */
#define C2                        TRM_R2     /*  R2-R3 (S2-S3)  */
#define C3                        TRM_R3     /*  R3-R4 (S3-S4)  */
#define C4                        TRM_R4     /*  R4-R5 (S5-S6)  */
#define C5                        TRM_R5     /*  R5-R6 (S7-S8)  */
#define C6                        TRM_R6     /*  R6-R7 (S8-S9)  */
#define C7                        TRM_R7     /*  R7-R8 (S9-S10)  */
#define C8                        TRM_R8     /*  R8-AIR (S10-AIR)  */
#define TOTAL_COEFFICIENTS        TOTAL_REGIONS

/*  OROPHARYNX SECTIONS  */
#define S1                        0      /*  R1  */
#define S2                        1      /*  R2  */
#define S3                        2      /*  R3  */
#define S4                        3      /*  R4  */
#define S5                        4      /*  R4  */
#define S6                        5      /*  R5  */
#define S7                        6      /*  R5  */
#define S8                        7      /*  R6  */
#define S9                        8      /*  R7  */
#define S10                       9      /*  R8  */
#define TOTAL_SECTIONS            10

/*  NASAL TRACT COEFFICIENTS  */
#define NC1                       TRM_N1     /*  N1-N2  */
#define NC2                       TRM_N2     /*  N2-N3  */
#define NC3                       TRM_N3     /*  N3-N4  */
#define NC4                       TRM_N4     /*  N4-N5  */
#define NC5                       TRM_N5     /*  N5-N6  */
#define NC6                       TRM_N6     /*  N6-AIR  */
#define TOTAL_NASAL_COEFFICIENTS  TOTAL_NASAL_SECTIONS

/*  THREE-WAY JUNCTION ALPHA COEFFICIENTS  */
#define LEFT                      0
#define RIGHT                     1
#define UPPER                     2
#define TOTAL_ALPHA_COEFFICIENTS  3

/*  FRICATION INJECTION COEFFICIENTS  */
#define FC1                       0      /*  S3  */
#define FC2                       1      /*  S4  */
#define FC3                       2      /*  S5  */
#define FC4                       3      /*  S6  */
#define FC5                       4      /*  S7  */
#define FC6                       5      /*  S8  */
#define FC7                       6      /*  S9  */
#define FC8                       7      /*  S10  */
#define TOTAL_FRIC_COEFFICIENTS   8


/*  SCALING CONSTANT FOR INPUT TO VOCAL TRACT & THROAT (MATCHES DSP)  */
//#define VT_SCALE                  0.03125     /*  2^(-5)  */
// this is a temporary fix only, to try to match dsp synthesizer
#define VT_SCALE                  0.125     /*  2^(-3)  */

/*  BI-DIRECTIONAL TRANSMISSION LINE POINTERS  */
#define TOP                       0
#define BOTTOM                    1



typedef struct {
    //  DERIVED VALUES
    int    controlPeriod;
    int    sampleRate;
    double actualTubeLength;            /*  actual length in cm  */

    double dampingFactor;               /*  calculated damping factor  */
    double crossmixFactor;              /*  calculated crossmix factor  */

    double breathinessFactor;

    //  REFLECTION AND RADIATION FILTER MEMORY
    double a10, b11, a20, a21, b21;

    //  NASAL REFLECTION AND RADIATION FILTER MEMORY
    double na10, nb11, na20, na21, nb21;

    //  THROAT LOWPASS FILTER MEMORY, GAIN
    double tb1, ta0, throatGain;

    //  FRICATION BANDPASS FILTER MEMORY
    double bpAlpha, bpBeta, bpGamma;

    //  MEMORY FOR TUBE AND TUBE COEFFICIENTS
    double oropharynx[TOTAL_SECTIONS][2][2];
    double oropharynx_coeff[TOTAL_COEFFICIENTS];

    double nasal[TOTAL_NASAL_SECTIONS][2][2];
    double nasal_coeff[TOTAL_NASAL_COEFFICIENTS];

    double alpha[TOTAL_ALPHA_COEFFICIENTS];
    int current_ptr;
    int prev_ptr;

    //  MEMORY FOR FRICATION TAPS
    double fricationTap[TOTAL_FRIC_COEFFICIENTS];

    //  VARIABLES FOR INTERPOLATION
    struct {
        TRMParameters parameters;
        TRMParameters delta;
    } current;

    TRMSampleRateConverter sampleRateConverter;
    TRMRingBuffer *ringBuffer;
    TRMWavetable *wavetable;
} TRMTubeModel;

#ifdef __cplusplus
};
#endif

#endif
