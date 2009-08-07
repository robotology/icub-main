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
#include "tube.h" // for TWO_PI

/*  CONSTANTS FOR THE FIR FILTER  */
#define LIMIT                     200
#define BETA_OUT_OF_RANGE         1
#define GAMMA_OUT_OF_RANGE        2
#define GAMMA_TOO_SMALL           3


static int maximallyFlat(double beta, double gamma, int *np, double *coefficient);
static void trim(double cutoff, int *numberCoefficients, double *coefficient);
static int increment(int pointer, int modulus);
static int decrement(int pointer, int modulus);
static void rationalApproximation(double number, int *order, int *numerator, int *denominator);

// Allocates memory and initializes the coefficients for the FIR filter used in the oversampling oscillator.

TRMFIRFilter *TRMFIRFilterCreate(double beta, double gamma, double cutoff)
{
    TRMFIRFilter *newFilter;

    int i, pointer, increment, numberCoefficients;
    double coefficient[LIMIT+1];

    newFilter = (TRMFIRFilter *)malloc(sizeof(TRMFIRFilter));
    if (newFilter == NULL) {
        fprintf(stderr, "Couldn't malloc() FIRFilter.\n");
        return NULL;
    }

    /*  DETERMINE IDEAL LOW PASS FILTER COEFFICIENTS  */
    maximallyFlat(beta, gamma, &numberCoefficients, coefficient);

    /*  TRIM LOW-VALUE COEFFICIENTS  */
    trim(cutoff, &numberCoefficients, coefficient);

    /*  DETERMINE THE NUMBER OF TAPS IN THE FILTER  */
    newFilter->numberTaps = (numberCoefficients * 2) - 1;

    /*  ALLOCATE MEMORY FOR DATA AND COEFFICIENTS  */
    newFilter->FIRData = (double *)calloc(newFilter->numberTaps, sizeof(double));
    if (newFilter->FIRData == NULL) {
        fprintf(stderr, "calloc() of FIRData failed.\n");
        free(newFilter);
        return NULL;
    }

    newFilter->FIRCoef = (double *)calloc(newFilter->numberTaps, sizeof(double));
    if (newFilter->FIRCoef == NULL) {
        fprintf(stderr, "calloc() of FIRCoef failed.\n");
        free(newFilter->FIRData);
        free(newFilter);
        return NULL;
    }

    /*  INITIALIZE THE COEFFICIENTS  */
    increment = -1;
    pointer = numberCoefficients;
    for (i = 0; i < newFilter->numberTaps; i++) {
	newFilter->FIRCoef[i] = coefficient[pointer];
	pointer += increment;
	if (pointer <= 0) {
	    pointer = 2;
	    increment = 1;
	}
    }

    /*  SET POINTER TO FIRST ELEMENT  */
    newFilter->FIRPtr = 0;

#if DEBUG
    /*  PRINT OUT  */
    printf("\n");
    for (i = 0; i < newFilter->numberTaps; i++)
	printf("FIRCoef[%-d] = %11.8f\n", i, newFilter->FIRCoef[i]);
#endif

    return newFilter;
}

void TRMFIRFilterFree(TRMFIRFilter *filter)
{
    if (filter == NULL)
        return;

    if (filter->FIRData != NULL) {
        free(filter->FIRData);
        filter->FIRData = NULL;
    }

    if (filter->FIRCoef != NULL) {
        free(filter->FIRCoef);
        filter->FIRCoef = NULL;
    }

    free(filter);
}


/******************************************************************************
*
*	function:	maximallyFlat
*
*	purpose:	Calculates coefficients for a linear phase lowpass FIR
*                       filter, with beta being the center frequency of the
*                       transition band (as a fraction of the sampling
*                       frequency), and gamme the width of the transition
*                       band.
*
*       arguments:      beta, gamma, np, coefficient
*
*	internal
*	functions:	rationalApproximation
*
*	library
*	functions:	cos, pow
*
******************************************************************************/

int maximallyFlat(double beta, double gamma, int *np, double *coefficient)
{
    double a[LIMIT+1], c[LIMIT+1], betaMinimum, ac;
    int nt, numerator, n, ll, i;


    /*  INITIALIZE NUMBER OF POINTS  */
    (*np) = 0;

    /*  CUT-OFF FREQUENCY MUST BE BETWEEN 0 HZ AND NYQUIST  */
    if ((beta <= 0.0) || (beta >= 0.5))
	return BETA_OUT_OF_RANGE;

    /*  TRANSITION BAND MUST FIT WITH THE STOP BAND  */
    betaMinimum = ((2.0 * beta) < (1.0 - 2.0 * beta)) ? (2.0 * beta) :
	(1.0 - 2.0 * beta);
    if ((gamma <= 0.0) || (gamma >= betaMinimum))
	return GAMMA_OUT_OF_RANGE;

    /*  MAKE SURE TRANSITION BAND NOT TOO SMALL  */
    nt = (int)(1.0 / (4.0 * gamma * gamma));
    if (nt > 160)
	return GAMMA_TOO_SMALL;

    /*  CALCULATE THE RATIONAL APPROXIMATION TO THE CUT-OFF POINT  */
    ac = (1.0 + cos(TWO_PI * beta)) / 2.0;
    rationalApproximation(ac, &nt, &numerator, np);

    /*  CALCULATE FILTER ORDER  */
    n = (2 * (*np)) - 1;
    if (numerator == 0)
	numerator = 1;


    /*  COMPUTE MAGNITUDE AT NP POINTS  */
    c[1] = a[1] = 1.0;
    ll = nt - numerator;

    for (i = 2; i <= (*np); i++) {
	int j;
	double x, sum = 1.0, y;
	c[i] = cos(TWO_PI * ((double)(i-1)/(double)n));
	x = (1.0 - c[i]) / 2.0;
	y = x;

	if (numerator == nt)
	    continue;

	for (j = 1; j <= ll; j++) {
	    double z = y;
	    if (numerator != 1) {
		int jj;
		for (jj = 1; jj <= (numerator - 1); jj++)
		    z *= 1.0 + ((double)j / (double)jj);
	    }
	    y *= x;
	    sum += z;
	}
	a[i] = sum * pow((1.0 - x), numerator);
    }


    /*  CALCULATE WEIGHTING COEFFICIENTS BY AN N-POINT IDFT  */
    for (i = 1; i <= (*np); i++) {
	int j;
	coefficient[i] = a[1] / 2.0;
	for (j = 2; j <= (*np); j++) {
	    int m = ((i - 1) * (j - 1)) % n;
	    if (m > nt)
		m = n - m;
	    coefficient[i] += c[m+1] * a[j];
	}
	coefficient[i] *= 2.0 / (double)n;
    }

    return 0;
}



/******************************************************************************
*
*	function:	trim
*
*	purpose:	Trims the higher order coefficients of the FIR filter
*                       which fall below the cutoff value.
*
*       arguments:      cutoff, numberCoefficients, coefficient
*
*	internal
*	functions:	none
*
*	library
*	functions:	fabs
*
******************************************************************************/

void trim(double cutoff, int *numberCoefficients, double *coefficient)
{
    int i;

    for (i = (*numberCoefficients); i > 0; i--) {
	if (fabs(coefficient[i]) >= fabs(cutoff)) {
	    (*numberCoefficients) = i;
	    return;
	}
    }
}



/******************************************************************************
*
*	function:	FIRFilter
*
*	purpose:	Is the linear phase, lowpass FIR filter.
*
*       arguments:      input, needOutput
*
*	internal
*	functions:	increment, decrement
*
*	library
*	functions:	none
*
******************************************************************************/

double FIRFilter(TRMFIRFilter *filter, double input, int needOutput)
{
    if (needOutput) {
	int i;
	double output = 0.0;

	/*  PUT INPUT SAMPLE INTO DATA BUFFER  */
	filter->FIRData[filter->FIRPtr] = input;

	/*  SUM THE OUTPUT FROM ALL FILTER TAPS  */
	for (i = 0; i < filter->numberTaps; i++) {
	    output += filter->FIRData[filter->FIRPtr] * filter->FIRCoef[i];
	    filter->FIRPtr = increment(filter->FIRPtr, filter->numberTaps);
	}

	/*  DECREMENT THE DATA POINTER READY FOR NEXT CALL  */
	filter->FIRPtr = decrement(filter->FIRPtr, filter->numberTaps);

	/*  RETURN THE OUTPUT VALUE  */
        //printf("FIRFilter(%g, %d) = %g\n", input, needOutput, output);
	return output;
    } else {
	/*  PUT INPUT SAMPLE INTO DATA BUFFER  */
	filter->FIRData[filter->FIRPtr] = input;

	/*  ADJUST THE DATA POINTER, READY FOR NEXT CALL  */
	filter->FIRPtr = decrement(filter->FIRPtr, filter->numberTaps);

        //printf("FIRFilter(%g, %d) = %g\n", input, needOutput, 0.0);
	return 0.0;
    }
}



/******************************************************************************
*
*	function:	increment
*
*	purpose:	Increments the pointer to the circular FIR filter
*                       buffer, keeping it in the range 0 -> modulus-1.
*
*       arguments:      pointer, modulus
*
*	internal
*	functions:	none
*
*	library
*	functions:	none
*
******************************************************************************/

int increment(int pointer, int modulus)
{
    if (++pointer >= modulus)
	return 0;

    return pointer;
}


/******************************************************************************
*
*	function:	decrement
*
*	purpose:	Decrements the pointer to the circular FIR filter
*                       buffer, keeping it in the range 0 -> modulus-1.
*
*       arguments:      pointer, modulus
*
*	internal
*	functions:	none
*
*	library
*	functions:	none
*
******************************************************************************/

int decrement(int pointer, int modulus)
{
    if (--pointer < 0)
	return modulus - 1;

    return pointer;
}



/******************************************************************************
*
*	function:	rationalApproximation
*
*	purpose:	Calculates the best rational approximation to 'number',
*                       given the maximum 'order'.
*
*       arguments:      number, order, numerator, denominator
*
*	internal
*	functions:	none
*
*	library
*	functions:	fabs
*
******************************************************************************/

void rationalApproximation(double number, int *order, int *numerator, int *denominator)
{
    double fractionalPart, minimumError = 1.0;
    int i, orderMaximum, modulus = 0;


    /*  RETURN IMMEDIATELY IF THE ORDER IS LESS THAN ONE  */
    if (*order <= 0) {
	*numerator = 0;
	*denominator = 0;
	*order = -1;
	return;
    }

    /*  FIND THE ABSOLUTE VALUE OF THE FRACTIONAL PART OF THE NUMBER  */
    fractionalPart = fabs(number - (int)number);

    /*  DETERMINE THE MAXIMUM VALUE OF THE DENOMINATOR  */
    orderMaximum = 2 * (*order);
    orderMaximum = (orderMaximum > LIMIT) ? LIMIT : orderMaximum;

    /*  FIND THE BEST DENOMINATOR VALUE  */
    for (i = (*order); i <= orderMaximum; i++) {
	double ps = i * fractionalPart;
	int ip = (int)(ps + 0.5);
	double error = fabs( (ps - (double)ip)/(double)i );
	if (error < minimumError) {
	    minimumError = error;
	    modulus = ip;
	    *denominator = i;
	}
    }

    /*  DETERMINE THE NUMERATOR VALUE, MAKING IT NEGATIVE IF NECESSARY  */
    *numerator = (int)fabs(number) * (*denominator) + modulus;
    if (number < 0)
	*numerator *= (-1);

    /*  SET THE ORDER  */
    *order = *denominator - 1;

    /*  RESET THE NUMERATOR AND DENOMINATOR IF THEY ARE EQUAL  */
    if (*numerator == *denominator) {
	*denominator = orderMaximum;
	*order = *numerator = *denominator - 1;
    }
}
