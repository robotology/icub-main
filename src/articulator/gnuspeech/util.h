// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1991, 1992, 1993, 1994, 1995, 1996, 2001 and 2002 David
 *               R. Hill, Leonard Manzara and Craig Schock
 * CopyPolicy: GNUSPEECH released under the terms of the GNU GPL
 *
 */

#ifndef __UTIL_H
#define __UTIL_H

#define BETA                      5.658        /*  kaiser window parameters  */
#define IzeroEPSILON              1E-21


#ifdef __cplusplus
extern "C" {
#endif

double speedOfSound(double temperature);
double amplitude(double decibelLevel);
double frequency(double pitch);
double Izero(double x);
double noise(void);
double noiseFilter(double input);


#ifdef __cplusplus
};
#endif

#endif
