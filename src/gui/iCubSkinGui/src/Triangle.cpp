// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/Triangle.h>

#define HUGE 1E+20
double Triangle::dXmin= HUGE;
double Triangle::dXmax=-HUGE;
double Triangle::dYmin= HUGE;
double Triangle::dYmax=-HUGE;

int Triangle::m_maxRange=0;
double* Triangle::Exponential=0;