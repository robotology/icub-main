// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/Triangle.h>

#define HUGE 1E+20
double Triangle::xMin= HUGE;
double Triangle::xMax=-HUGE;
double Triangle::yMin= HUGE;
double Triangle::yMax=-HUGE;

int Triangle::m_maxRange=0;
double* Triangle::Exponential=0;