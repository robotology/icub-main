// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/Triangle.h>

#define VERY_FAR 1E+20
double Triangle::dXmin= VERY_FAR;
double Triangle::dXmax=-VERY_FAR;
double Triangle::dYmin= VERY_FAR;
double Triangle::dYmax=-VERY_FAR;

int Triangle::m_maxRange=0;
double* Triangle::Exponential=0;
