// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2010 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/


#ifndef ICUBSIMULATION_SIMULATIONRUN_INC
#define ICUBSIMULATION_SIMULATIONRUN_INC

#include "SimulationBundle.h"

class SimulationRun {
public:
    bool run(SimulationBundle *bundle, int argc, char *argv[]);
};

#endif
