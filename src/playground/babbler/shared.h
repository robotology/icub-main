// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef BABBLER_SHARED
#define BABBLER_SHARED

#include <yarp/os/Property.h>

extern int watcher_main(yarp::os::Property& conf);
extern int aligner_main(yarp::os::Property& conf);
extern bool watcher_interrupt();
extern bool aligner_interrupt();


class SearchState {
public:
    // from searcher
    double currentTime;
    double globalLastAct;
    double period;
    
    // from observer
    double stableSummaryTime;
    double niceSummaryTime;
    
    SearchState() {
        currentTime = -10000;
        globalLastAct = -10000;
        stableSummaryTime = -10000;
        niceSummaryTime = -10000;
        period = 1;
    }
};

extern SearchState searchState;

#endif




