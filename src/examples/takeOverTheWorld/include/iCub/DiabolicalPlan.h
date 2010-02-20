// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
 
#ifndef __ICUB_DIABOLICALPLAN__
#define __ICUB_DIABOLICALPLAN__

#include <stdio.h>

class DiabolicalPlan {
private:
    int ct;
public:
    DiabolicalPlan() {
        ct = 0;
    }

    bool plan() {
        printf("Thinking...\n");
        ct++;
        return (ct>=3);
    }
};

#endif


