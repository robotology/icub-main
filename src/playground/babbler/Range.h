// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef BABBLER_RANGE
#define BABBLER_RANGE

class Range {
public:
    virtual double getMin(int i) = 0;
    virtual double getMax(int i) = 0;
};

#endif
