// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2011 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUBSIMULATION_FAKELOGICALJOINTS_INC
#define ICUBSIMULATION_FAKELOGICALJOINTS_INC

#include "LogicalJoints.h"
#include "FakeLogicalJoint.h"

#include <map>

class FakeLogicalJoints : public LogicalJoints {
private:
    std::map<int,FakeLogicalJoint> joints;
public:
    virtual LogicalJoint& control(int part, int axis) {
        int idx = part*100+axis;
        std::map<int,FakeLogicalJoint>::iterator it = joints.find(idx);
        if (it==joints.end()) {
            return joints[idx] = FakeLogicalJoint();
        }
        return it->second;
    }
};

#endif
