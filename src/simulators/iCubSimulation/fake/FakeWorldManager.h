// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2011 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUBSIMULATION_FAKEWORLDMANAGER_INC
#define ICUBSIMULATION_FAKEWORLDMANAGER_INC

#include "WorldManager.h"

class FakeWorldManager : public WorldManager {
    virtual void apply(const WorldOp& op, WorldResult& result);
};

#endif

