// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_FAKEWORLDMANAGER_INC
#define ICUBSIMULATION_FAKEWORLDMANAGER_INC

#include "WorldManager.h"

class FakeWorldManager : public WorldManager {
    virtual void apply(const WorldOp& op, WorldResult& result) {}
};

#endif

