// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include "WorldManager.h"

using namespace yarp::os;

// WorldManager hasn't been cleaned up yet, need to include a stub.
bool WorldManager::respond(const Bottle &command, Bottle &reply) {
    return false;
}

void WorldManager::clear() {
}
