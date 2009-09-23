/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the YARP-aware managing interface for the EventDispatcher
 *
 */

//#include <cassert>
//#include <iostream>
#include <stdexcept>

#include <yarp/os/Vocab.h>

#include "iCub/DispatcherManager.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


DispatcherManager::DispatcherManager() {
}

bool DispatcherManager::respond(const Bottle& cmd, Bottle& reply) {
    bool success = false;

    try {
        switch(cmd.get(0).asVocab()) {
            case VOCAB4('h','e','l','p'): // print help information 
                reply.add(Value::makeVocab("help"));

                reply.addString("Event Manager configuration options");
                reply.addString("  help                  Displays this message");
                reply.addString("  add type [type2 ...]  Adds one or more event listeners");
                reply.addString("  remove [all|idx]      Removes event listener at an index or all");
                reply.addString("  set [all|idx]      Removes event listener at an index or all");
                reply.addString("  stats                 Prints information");
                reply.addString("  pause                 ");
                success = true;
                break;

            default:
                break;

        }
    } catch(const std::exception& e) {
        std::string msg = std::string("Error: ") + e.what();
        reply.addString(msg.c_str());
        success = true;
    }

    return success;
}




} // learningmachine
} // contrib
} // iCub

