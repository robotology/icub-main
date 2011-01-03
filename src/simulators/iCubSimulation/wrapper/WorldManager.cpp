// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WorldManager.h"

#include <stdio.h>

class ManagerState {
public:
    const yarp::os::Bottle& command;
    WorldOp& op;
    WorldResult& result;
    int offset;

    ManagerState(const yarp::os::Bottle &command,
                 WorldOp& op,
                 WorldResult& result) : command(command), op(op), result(result) {
        offset = 2;
    }

};

bool doGet(ManagerState& state) {
    return false;
}

bool doSet(ManagerState& state) {
    return false;
}

bool doMake(ManagerState& state) {
    state.op.kind = WorldOpName(state.command.get(2).asString());
    int kind = state.command.get(2).asVocab();
    switch (kind) {
    case VOCAB3('b','o','x'):
        break;
    case VOCAB3('c','y','l'):
        break;
    case VOCAB4('s','b','o','x'):
        break;
    case VOCAB4('s','c','y','l'):
        break;
    }
    return false;
}

bool doGrab(ManagerState& state) {
    return false;
}

bool doRotate(ManagerState& state) {
    return false;
}

bool doDelete(ManagerState& state) {
    return false;
}

bool WorldManager::respond(const yarp::os::Bottle &command, 
                           yarp::os::Bottle &reply) {
    printf("Working on %s\n", command.toString().c_str());
    WorldOp op;
    WorldResult result;
    ManagerState state(command,op,result);

    op.cmd = (WORLD_OP)command.get(1).asVocab(); 
    switch (op.cmd) {
    case WORLD_OP_GET:
        doGet(state);
        break;
    case WORLD_OP_SET:
        doSet(state);
        break;
    case WORLD_OP_MK:
        doMake(state);
        break;
    case WORLD_OP_GRAB:
        doGrab(state);
        break;
    case WORLD_OP_ROT:
        doRotate(state);
        break;
    case WORLD_OP_DEL:
        doDelete(state);
        break;
    default:
        // unrecognized command
        return false;
    }

    if (!result.success) return false;

    return true;
}
