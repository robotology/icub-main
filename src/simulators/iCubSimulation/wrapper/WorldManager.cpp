// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WorldManager.h"

#include <stdio.h>

class ManagerState {
public:
    bool failed;
    std::string why;
    const yarp::os::Bottle& command;
    WorldOp& op;
    WorldResult& result;
    int offset;
    bool needIndex;
    WorldManager& manager;

    ManagerState(const yarp::os::Bottle &command,
                 WorldOp& op,
                 WorldResult& result,
                 WorldManager& manager) : 
        command(command), 
        op(op), 
        result(result),
        manager(manager) 
    {
        offset = 2;
        failed = false;
        needIndex = false;
    }

    const yarp::os::Value& get(int offset) {
        return command.get(offset);
    }

    bool consume(WorldOpTriplet& x, const char *msg) {
        if (failed) return false;
        bool ok0 = (get(offset).isDouble()||get(offset).isInt());
        bool ok1 = (get(offset+1).isDouble()||get(offset+1).isInt());
        bool ok2 = (get(offset+2).isDouble()||get(offset+2).isInt());
        x.valid = false;
        if (!(ok0&&ok1&&ok2)) {
            failed = true;
            why = std::string("could not parse ") + msg + " from " +
                get(offset).toString().c_str() + " " +
                get(offset+1).toString().c_str() + " " +
                get(offset+2).toString().c_str();
            return false;
        }
        x.x[0] = get(offset).asDouble();
        x.x[1] = get(offset+1).asDouble();
        x.x[2] = get(offset+2).asDouble();
        x.valid = true;
        offset += 3;
        return true;
    }

    bool consume(WorldOpIndex& x, const char *msg) {
        if (failed) return false;
        bool ok = (get(offset).isInt());
        x.valid = false;
        if (!ok) {
            failed = true;
            return false;
        }
        x.index = get(offset).asInt();
        x.valid = true;
        offset++;
        return true;
    }

    bool consume(WorldOpScalar& x, const char *msg) {
        if (failed) return false;
        bool ok = (get(offset).isDouble()||get(offset).isInt());
        x.valid = false;
        if (!ok) {
            failed = true;
            return false;
        }
        x.val = get(offset).asDouble();
        x.valid = true;
        offset++;
        return true;
    }

    bool consume(WorldOpFlag& x, const char *msg) {
        if (failed) return false;
        bool ok = (get(offset).isInt());
        x.valid = false;
        if (!ok) {
            failed = true;
            return false;
        }
        x.setting = get(offset).asInt()?true:false;
        x.valid = true;
        offset++;
        return true;
    }

    void debug() {
        //printf("Command: %s\n", command.toString().c_str());
        //op.show();
    }
};

void consumeKind(ManagerState& state) {
    state.op.kind = WorldOpName(state.command.get(2).asString());
    int kind = state.command.get(2).asVocab();
    state.offset++;
    bool static_obj = false;
    switch (kind) {
    case VOCAB4('s','b','o','x'):
        static_obj = true;
    case VOCAB3('b','o','x'):
        state.op.kind = "box";
        state.op.dynamic = WorldOpFlag(!static_obj);
        state.needIndex = true;
        break;
    case VOCAB4('s','c','y','l'):
        static_obj = true;
    case VOCAB3('c','y','l'):
        state.op.kind = "cyl";
        state.op.dynamic = WorldOpFlag(!static_obj);
        state.needIndex = true;
        break;
    case VOCAB4('s','s','p','h'):
        static_obj = true;
    case VOCAB3('s','p','h'):
        state.op.kind = "sph";
        state.op.dynamic = WorldOpFlag(!static_obj);
        state.needIndex = true;
        break;
    case VOCAB4('l','h','a','n'):
        state.op.kind = WorldOpName("hand");
        //state.op.name = WorldOpName("icub_left_hand");
        state.op.index = WorldOpIndex(1);
        state.op.dynamic = WorldOpFlag(true);
        state.op.rightHanded = WorldOpFlag(false);
        break;
    case VOCAB4('r','h','a','n'):
        state.op.kind = WorldOpName("hand");
        //state.op.name = WorldOpName("icub_right_hand");
        state.op.index = WorldOpIndex(2);
        state.op.dynamic = WorldOpFlag(true);
        state.op.rightHanded = WorldOpFlag(true);
        break;
    case VOCAB4('t','a','b','l'):
    case VOCAB4('c','u','b','e'):
    case VOCAB4('b','a','l','l'):
    case VOCAB3('a','l','l'):
        break;
    default:
        state.failed = true;
        state.why = "unrecognized object type";
        break;
    }
}

void consumeHand(ManagerState& state) {
    yarp::os::ConstString grabber = state.get(state.offset).asString();
    state.offset++;
    if (grabber=="left") {
        state.op.rightHanded = WorldOpFlag(false);
    } else if (grabber=="right") {
        state.op.rightHanded = WorldOpFlag(true);
    } else {
        state.failed = "true";
        state.why = "hand not recognized";
    }
}

void consumeObject(ManagerState& state) {
    consumeKind(state);
    if (state.needIndex) {
        if (!state.op.index.valid) {
            state.consume(state.op.index,"index");
        }
    }
}

bool doGet(ManagerState& state) {
    consumeObject(state);
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doSet(ManagerState& state) {
    consumeKind(state);
    state.consume(state.op.index,"index");
    state.consume(state.op.location,"location");
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doMake(ManagerState& state) {
    consumeKind(state);
    std::string name = state.op.kind.name;
    if (!(name=="box"||name=="cyl"||name=="sph")) {
        state.result.setFail("cannot create object of requested type");
        return false;
    }
    if (name == "box") {
        state.consume(state.op.size,"size");
    }
    if (name == "cyl" || name == "sph") {
        state.consume(state.op.radius,"radius");
    }
    state.consume(state.op.location,"location");
    state.consume(state.op.color,"color");
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doGrab(ManagerState& state) {
    consumeObject(state);
    consumeHand(state);
    state.consume(state.op.active,"active");
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doRotate(ManagerState& state) {
    consumeObject(state);
    state.consume(state.op.rotation,"rotation");
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doDelete(ManagerState& state) {
    consumeObject(state);
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool WorldManager::respond(const yarp::os::Bottle& command, 
                           yarp::os::Bottle& reply) {
    WorldOp op;
    WorldResult result;
    ManagerState state(command,op,result,*this);
    reply.clear();

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
        state.failed = true;
        state.why = "unrecognized command";
        break;
    }

    state.debug();
    if (!result.success) {
        if (reply.size()==0) {
            reply.addVocab(VOCAB4('f','a','i','l'));
        }
        if (state.failed) {
            reply.addString(state.why.c_str());
        }
        if (result.msg!="") {
            reply.addString(result.msg.c_str());
        }
        return true;
    } else {
        if (reply.size()==0) {
            if (result.location.isValid()) {
                reply.addDouble(result.location.get(0));
                reply.addDouble(result.location.get(1));
                reply.addDouble(result.location.get(2));
            } else {
                reply.addVocab(VOCAB2('o','k'));
            }
        }
    }
    return true;
}
