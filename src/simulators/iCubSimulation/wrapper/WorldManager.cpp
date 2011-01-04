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

    void show(WorldOpTriplet& x) {
        if (!x.valid) {
            printf("not set");
        } else {
            printf("%g %g %g", x.x[0], x.x[1], x.x[2]);
        }
        printf("\n");
    }

    void show(WorldOpScalar& x) {
        if (!x.valid) {
            printf("not set");
        } else {
            printf("%g", x.val);
        }
        printf("\n");
    }

    void show(WorldOpFlag& x, 
              const char *yes="True", 
              const char *no="False") {
        if (!x.valid) {
            printf("not set");
        } else {
            printf("%s", x.setting?yes:no);
        }
        printf("\n");
    }

    void show(WorldOpIndex& x) {
        if (!x.valid) {
            printf("not set");
        } else {
            printf("%d", x.index);
        }
        printf("\n");
    }

    void show(WorldOpName& x) {
        if (!x.valid) {
            printf("not set");
        } else {
            printf("%s", x.name.c_str());
        }
        printf("\n");
    }

    void debug() {
        printf("Command: %s\n", command.toString().c_str());
        printf("Parsed command:\n");
        printf("  tag: %s\n", yarp::os::Vocab::decode(op.cmd).c_str());
        printf("  kind: ");
        show(op.kind);
        printf("  name: ");
        show(op.name);
        printf("  dynamic: ");
        show(op.dynamic);
        printf("  location: ");
        show(op.location);
        printf("  size: ");
        show(op.size);
        printf("  color: ");
        show(op.color);
        printf("  rotation: ");
        show(op.rotation);
        printf("  radius: ");
        show(op.radius);
        printf("  active: ");
        show(op.active);
        printf("  index: ");
        show(op.index);
        printf("  hand: ");
        show(op.rightHanded,"right","left");
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
        break;
    case VOCAB4('s','c','y','l'):
        static_obj = true;
    case VOCAB3('c','y','l'):
        state.op.kind = "cyl";
        state.op.dynamic = WorldOpFlag(!static_obj);
        break;
    case VOCAB4('s','s','p','h'):
        static_obj = true;
    case VOCAB3('s','p','h'):
        state.op.kind = "sph";
        state.op.dynamic = WorldOpFlag(!static_obj);
        break;
    case VOCAB4('l','h','a','n'):
        state.op.kind = WorldOpName("hand");
        state.op.name = WorldOpName("icub_left_hand");
        state.op.index = WorldOpIndex(1);
        state.op.dynamic = WorldOpFlag(true);
        state.op.rightHanded = WorldOpFlag(false);
        break;
    case VOCAB4('r','h','a','n'):
        state.op.kind = WorldOpName("hand");
        state.op.name = WorldOpName("icub_right_hand");
        state.op.index = WorldOpIndex(2);
        state.op.dynamic = WorldOpFlag(true);
        state.op.rightHanded = WorldOpFlag(true);
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

bool doGet(ManagerState& state) {
    consumeKind(state);
    if (!state.op.index.valid) {
        state.consume(state.op.index,"index");
    }
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
    if (state.op.kind.name == "box") {
        state.consume(state.op.size,"size");
        state.consume(state.op.location,"location");
        state.consume(state.op.color,"color");
    } else if (state.op.kind.name == "cyl") {
        state.consume(state.op.radius,"radius");
        state.consume(state.op.location,"location");
        state.consume(state.op.color,"color");
    } else if (state.op.kind.name == "sph") {
        state.consume(state.op.radius,"radius");
        state.consume(state.op.location,"location");
        state.consume(state.op.color,"color");
    } else {
        return false;
    }
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doGrab(ManagerState& state) {
    consumeKind(state);
    if (state.failed) {
        // ok to fail
        state.failed = false;
        state.why = "";
        state.op.name = state.op.kind;
    } else {
        state.consume(state.op.index,"index");
    }
    consumeHand(state);
    state.consume(state.op.active,"active");
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doRotate(ManagerState& state) {
    consumeKind(state);
    state.consume(state.op.index,"index");
    state.consume(state.op.rotation,"rotation");
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doDelete(ManagerState& state) {
    return false;
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
        // unrecognized command
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
    }
    return true;
}
