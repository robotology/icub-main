// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Vadim Tikhanoff, Paul Fitzpatrick
* email:   vadim.tikhanoff@iit.it, paulfitz@alum.mit.edu
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

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

    void fail(const char *msg) {
        failed = true;
        why = std::string("could not set ") + msg;
        result.setFail(why.c_str());
    }

    bool consume(WorldOpTriplet& x, const char *msg) {
        if (failed) return false;
        bool ok0 = (get(offset).isDouble()||get(offset).isInt());
        bool ok1 = (get(offset+1).isDouble()||get(offset+1).isInt());
        bool ok2 = (get(offset+2).isDouble()||get(offset+2).isInt());
        x.valid = false;
        if (!(ok0&&ok1&&ok2)) {
            fail(msg);
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
            fail(msg);
            return false;
        }
        x.index = get(offset).asInt();
        x.valid = true;
        offset++;
        return true;
    }

    bool consume(WorldOpName& x, const char *msg) {
        if (failed) return false;
        bool ok = get(offset).isString();
        x.valid = false;
        if (!ok) {
            fail(msg);
            return false;
        }
        x.name = get(offset).asString();
        x.valid = true;
        offset++;
        return true;
    }

    bool consume(WorldOpScalar& x, const char *msg) {
        if (failed) return false;
        bool ok = (get(offset).isDouble()||get(offset).isInt());
        x.valid = false;
        if (!ok) {
            fail(msg);
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
            fail(msg);
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
    case VOCAB4('s','m','o','d'):
        static_obj = true;
    case VOCAB4('m','o','d','e'):
        state.op.kind = "model";
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
    case VOCAB4('m','d','i','r'):
        state.op.parameter = WorldOpFlag(true);
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

bool doColor(ManagerState& state) {
        consumeObject(state);
        state.consume(state.op.color,"color");
        if (!state.failed) {
            state.manager.apply(state.op,state.result);
        }
        return !state.failed;
    }

bool doSet(ManagerState& state) {
    consumeObject(state);
    if (state.op.parameter.get()) {
        if (state.op.kind.get() == "mdir") {
            state.consume(state.op.modelName,"model path");
        } else {
            state.result.setFail("parameter not recognized");
            return false;
        }
    } else {
        state.consume(state.op.location,"location");
    }
    if (!state.failed) {
        state.manager.apply(state.op,state.result);
    }
    return !state.failed;
}

bool doMake(ManagerState& state) {
    consumeKind(state);
    std::string name = state.op.kind.name;
    if (!(name=="box"||name=="cyl"||name=="sph"||name=="model")) {
        state.result.setFail("cannot create object of requested type");
        return false;
    }
    if (name == "box") {
        state.consume(state.op.size,"size");
    }
    if (name == "cyl" || name == "sph") {
        state.consume(state.op.radius,"radius");
    }
    if (name == "cyl") {
        state.consume(state.op.length,"length");
    }
    if (name == "model") {
        state.consume(state.op.modelName,"model name");
        state.consume(state.op.modelTexture,"model texture");
    }
    state.consume(state.op.location,"location");
    if (name != "model") {
        state.consume(state.op.color,"color");
    }
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
    case WORLD_OP_COL:
        doColor(state);
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
            if (result.msg!=state.why) {
                reply.addString(result.msg.c_str());
            }
        }
        return true;
    } else {
        if (reply.size()==0) {
            if (result.location.isValid()) {
                reply.addDouble(result.location.get(0));
                reply.addDouble(result.location.get(1));
                reply.addDouble(result.location.get(2));
            } else if (result.path.isValid()) {
                reply.addString(result.path.get().c_str());
            } else {
                reply.addVocab(VOCAB2('o','k'));
            }
        }
    }
    return true;
}
