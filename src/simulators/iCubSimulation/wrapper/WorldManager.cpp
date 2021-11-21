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

#include <cstdio>
#include <cstring>
#include <string>

#include <yarp/os/Value.h>

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

    bool more() {
        return command.size()>offset;
    }

    void fail(const char *msg) {
        failed = true;
        why = std::string("could not set ") + msg;
        result.setFail(why.c_str());
    }

    bool consume(WorldOpTriplet& x, const char *msg) {
        if (failed) return false;
        bool ok0 = (get(offset).isFloat64()||get(offset).isInt32());
        bool ok1 = (get(offset+1).isFloat64()||get(offset+1).isInt32());
        bool ok2 = (get(offset+2).isFloat64()||get(offset+2).isInt32());
        x.valid = false;
        if (!(ok0&&ok1&&ok2)) {
            fail(msg);
            return false;
        }
        x.x[0] = get(offset).asFloat64();
        x.x[1] = get(offset+1).asFloat64();
        x.x[2] = get(offset+2).asFloat64();
        x.valid = true;
        offset += 3;
        return true;
    }

    bool consume(WorldOpIndex& x, const char *msg) {
        if (failed) return false;
        bool ok = (get(offset).isInt32());
        x.valid = false;
        if (!ok) {
            fail(msg);
            return false;
        }
        x.index = get(offset).asInt32();
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
        bool ok = (get(offset).isFloat64()||get(offset).isInt32());
        x.valid = false;
        if (!ok) {
            fail(msg);
            return false;
        }
        x.val = get(offset).asFloat64();
        x.valid = true;
        offset++;
        return true;
    }

    bool consume(WorldOpFlag& x, const char *msg) {
        if (failed) return false;
        bool ok;
        int getBool = 0;
        if ( get(offset).isNull() && strcmp ( msg, "collide" ) == 0 )
        {
            ok = true;
            getBool = 1;
        }
        else if (get(offset).isString())
        {
            if ( get(offset).asString() == "true" || get(offset).asString() == "TRUE" )
            {
                ok  = true;
                getBool = 1;
            }
            else if ( get(offset).asString() == "false" || get(offset).asString() == "FALSE" )
            {
                ok  = true;
                getBool = 0;
            }
            else
               ok = false;
        }
        else
           ok  = (get(offset).isInt32());

        x.valid = false;
        if (!ok) {
            fail(msg);
            return false;
        }
        if (get(offset).isString() || get(offset).isNull() && strcmp ( msg, "collide" ) == 0)
        {
            x.setting = getBool?true:false;
        }
        else
            x.setting = get(offset).asInt32()?true:false;

        x.valid = true;
        offset++;
        return true;
    }

    void debug() {
        //yDebug("Command: %s\n", command.toString().c_str());
        //op.show();
    }
};

void consumeKind(ManagerState& state) {
    state.op.kind = WorldOpName(state.command.get(2).asString());
    int kind = state.command.get(2).asVocab32();
    state.offset++;
    bool static_obj = false;
    //bool mustCollide = state.command.get(2).asVocab32()
    switch (kind) {
    case yarp::os::createVocab32('s','b','o','x'):
        static_obj = true;
    case yarp::os::createVocab32('b','o','x'):
        state.op.kind = "box";
        state.op.dynamic = WorldOpFlag(!static_obj);
        state.needIndex = true;
        break;
    case yarp::os::createVocab32('s','c','y','l'):
        static_obj = true;
    case yarp::os::createVocab32('c','y','l'):
        state.op.kind = "cyl";
        state.op.dynamic = WorldOpFlag(!static_obj);
        state.needIndex = true;
        break;
    case yarp::os::createVocab32('s','s','p','h'):
        static_obj = true;
    case yarp::os::createVocab32('s','p','h'):
        state.op.kind = "sph";
        state.op.dynamic = WorldOpFlag(!static_obj);
        state.needIndex = true;
        break;
    case yarp::os::createVocab32('s','m','o','d'):
        static_obj = true;
    case yarp::os::createVocab32('m','o','d','e'):
        state.op.kind = "model";
        state.op.dynamic = WorldOpFlag(!static_obj);
        state.needIndex = true;
        break;
    case yarp::os::createVocab32('l','h','a','n'):
        state.op.kind = WorldOpName("hand");
        //state.op.name = WorldOpName("icub_left_hand");
        state.op.index = WorldOpIndex(1);
        state.op.dynamic = WorldOpFlag(true);
        state.op.rightHanded = WorldOpFlag(false);
        break;
    case yarp::os::createVocab32('r','h','a','n'):
        state.op.kind = WorldOpName("hand");
        //state.op.name = WorldOpName("icub_right_hand");
        state.op.index = WorldOpIndex(2);
        state.op.dynamic = WorldOpFlag(true);
        state.op.rightHanded = WorldOpFlag(true);
        break;
    case yarp::os::createVocab32('m','d','i','r'):
        state.op.parameter = WorldOpFlag(true);
        break;
    case yarp::os::createVocab32('t','a','b','l'):
    case yarp::os::createVocab32('c','u','b','e'):
    case yarp::os::createVocab32('b','a','l','l'):
    case yarp::os::createVocab32('s','c','r','e'):
    case yarp::os::createVocab32('a','l','l'):
        break;
    default:
        state.failed = true;
        state.why = "unrecognized object type";
        break;
    }
}

void consumeHand(ManagerState& state) {
    std::string grabber = state.get(state.offset).asString();
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
        // if color supplied, set, otherwise, get.
        if (state.more()) {
            state.consume(state.op.color,"color");
        }
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

    state.consume(state.op.collide,"collide");

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
    if (state.more()) {
        state.consume(state.op.rotation,"rotation");
    }
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

bool doNumber(ManagerState& state) {
    consumeKind(state);
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

    op.cmd = (WORLD_OP)command.get(1).asVocab32(); 
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
    case WORLD_OP_NUM:
        doNumber(state);
        break;
    default:
        state.failed = true;
        state.why = "unrecognized command";
        break;
    }

    state.debug();
    if (!result.success) {
        if (reply.size()==0) {
            reply.addVocab32(yarp::os::createVocab32('f','a','i','l'));
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
                reply.addFloat64(result.location.get(0));
                reply.addFloat64(result.location.get(1));
                reply.addFloat64(result.location.get(2));
            } else if (result.rotation.isValid()) {
                reply.addFloat64(result.rotation.get(0));
                reply.addFloat64(result.rotation.get(1));
                reply.addFloat64(result.rotation.get(2));
            } else if (result.color.isValid()) {
                reply.addFloat64(result.color.get(0));
                reply.addFloat64(result.color.get(1));
                reply.addFloat64(result.color.get(2));
            } else if (result.count.isValid()) {
                reply.addInt32(result.count.get());
            } else if (result.path.isValid()) {
                reply.addString(result.path.get().c_str());
            } else {
                reply.addVocab32(yarp::os::createVocab32('o','k'));
            }
        }
    }
    return true;
}
