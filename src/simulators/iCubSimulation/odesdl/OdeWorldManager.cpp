// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Paul Fitzpatrick, Vadim Tikhanoff
* email:   paulfitz@alum.mit.edu, vadim.tikhanoff@iit.it
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

#include "OdeWorldManager.h"

#include "OdeInit.h"
#include "iCub_Sim.h"
#include <yarp/os/LogStream.h>
#include <map>
#include <mutex>

#define DENSITY (1.0)		// density of all objects

using namespace yarp::os;
using namespace std;

class OdeLink {
    std::mutex ODE_access;
public:
    const WorldOp& op;
    WorldResult& result;
    int setBody;
    dBodyID bid;
    dGeomID gid;
    WorldObjectList *store;
    WorldObject *object;

    OdeLink(const WorldOp& op, WorldResult& result) : op(op), result(result) {
        setBody = -1;
        object = NULL;
        store = NULL;
    }

    bool checkObject(bool forCreate = false);

    void doGet();
    void doSet();
    void doMake();
    void doGrab();
    void doRotate();
    void doDelete();
    void doColor();
    void doNumber();
    void apply();
};

bool OdeLink::checkObject(bool justNeedKind) {
    //op.show();
    OdeInit& odeinit = OdeInit::get();

    bid = NULL;
    gid = NULL;
    store = NULL;
    object = NULL;

    std::string kind = op.kind.get();

    if (kind=="cube") {
        bid = odeinit._wrld->Box;
    } else if (kind=="ball") {
        bid = odeinit._wrld->ballBody;
    } else if (kind=="screen") {
        gid = odeinit._iCub->screenGeom;
        return true;
    } else if (kind=="hand") {
        if (op.rightHanded.get()) {
             if (
                odeinit._iCub->actRHand=="on") {
                bid = odeinit._iCub->body[11];
                yDebug("Full left hand\n");
            } else {
                bid = odeinit._iCub->r_hand;
                yDebug("slim left hand\n");
            }
        } else {
            if (
                odeinit._iCub->actLHand=="on") {
                bid = odeinit._iCub->body[10];
                yDebug("Full left hand\n");
            } else {
                bid = odeinit._iCub->l_hand;
                yDebug("slim left hand\n");
            }
        }
    }

    if (bid!=NULL) {
        return true;
    }

    if (!justNeedKind) {
        if ((!bid) && !op.index.isValid()) {
            result.setFail("object without index is not known");
            return false;
        }
    }

    if (!op.dynamic.isValid()) {
        result.setFail("do not know if object is dynamic or static");
        return false;
    }

    bool dynamic = op.dynamic.get();

    if (kind=="box") {
        store = dynamic?(&odeinit._wrld->box_dynamic):(&odeinit._wrld->box_static);
    } else if (kind=="cyl") {
        store = dynamic?(&odeinit._wrld->cylinder_dynamic):(&odeinit._wrld->cylinder_static);
    } else if (kind=="model") {
        store = dynamic?(&odeinit._wrld->model_dynamic):(&odeinit._wrld->model_static);
    } else if (kind=="sph") {
        store = dynamic?(&odeinit._wrld->sphere_dynamic):(&odeinit._wrld->sphere_static);
    }

    if (store==NULL) {
        result.setFail("unknown object");
        return false;
    }

    if (!justNeedKind) {
        int index = op.index.get()-1;
        if (!store->inRange(index)) {
            result.setFail("out of range");
            return false;
        }

        WorldObject& obj = store->get(index);
        if (op.dynamic.get()) {
            bid = obj.getBody();
        } else {
            gid = obj.getGeometry();
        }
        object = &obj;
    }

    return true;
}


void OdeLink::doGet() {
    OdeInit& odeinit = OdeInit::get();

    if (op.parameter.get()) {
        if (op.kind.get()=="mdir") {
            result.path = WorldOpName(odeinit._wrld->model_DIR.c_str());
            result.setOk();
            return;
        } else {
            result.setFail("parameter not recognized");
            return;
        }
    }

    if (!checkObject()) return;
    if (bid!=NULL) {
        lock_guard<mutex> lck(odeinit.mtx);
        const dReal *coords = dBodyGetPosition(bid);
        result.location = WorldOpTriplet(coords[0],coords[1],coords[2]);
        result.setOk();
        return;
    }
    if (gid!=NULL) {
        lock_guard<mutex> lck(odeinit.mtx);
        const dReal *coords = dGeomGetPosition(gid);
        result.location = WorldOpTriplet(coords[0],coords[1],coords[2]);
        result.setOk();
        return;
    }
    result.setFail("no object found");
}

void OdeLink::doSet() {
    OdeInit& odeinit = OdeInit::get();

    if (op.parameter.get()) {
        if (op.kind.get()=="mdir") {
            odeinit._wrld->model_DIR = op.modelName.get().c_str();
            result.setOk();
            return;
        } else {
            result.setFail("parameter not recognized");
            return;
        }
    }

    if (!checkObject()) return;

    if (!op.location.isValid()) {
        result.setFail("no location set");
        return;
    }
    if (bid!=NULL) {
        lock_guard<mutex> lck(odeinit.mtx);
        dBodySetPosition(bid,
                         op.location.get(0),
                         op.location.get(1),
                         op.location.get(2));
        dBodySetLinearVel(bid,0.0,0.0,0.0);
        dBodySetAngularVel(bid,0.0,0.0,0.0);
        result.setOk();
        return;
    }
    if (gid!=NULL) {
        lock_guard<mutex> lck(odeinit.mtx);
        dGeomSetPosition(gid,
                         op.location.get(0),
                         op.location.get(1),
                         op.location.get(2));
        result.setOk();
        return;
    }
    result.setFail("no object found");
}

void OdeLink::doMake() {
    if (!checkObject(true)) return;
    if (store==NULL) {
        result.setFail("cannot create that kind of object");
        return;
    }
    OdeInit& odeinit = OdeInit::get();
    lock_guard<mutex> lck(odeinit.mtx);
    if (store->create(op,result)) {
        result.setOk();
    }
}

void OdeLink::doGrab() {
    OdeInit& odeinit = OdeInit::get();

    if (!checkObject()) return;

    if (!op.rightHanded.isValid()) {
        result.setFail("hand not set");
        return;
    }
    bool right = op.rightHanded.get();
    bool left = !right;

    if (left && odeinit._iCub->actLHand!="off") {
        result.setFail("left hand must be disabled, cannot use grab with fingers");
        return;
    }
    if (right && odeinit._iCub->actRHand!="off") {
        result.setFail("right hand must be disabled, cannot use grab with fingers");
        return;
    }

    if (!op.active.isValid()) {
        result.setFail("activity flag not set");
        return;
    }
    bool active = op.active.get();

    if (bid==NULL) {
        result.setFail("grab requires a dynamic object");
        return;
    }

    lock_guard<mutex> lck(odeinit.mtx);
    if (active) {
        if (bid!=NULL) {
            if (left) {
                odeinit._iCub->grab = dJointCreateFixed(odeinit.world,0);
                dJointAttach (odeinit._iCub->grab,odeinit._iCub->l_hand,bid);
                dJointSetFixed(odeinit._iCub->grab);
            }
            if (right) {
                odeinit._iCub->grab1 = dJointCreateFixed(odeinit.world,0);
                dJointAttach (odeinit._iCub->grab1,odeinit._iCub->r_hand,bid);
                dJointSetFixed(odeinit._iCub->grab1);
            }
        }
    } else {
        if (left) {
            dJointDestroy(odeinit._iCub->grab);
        }
        if (right) {
            dJointDestroy(odeinit._iCub->grab1);
        }
    }
    result.setOk();
}


void OdeLink::doRotate() {
    OdeInit& odeinit = OdeInit::get();

    if (!checkObject()) return;
    if (object==NULL) {
        result.setFail("no geometry found");
        return;
    }

    // We want to get the object rotation
    if (!op.rotation.isValid()) {
        if (bid!=NULL) {
            lock_guard<mutex> lck(odeinit.mtx);
            const dReal *R = dBodyGetRotation(bid);
            result.rotation = WorldOpTriplet(atan2(R[9], R[10])*180/M_PI, asin(-R[8])*180/M_PI, atan2(R[4], R[0])*180/M_PI);
            result.setOk();
            return;
        }
        if (gid!=NULL) {
            lock_guard<mutex> lck(odeinit.mtx);
            const dReal *R = dGeomGetRotation(gid);
            result.rotation = WorldOpTriplet(atan2(R[9], R[10])*180/M_PI, asin(-R[8])*180/M_PI, atan2(R[4], R[0])*180/M_PI);
            result.setOk();
            return;
        }
        result.setFail("no object found");
    }
    if (object==NULL) {
        result.setFail("no geometry found");
        return;
    } else {
        // We want to set the object rotation

        dMatrix3 Rtx,Rty,Rtz, Rtmp1,Rtmp2;

        double rotx = (op.rotation.get(0) * M_PI) / 180;
        double roty = (op.rotation.get(1) * M_PI) / 180;
        double rotz = (op.rotation.get(2) * M_PI) / 180;

        dRFromAxisAndAngle(Rtx,1,0,0,rotx);
        dRFromAxisAndAngle(Rty,0,1,0,roty);
        dRFromAxisAndAngle(Rtz,0,0,1,rotz);

        dMultiply0 (Rtmp1,Rty,Rtz,3,3,3);
        dMultiply0 (Rtmp2,Rtx,Rtmp1,3,3,3);
        lock_guard<mutex> lck(odeinit.mtx);
        dGeomSetRotation(object->getGeometry(),Rtmp2);
        result.setOk();
    }
}

void OdeLink::doColor() {
    OdeInit& odeinit = OdeInit::get();
    if (!checkObject()) return;
    if (op.color.isValid()) {
        if (store->colors!=NULL) {
            store->colors[op.index.index-1][0] = op.color.get(0);
            store->colors[op.index.index-1][1] = op.color.get(1);
            store->colors[op.index.index-1][2] = op.color.get(2);

        }
    } else {
        if (store->colors!=NULL) {
            result.color =
                WorldOpTriplet(store->colors[op.index.index-1][0],
                               store->colors[op.index.index-1][1],
                               store->colors[op.index.index-1][2]);
        }
    }
    if (object==NULL) {
        result.setFail("no geometry found");
        return;
    }
    result.setOk();
}

void OdeLink::doDelete() {
    OdeInit& odeinit = OdeInit::get();

    if (op.kind.get() == "all") {
        lock_guard<mutex> lck(odeinit.mtx);
        odeinit._wrld->box_dynamic.clear();
        odeinit._wrld->box_static.clear();
        odeinit._wrld->cylinder_dynamic.clear();
        odeinit._wrld->cylinder_static.clear();
        odeinit._wrld->model_dynamic.clear();
        odeinit._wrld->model_static.clear();
        odeinit._wrld->sphere_dynamic.clear();
        odeinit._wrld->sphere_static.clear();
        result.setOk();
        return;
    }
    if (!checkObject()) return;
    if (store!=NULL) {
        // "store" + "object" can be used to access object to be destroyed
    }
    result.setFail("delete operation implemented only for target: all");
}

void OdeLink::doNumber() {
    if (!checkObject(true)) return;
    if (store==NULL) {
        result.setFail("cannot access that kind of object");
        return;
    }
    OdeInit& odeinit = OdeInit::get();
    lock_guard<mutex> lck(odeinit.mtx);
    int ct = store->length();
    result.count = WorldOpIndex(ct);
    result.setOk();
}

void OdeLink::apply() {
    yDebug("ODE world\n");
    //op.show();
    std::lock_guard<std::mutex> lck(ODE_access);
    switch (op.cmd) {
    case WORLD_OP_GET:
        doGet();
        break;
    case WORLD_OP_SET:
        doSet();
        break;
    case WORLD_OP_MK:
        doMake();
        break;
    case WORLD_OP_GRAB:
        doGrab();
        break;
    case WORLD_OP_ROT:
        doRotate();
        break;
    case WORLD_OP_DEL:
        doDelete();
        break;
    case WORLD_OP_COL:
        doColor();
        break;
    case WORLD_OP_NUM:
        doNumber();
        break;
    default:
        result.setFail("unrecognized command");
        break;
    }
    //result.show();
}

void OdeWorldManager::apply(const WorldOp& op, WorldResult& result) {
    OdeLink link(op,result);
    link.apply();
}
