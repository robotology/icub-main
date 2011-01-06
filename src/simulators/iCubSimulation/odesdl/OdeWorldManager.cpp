// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007, 2010 Vadim Tikhanoff, Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include "OdeWorldManager.h"

#include "OdeInit.h"
#include "iCub_Sim.h"

#include <map>

#define DENSITY (1.0)		// density of all objects

using namespace yarp::os;
using namespace std;

class OdeLink {
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
    void apply();
};

bool OdeLink::checkObject(bool forCreate) {
    OdeInit& odeinit = OdeInit::get();

    bid = (dBodyID)0;
    gid = (dGeomID)0;
    store = NULL;
    object = NULL;

    std::string kind = op.kind.get();

    if (kind=="cube") {
        bid = odeinit._wrld->Box;
    } else if (kind=="ball") {
        bid = odeinit._wrld->ballBody;
    }

    if (!forCreate) {
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

    if (!forCreate) {
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
    if (!checkObject()) return;
    if (bid!=NULL) {
        const dReal *coords = dBodyGetPosition(bid);
        result.location = WorldOpTriplet(coords[0],coords[1],coords[2]);
        result.setOk();
        return;
    }
    if (gid!=NULL) {
        const dReal *coords = dGeomGetPosition(gid);
        result.location = WorldOpTriplet(coords[0],coords[1],coords[2]);
        result.setOk();
        return;
    }
    result.setFail("no object found");
}

void OdeLink::doSet() {
    if (!checkObject()) return;
    if (!op.location.isValid()) {
        result.setFail("no location set");
        return;
    }
    if (bid!=NULL) {
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
    odeinit.mutex.wait();
    if (store->create(op,result)) {
        result.setOk();
    }
    odeinit.mutex.post();
}

void OdeLink::doGrab() {
    if (!checkObject()) return;

    OdeInit& odeinit = OdeInit::get();

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
            dJointDestroy(odeinit._iCub->grab);
        }
    }
    result.setOk();
}


void OdeLink::doRotate() {
    if (!checkObject()) return;
    if (!op.rotation.isValid()) {
        result.setFail("no rotation set");
        return;
    }
    if (object==NULL) {
        result.setFail("no geometry found");
        return;
    }

    dMatrix3 Rtx,Rty,Rtz, Rtmp1,Rtmp2;
    
    double rotx = (op.rotation.get(0) * M_PI) / 180;
    double roty = (op.rotation.get(1) * M_PI) / 180;
    double rotz = (op.rotation.get(2) * M_PI) / 180;
    
    dRFromAxisAndAngle(Rtx,1,0,0,rotx);
    dRFromAxisAndAngle(Rty,0,1,0,roty);
    dRFromAxisAndAngle(Rtz,0,0,1,rotz);
    
    dMultiply0 (Rtmp1,Rty,Rtz,3,3,3);
    dMultiply0 (Rtmp2,Rtx,Rtmp1,3,3,3);
    dGeomSetRotation(object->getGeometry(),Rtmp2);
    result.setOk();
}

void OdeLink::doDelete() {
    result.setFail("delete operation not implemented");
    /*
    if (op.kind.get() == "all") {
        deleteObjects();
        result.setOk();
        return;
    }
    result.setFail("delete operation implemented only for target: all");
    */
}

void OdeLink::apply() {
    printf("ODE world\n");
    op.show();
    ODE_access.wait();
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
    default:
        result.setFail("unrecognized command");
        break;
    }
    result.show();
    ODE_access.post();
}

void OdeWorldManager::apply(const WorldOp& op, WorldResult& result) {
    OdeLink link(op,result);
    link.apply();
}
