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

    return true;
}


void OdeLink::doGet() {
    if (!checkObject()) return;
    if (store==NULL) {
        const dReal *coords = dBodyGetPosition(bid);
        result.location = WorldOpTriplet(coords[0],coords[1],coords[2]);
        result.setOk();
        return;
    }
    int index = op.index.get()-1;
    if (!store->inRange(index)) {
        result.setFail("out of range");
        return;
    }
    WorldObject& obj = store->get(index);
    if (op.dynamic.get()) {
        const dReal *coords = dBodyGetPosition(obj.getBody());
        result.location = WorldOpTriplet(coords[0],coords[1],coords[2]);
        result.setOk();
        return;
    } else {
        const dReal *coords = dGeomGetPosition(obj.getGeometry());
        result.location = WorldOpTriplet(coords[0],coords[1],coords[2]);
        result.setOk();
        return;
    }
}

void OdeLink::doSet() {
    result.setFail("set operation not implemented");
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
    result.setFail("grab operation not implemented");
}

void OdeLink::doRotate() {
    result.setFail("rotate operation not implemented");
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
