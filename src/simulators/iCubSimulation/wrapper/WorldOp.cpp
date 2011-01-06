// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "WorldOp.h"

#include <stdio.h>

void show(const WorldOpTriplet& x) {
    if (!x.valid) {
        printf("not set");
    } else {
        printf("%g %g %g", x.x[0], x.x[1], x.x[2]);
    }
    printf("\n");
}

void show(const WorldOpScalar& x) {
    if (!x.valid) {
        printf("not set");
    } else {
        printf("%g", x.val);
    }
    printf("\n");
}

void show(const WorldOpFlag& x, 
          const char *yes="True", 
          const char *no="False") {
    if (!x.valid) {
        printf("not set");
    } else {
        printf("%s", x.setting?yes:no);
    }
    printf("\n");
}

void show(const WorldOpIndex& x) {
    if (!x.valid) {
        printf("not set");
    } else {
        printf("%d", x.index);
    }
    printf("\n");
}

void show(const WorldOpName& x) {
    if (!x.valid) {
        printf("not set");
    } else {
        printf("%s", x.name.c_str());
    }
    printf("\n");
}

void WorldOp::show() const {
    printf("Operation:\n");
    printf("  cmd?: %s\n", yarp::os::Vocab::decode(cmd).c_str());
    printf("  kind: ");
    ::show(kind);
    printf("  name: ");
    ::show(name);
    printf("  dynamic: ");
    ::show(dynamic);
    printf("  location: ");
    ::show(location);
    printf("  size: ");
    ::show(size);
    printf("  color: ");
    ::show(color);
    printf("  rotation: ");
    ::show(rotation);
    printf("  radius: ");
    ::show(radius);
    printf("  length: ");
    ::show(length);
    printf("  active: ");
    ::show(active);
    printf("  index: ");
    ::show(index);
    printf("  rightHanded?: ");
    ::show(rightHanded,"right","left");
    printf("  modelName: ");
    ::show(modelName);
    printf("  modelTexture: ");
    ::show(modelTexture);
}

void WorldResult::show() {
    printf("Result:\n");
    printf("  success: %s\n", success?"true":"false");
    printf("  msg: %s\n", msg.c_str());
    printf("  location: ");
    ::show(location);
 }





