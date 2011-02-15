// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
* Copyright (C) 2011 Paul Fitzpatrick, Vadim Tikhanoff
* CopyPolicy: Released under the terms of the GNU GPL v2.0. 
*
*/

#ifndef ICUBSIMULATION_WORLDOP_INC
#define ICUBSIMULATION_WORLDOP_INC

#include <yarp/os/Vocab.h>

#include <string>

enum WORLD_OP {
    WORLD_OP_NULL,
    WORLD_OP_GET = VOCAB3('g','e','t'),
    WORLD_OP_SET = VOCAB3('s','e','t'),
    WORLD_OP_MK = VOCAB2('m','k'),
    WORLD_OP_GRAB = VOCAB4('g','r','a','b'),
    WORLD_OP_ROT = VOCAB3('r','o','t'),
    WORLD_OP_DEL = VOCAB3('d','e','l'),
};

class WorldOpDatum {
public:
    bool valid;

    WorldOpDatum() { valid = false; }

    bool isValid() const {
        return valid;
    }
};

class WorldOpName : public WorldOpDatum {
public:
    std::string name;

    WorldOpName() {}

    WorldOpName(const char *name) : name(name) { valid = true; }

    std::string get() const { return name; }
};

class WorldOpFlag : public WorldOpDatum {
public:
    bool setting;

    WorldOpFlag() { setting=false; }

    WorldOpFlag(bool setting) : setting(setting) { valid = true; }

    bool get() const { return setting; }
};

class WorldOpIndex : public WorldOpDatum {
public:
    int index;

    WorldOpIndex() { index = 0; }

    WorldOpIndex(int index) : index(index) { valid = true; }

    int get() const { return index; }
};

class WorldOpScalar : public WorldOpDatum {
public:
    double val;

    WorldOpScalar() { val = 0; }

    WorldOpScalar(double val) : val(val) { valid = true; }

    double get() const { return val; }
};

class WorldOpTriplet : public WorldOpDatum {
public:
    double x[3];

    WorldOpTriplet() { x[0] = x[1] = x[2] = 0; }

    WorldOpTriplet(double x, double y, double z) { 
        valid = true; 
        this->x[0] = x;
        this->x[1] = y;
        this->x[2] = z;
    }

    double get(int offset) const { return x[offset]; }

};

class WorldOp {
public:
    WORLD_OP cmd;
    WorldOpName kind;
    WorldOpName name;
    WorldOpIndex index;
    WorldOpTriplet location;
    WorldOpTriplet size;
    WorldOpTriplet color;
    WorldOpTriplet rotation;
    WorldOpScalar radius;
    WorldOpScalar length;
    WorldOpFlag active;
    WorldOpFlag dynamic;
    WorldOpFlag rightHanded;
    WorldOpFlag parameter;
    WorldOpName modelName;
    WorldOpName modelTexture;

    // for debugging
    void show() const;
};

class WorldResult {
public:
    bool success;
    std::string msg;
    WorldOpTriplet location;
    WorldOpName path;

    WorldResult() {
        success = false;
        msg = "";
    }

    void setOk() {
        success = true;
        msg = "";
    }

    void setFail(const char *msg) {
        success = false;
        this->msg = msg;
    }

    void show();
};

#endif
