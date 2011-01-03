// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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

class WorldOpName {
public:
    bool valid;
    std::string name;

    WorldOpName() { valid = false; }

    WorldOpName(const char *name) : name(name) { valid = true; }
};

class WorldOpFlag {
public:
    bool valid;
    bool setting;

    WorldOpFlag() { valid = false; }

    WorldOpFlag(bool setting) : setting(setting) { valid = true; }
};

class WorldOpIndex {
public:
    bool valid;
    bool index;

    WorldOpIndex() { valid = false; }

    WorldOpIndex(int index) : index(index) { valid = true; }
};

class WorldOpScalar {
public:
    bool valid;
    double val;

    WorldOpScalar() { valid = false; }

    WorldOpScalar(double val) : val(val) { valid = true; }
};

class WorldOpTriplet {
public:
    bool valid;
    double x[3];

    WorldOpTriplet() { valid = false; }

    WorldOpTriplet(double x, double y, double z) { 
        valid = true; 
        this->x[0] = x;
        this->x[1] = y;
        this->x[2] = z;
    }
};

class WorldOp {
public:
    WORLD_OP cmd;
    WorldOpName kind;
    WorldOpName name;
    WorldOpTriplet location;
    WorldOpTriplet size;
    WorldOpTriplet color;
    WorldOpTriplet rotation;
    WorldOpScalar radius;
    WorldOpFlag active;
    WorldOpIndex index;
};

class WorldResult {
public:
    bool success;

    WorldResult() {
        success = false;
    }
};

#endif
