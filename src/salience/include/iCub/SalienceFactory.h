// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_SALIENCEFACTORY_INC
#define ICUB_SALIENCEFACTORY_INC

// std
#include <vector>

// iCub
#include "Salience.h"

class SalienceFactory {
public:
    virtual iCub::contrib::Salience *create() = 0;
    virtual yarp::os::ConstString getName() = 0;
};


template <class T>
class SalienceFactoryOf : public SalienceFactory {
private:
    yarp::os::ConstString name;
public:
    SalienceFactoryOf(const char *name) : name(name) {
    }

    virtual iCub::contrib::Salience *create() {
        return new T;
    }
    
    virtual yarp::os::ConstString getName() {
        return name;
    }
};


class SalienceFactories {
public:
    virtual ~SalienceFactories() {
        clear();
    }
    
    void add(SalienceFactory *factory) {
        group.push_back(factory);
    }

    static SalienceFactories& getPool() {
        return pool;
    }

    iCub::contrib::Salience *get(const char *name) {
        for (unsigned int i=0; i<group.size(); i++) {
            if (group[i]->getName() == name) {
                return group[i]->create();
            }
        }
        return 0;
    }

    std::vector<std::string> getNames() {
        std::vector<std::string> result;
        for (unsigned int i=0; i<group.size(); i++) {
            result.push_back(std::string(group[i]->getName().c_str()));
        }
        return result;
    }

private:
    void clear() {
        for (unsigned int i=0; i<group.size(); i++) {
            delete group[i];
            group[i] = NULL;
        }
        group.clear();
    }
    std::vector<SalienceFactory*> group;
    static SalienceFactories pool;
};

#endif

