// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch inspired by Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_CALIBTOOLFACTORY__
#define __UZH_CALIBTOOLFACTORY__

// std
#include <vector>
#include <string>

// iCub
#include <iCub/ICalibTool.h>

class CalibToolFactory {
public:
    virtual iCub::contrib::ICalibTool *create() = 0;
    virtual yarp::os::ConstString getName() = 0;
};

template <class T>
class CalibToolFactoryOf : public CalibToolFactory {
private:
    yarp::os::ConstString name;
public:
    CalibToolFactoryOf(const char *name) : name(name) {
    }

    virtual iCub::contrib::ICalibTool *create() {
        return new T;
    }
    
    virtual yarp::os::ConstString getName() {
        return name;
    }
};


class CalibToolFactories {
public:
    virtual ~CalibToolFactories() {
        clear();
    }
    
    void add(CalibToolFactory *factory) {
        group.push_back(factory);
    }

    static CalibToolFactories& getPool() {
        return pool;
    }

    iCub::contrib::ICalibTool *get(const char *name) {
        for (unsigned int i=0; i<group.size(); i++) {
            if (group[i]->getName() == name) {
                return group[i]->create();
            }
        }
        return NULL;
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
    std::vector<CalibToolFactory*> group;
    static CalibToolFactories pool;
};

#endif
