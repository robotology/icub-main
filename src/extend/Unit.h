// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef SEQ_UNIT_INC
#define SEQ_UNIT_INC


#include "IUnit.h"



/**
 * A generic container for a single prediction unit.
 * Call the set() method to set up its content.
 * All other methods are delegated to the content.
 */
class Unit : public IUnit {
private:
    boost::shared_ptr<IUnit> content;
public:
    Unit(IUnit *unit) : content(unit) {
    }

    Unit() {
    }

    void set(IUnit *unit) {
        content = boost::shared_ptr<IUnit>(unit);
    }

    virtual int size() const {
        return content->size();
    }

    virtual double get(int index) const {
        return content->get(index);
    }

    virtual double predict(int index) {
        return content->predict(index);
    }

    virtual bool extend(int len) {
        return content->extend(len);
    }

    virtual void copy(const IUnit& alt) {
        content->copy(alt);
    }

    virtual int futureSize() const {
        return content->futureSize();
    }

};


#endif

