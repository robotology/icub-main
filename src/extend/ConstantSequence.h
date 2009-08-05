// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef SEQ_CONSTANTSEQUENCE_INC
#define SEQ_CONSTANTSEQUENCE_INC


#include "IUnit.h"
#include "Histogram.h"



/**
 *
 * A trivial sequence predictor specializing in constanst sequences.
 *
 */
class ConstantSequence : public IUnit {
private:
    Histogram h;
    int fsize;
public:
    ConstantSequence() {
        fsize = 0;
    }

    virtual int futureSize() const {
        return fsize;
    }

    void clear() {
        h.clear();
    }

    void add(double past) {
        h.add(past);
    }

    virtual int size() const {
        return h.getLength();
    }

    virtual double get(int index) const {
        return h.getPast(index);
    }

    virtual double predict(int index);

    virtual bool extend(int len) {
        if (len>fsize) {
            fsize = len;
        }
        return true;
    }

    virtual void copy(const IUnit& alt) {
        clear();
        for (int i=0; i<alt.size(); i++) {
            add(alt[i]);
        }
    }
};


#endif


