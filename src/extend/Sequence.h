// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef SEQ_SEQUENCE_INC
#define SEQ_SEQUENCE_INC


#include "IUnit.h"


/**
 * The main implementation of a prediction unit.  
 * Historical, is being replaced with a more modular implementation.
 */
class Sequence : public IUnit {
private:
    std::vector<double> past;
    std::vector<double> future;
public:
    void clear() {
        past.clear();
        future.clear();
    }

    void add(double past) {
        this->past.push_back(past);
    }

    void add(const std::vector<double>& past) {
        for (std::vector<double>::const_iterator it=past.begin(); it!=past.end();
             it++) {
            add(*it);
        }
    }

    virtual int size() const {
        return past.size();
    }

    virtual int futureSize() const {
        return future.size();
    }

    virtual double get(int index) const {
        if (index<0) return 0.0;
        if (index>=past.size()) return 0.0;
        return past[index];
    }

    virtual double predict(int index) {
        if (extend(index+1)) {
            return future[index];
        }
        return 0;
    }

    virtual bool extend(int len);

    virtual void copy(const IUnit& alt) {
        clear();
        for (int i=0; i<alt.size(); i++) {
            add(alt[i]);
        }
    }

    void takeFuture(IUnit& alt, int len) {
        bool ok = alt.extend(len);
        clear();
        if (ok) {
            for (int i=0; i<len; i++) {
                add(alt.predict(i));
            }
        }
    }
};



#endif

