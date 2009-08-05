// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef SEQ_SYMBOLICSEQUENCE_INC
#define SEQ_SYMBOLICSEQUENCE_INC


#include "IUnit.h"


/**
 *
 * A sequence predictor specializing in symbolic patterns.
 *
 */
class SymbolicSequence : public IUnit {
private:
    std::vector<int> past;
    std::vector<int> future;
public:
    void clear() {
        past.clear();
        future.clear();
    }

    virtual int futureSize() const {
        return future.size();
    }

    void add(double past) {
        this->past.push_back((int)(past+0.5));
    }

    virtual int size() const {
        return past.size();
    }

    virtual double get(int index) const {
        return past[index];
    }


    int getSym(int index) const {
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
};




#endif

