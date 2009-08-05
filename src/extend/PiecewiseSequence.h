// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef SEQ_PIECEWISESEQUENCE_INC
#define SEQ_PIECEWISESEQUENCE_INC


#include "IUnit.h"
#include "Unit.h"
#include "SymbolicSequence.h"

#include "Histogram.h"



class Piece {
public:
    double mean, dev;
    double len;
    Unit lengthUnit;
};


/**
 *
 * Asequence predictor specializing in piece-wise continuous sequences.
 *
 */
class PiecewiseSequence : public IUnit {
private:
    SymbolicSequence sym;
    std::vector<Piece> parts;
    Piece whole;
    std::vector<double> future;
    int places, scale;
    int trimLeft, trimRight;
    double meanLength;

    bool extendParts(int len, int pad);
public:
    virtual void copy(const IUnit& alt) {
        apply(alt);
    }

    void apply(const IUnit& unit);


    virtual int size() const {
        return sym.size();
    }

    virtual int futureSize() const {
        return future.size();
    }

    virtual double get(int index) const {
        return sym[index];
    }

    virtual double predict(int index) {
        if (extend(index+1)) {
            return future[index];
        }
        return 0;
    }

    virtual bool extend(int len);
};


#endif
