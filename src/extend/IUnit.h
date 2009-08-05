// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#ifndef SEQ_IUNIT_INC
#define SEQ_IUNIT_INC

#include <vector>
#include <list>
#include <string>
#include <stdio.h>

#include <boost/shared_ptr.hpp>

/**
 *
 * Basic interface for sequence prediction.  We model
 * a sequence as a vector, with the added ability to
 * extend that sequnce with plausible data. "Plausible" means loosely
 * following the pattern of the sequence so far.
 *
 */
class IUnit {
public:

    // normal vector-like operations

    virtual ~IUnit() {}

    virtual int size() const = 0;

    virtual double get(int index) const = 0;

    double operator[] (int index) const {
        return get(index);
    }

    virtual void copy(const IUnit& alt) = 0;

    std::string toString() const {
        std::string str = "";
        for (int i=0; i<size(); i++) {
            if (i>0) {
                str += " ";
            }
            char buf[256];
            sprintf(buf,"%g",get(i));
            str += buf;
        }
        return str;
    }


    // into-the-future operations

    /**
     * Number of extra predicted values available in cache.
     */
    virtual int futureSize() const = 0;

    /**
     * Return predicted value at a certain offset beyond end of vector.
     */
    virtual double predict(int index) = 0;

    /**
     * Predict values out to a certain offset beyond end of vector.
     */
    virtual bool extend(int len) = 0;

    /**
     * Create a specialized representation of the sequence, based on its
     * content.  
     */
    static IUnit *specialize(const IUnit& source);
};



#endif

