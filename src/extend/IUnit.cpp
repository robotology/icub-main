// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



#include <iostream>

#include <math.h>

#include "IUnit.h"
#include "Unit.h"
#include "Sequence.h"
#include "ConstantSequence.h"
#include "PiecewiseSequence.h"

using namespace std;
using namespace boost;

#define DBG if (0) 
//#define DBG if (0) 



IUnit *IUnit::specialize(const IUnit& source) {
    // I should create a unit that is appropriate for my source

    bool varies = false;
    double maxd = 0;
    Histogram h;
    for (int i=1; i<source.size(); i++) {
        if (source[i]!=source[0]) {
            varies = true;
        }
        double dx = source[i]-source[i-1];
        if (dx>maxd) {
            maxd = dx;
        }
        h.add(source[i]);
    }

    DBG cerr << "Max deviation is " << maxd << " versus " 
             << h.getDeviation() << endl;

    if (varies) {
        if (maxd<h.getDeviation() && source.size()>=20) {
            DBG cerr << "Treating as plateau-symbolic" << endl;
            Sequence *seq = new Sequence();
            assert(seq!=NULL);
            seq->copy(source);
            return seq;
        }
        
        if (source.size()>=10) {
            DBG cerr << "Treating as piecewise continuous" << endl;
            PiecewiseSequence *pw = new PiecewiseSequence();
            assert(pw!=NULL);
            pw->apply(source);
            return pw;
        }

        if (source.size()>=4) {
            DBG cerr << "Treating as symbolic" << endl;
            SymbolicSequence *seq = new SymbolicSequence();
            assert(seq!=NULL);
            seq->copy(source);
            return seq;
        }
    }
    DBG cerr << "Treating as constant" << endl;
    IUnit *c = new ConstantSequence();
    assert(c!=NULL);
    c->copy(source);
    return c;
}
