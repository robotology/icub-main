// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "Tests.h"
#include "Sequence.h"
#include "Unit.h"

#include <math.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
using namespace boost;


static double alternator_data[] = {
	200.0,
	199.71745,
	199.547511,
	199.698568,
	199.547511,
	199.683462,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.547511,
	199.097065,
	199.322288,
	200.0,
	199.548192,
	200.0,
	199.661144,
	199.547511,
	199.638418,
	199.547511,
	199.623267,
	199.097065,
	199.548095,
	200.0,
	199.604583,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	200.0,
	199.547511,
	199.773756,
	199.547511,
	199.698341,
	199.097065,
	199.548022,
	199.097065,
	199.457831,
	200.0,
	199.548192,
	200.0,
	199.612736,
	199.547511,
	199.604583,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.097065,
	199.547511,
	199.322288,
	199.547511,
	199.397363,
	199.547511,
	199.4349,
	199.547511,
	199.457422,
	199.097065,
	199.397363,
	199.097065,
	199.354463,
	199.547511,
	199.378594,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.547511,
	200.0,
	199.773756,
	199.097065,
	199.548192,
	200.0,
	199.661144,
	199.547511,
	199.638418,
	199.547511,
	199.623267,
	199.547511,
	199.612444,
	199.547511,
	199.604328,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.547511,
	199.547511,
	199.547511,
	200.0,
	199.698341,
	199.547511,
	199.660633,
	200.0,
	199.728507,
	199.547511,
	199.698341,
	200.0,
	199.741435,
	199.547511,
	199.717195,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	200.0,
	199.547511,
	199.773756,
	199.547511,
	199.698341,
	199.547511,
	199.660633,
	199.097065,
	199.54792,
	200.0,
	199.623267,
	200.0,
	199.677086,
	199.547511,
	199.660889,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.097065,
	200.0,
	199.548533,
	199.547511,
	199.548192,
	200.0,
	199.661144,
	200.0,
	199.728915,
	199.097065,
	199.623607,
	199.097065,
	199.548387,
	200.0,
	199.604838,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.547511,
	199.097065,
	199.322288,
	200.0,
	199.548192,
	199.097065,
	199.435411,
	200.0,
	199.548328,
	199.547511,
	199.548192,
	199.547511,
	199.548095,
	199.097065,
	199.491716,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.547511,
	199.097065,
	199.4349,
	200.0,
	199.54792,
	199.547511,
	199.547852,
	200.0,
	199.612444,
	199.547511,
	199.604328,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
    -999
};



bool runTest(double *in, int len1, 
             double *out, int len2, 
             double tolerance = 0.001,
             bool verbose = true,
             bool output = false) {
    if (len1==-1) {
        len1 = 0;
        while (fabs(*(in+len1)+999)>0.001) {
            len1++;
        }
    }
    if (len2==-1) {
        len2 = 0;
        while (fabs(*(out+len2)+999)>0.001) {
            len2++;
        }
    }


    Sequence seq;
    for (int i=0; i<len1; i++) {
        seq.add(*in);
        in++;
    }
    Unit unit(seq.specialize(seq));
    Sequence fut;
    fut.takeFuture(unit,len2);
    if (tolerance>=0) {
        for (int i=0; i<len2; i++) {
            if (fabs(*(out+i)-fut.get(i))>=tolerance) {
                if (verbose) {
                    printf("Problem at [%d], %g (predicted) versus %g (actual)\n",
                           i, fut.get(i), *(out+i));
                    printf("%s -> %s\n", 
                           seq.toString().c_str(), 
                           fut.toString().c_str());
                }
                return false;
            }
        }
    }
    if (output) {
        for (int i=0; i<len2; i++) {
            out[i] = fut.get(i);
        }
    }
    return true;    
}

bool runGen(double *in, int len1, 
            double *out, int len2) {
    return runTest(in,len1,out,len2,-1,false,true);
}

bool runTest(double *in, double *out, double tolerance = 0.001) {
    return runTest(in,-1,out,-1,tolerance);
}

bool runTest(double *in, int len1, int lenTotal, double tolerance = 0.001) {
    if (len1==-1) { exit(1); }
    return runTest(in,len1,in+len1,lenTotal-len1,tolerance);
}


bool testSimpleAlternation() {
    printf(">>> Testing simple alternation\n");
    double in[] = { 1, 2, 1, 2, 1, -999 };
    double out[] = { 2, 1, 2, 1, 2, 1, 2, -999 };
    return runTest(in,out);
}

bool testDoubleAlternation() {
    printf(">>> Testing simple double alternation\n");
    double in[] = { 1, 1, 2, 2, 1, 1, -999 };
    double out[] = { 2, 2, 1, 1, 2, 2, -999 };
    return runTest(in,out);
}

bool testSinusoid() {
    printf(">>> Testing sinusoid\n");
    double in[1000];
    for (int i=0; i<1000; i++) {
        in[i] = sin(i*0.4);
    }
    bool ok = true;
    ok = ok && runTest(in,50,150,0.2);
    ok = ok && runTest(in,75,150,0.2);
    return ok;
}


bool testSquaredSinusoid() {
    printf(">>> Testing squared sinusoid\n");
    double in[1000];
    for (int i=0; i<1000; i++) {
        in[i] = sin(i*0.2);
        in[i] *= in[i];
    }
    bool ok = true;
    ok = ok && runTest(in,50,100,0.2);
    ok = ok && runTest(in,75,150,0.2);
    return ok;
}


bool testAlternationWithNoise() {
    printf(">>> Testing alternation with noise\n");
    double in[] = { 
        0, 1, 0, 1, 10, 11, 12, 11, 0, 0, 1, 1, 11, 11, 10, 10,
        -999
    };
    return runTest(in,in,5);
}


bool testDurationAlternation() {
    printf(">>> Testing duration alternation\n");
    double in[] = { 
        0, 0, 10, 10, 10, 0, 0, 10, 10, 10, 0, 0, 10, 10, 10,
        -999
    };
    return runTest(in,in);
}


#define OUT_LEN 3

bool testSpeed() {
    printf(">>> Making sure speed is acceptable\n");

    mt19937 rng;
    rng.seed(0);
    boost::uniform_real<> uni_dist(0,1);
    boost::variate_generator<mt19937&, boost::uniform_real<> >
        uni(rng, uni_dist);

    double out[OUT_LEN];
    for (int i=0; i<OUT_LEN; i++) {
        out[i] = 0;
    }
    for (int len=1; len<=15; len++) {
        printf(">>> speed test for length %d\n", len);
        double *in = new double[len];
        for (int k=0; k<1000; k++) {
            for (int at=0; at<len; at++) {
                in[at] = uni();
            }
            runTest(in,len,out,OUT_LEN,0.001,false);
        }
        delete[] in;
    }
    return true;
}

#define OUT_LEN_TWIDDLE 100
bool testTwiddle() {
    printf(">>> Testing a twiddle\n");
    double in[] = { 
        0, 0, 0, 0, 0, 0, 0, 50, 55, 52, 0, 0, 54, 52, 58, 0, 0, 0, 0, 0, 0,
        -999
    };
    double out[OUT_LEN_TWIDDLE];
    runTest(in,-1,out,OUT_LEN_TWIDDLE,-1);
    int zs = 0;
    for (int i=0; i<OUT_LEN_TWIDDLE; i++) {
        if (fabs(out[i]-0)<0.05) {
            zs++;
        }
    }
    if (zs<40) {
        printf("NOT ENOUGH ZEROS (just %d out of %d)\n", zs, OUT_LEN_TWIDDLE);
        return false;
    }

    return true;
}


bool testHarderTwiddle() {
    printf(">>> Testing a harder twiddle\n");
    double in[] = { 
        0, 0, 0, 0, 0, 0, 0, 50, 53, 52, 0, 0, 60, 63, 61, 0, 0, 50, 51, 50, 0,
        0, 0, 0, 0, 0, 0, 0, 50, 53, 52, 0, 0, 60, 63, 61, 0, 0, 50, 51, 50, 0,
        0, 0, 0, 0, 0, 0, 0,
        -999
    };
    double out[OUT_LEN_TWIDDLE];
    runTest(in,-1,out,OUT_LEN_TWIDDLE,-1);
    int zs = 0;
    for (int i=0; i<OUT_LEN_TWIDDLE; i++) {
        if (fabs(out[i]-0)<0.05) {
            zs++;
        }
    }
    if (zs<40) {
        printf("NOT ENOUGH ZEROS (just %d out of %d)\n", zs, OUT_LEN_TWIDDLE);
        return false;
    }

    return true;
}



bool testTiming() {
    printf(">>> Testing timing\n");
    double in[] = { 
        0, 0, 0, 0, 0, 0, 0, 50, 53, 52, 
        0, 0, 0, 0, 0, 0, 0, 51, 52, 53, 
        0, 0, 0, 0, 0, 0, 0, 51, 52, 53, 
        -999
    };
    double out[OUT_LEN_TWIDDLE];
    int th = 5;
    runTest(in,-1,out,th,-1);
    int zs = 0;
    for (int i=0; i<th; i++) {
        if (fabs(out[i]-0)<0.05) {
            zs++;
        }
    }
    if (zs<th) {
        printf("NOT ENOUGH ZEROS (just %d out of %d)\n", zs, th);
        return false;
    }

    return true;
}


#define PRED_ALT 100
bool testEmpAlternation() {
    printf(">>> Testing empirical alternation sequence\n");
    double out[PRED_ALT];
    for (int i=0; i<PRED_ALT; i++) {
        out[i] = 0;
    }
    runGen(alternator_data,-1,out,PRED_ALT);
    for (int i=0; i<PRED_ALT; i++) {
        printf("[%g] ", out[i]);
    }
    printf("\n");
    return true;
};


bool Tests::run() {
    bool ok = true;

    ok = ok && testSimpleAlternation();
    ok = ok && testDoubleAlternation();
    ok = ok && testSinusoid();
    ok = ok && testSquaredSinusoid();
    ok = ok && testAlternationWithNoise();

    ok = ok && testDurationAlternation();
    ok = ok && testTwiddle();
    ok = ok && testHarderTwiddle();
    ok = ok && testTiming();
    ok = ok && testEmpAlternation();
    //ok = ok && testSpeed();
    return ok;
}





