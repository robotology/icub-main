// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006, 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <stdio.h>
#include <iostream>

#include "fetch.h"
#include "Sequence.h"
#include "Unit.h"
#include "Tests.h"

#include "yarp.h"

#include <yarp/os/Property.h>

using namespace std;
using namespace yarp::os;

string global_path = ".";


static vector<double> getSeq() {
    vector<double> result;
    while (!cin.eof()) {
        double v = 0;
        cin >> v;
        if (!cin.eof()) {
            result.push_back(v);
        }
    }
    return result;
}



int main(int argc, char *argv[]) {
    if (argc>1) {
        Property p;
        p.fromCommand(argc,argv);
        if (p.check("dir")) { 
            global_path = p.find("dir").asString().c_str();
        }
        std::string seq = argv[1];
        if (seq=="--net") {
            return netmain(argc,argv);
        }
        if (seq=="--test") {
            Tests tests;
            bool ok = tests.run();
            printf("Tests outcome: %s\n", ok?"OK":"Fail");
            return ok;
        }

        std::string pattern = sequenceToPattern(seq);
        printf("pattern is %s\n", pattern.c_str());
        if (argc>2) {
            int ct = atoi(argv[2]);
            std::string seqExt = extendSequence(seq,pattern,ct);
            printf("extended sequence by %d from %s: %s\n", 
                   ct, seq.c_str(), seqExt.c_str());
	     
        }
    } else {
        Sequence seq;

        vector<double> lst = getSeq();
        for (int i=0; i<lst.size(); i++) {
            seq.add(lst[i]);
        }

        Unit unit(seq.specialize(seq));

        Sequence fut;
        fut.takeFuture(unit,lst.size()*5);
        cout << fut.toString() << endl;

    }
    return 0;
}

