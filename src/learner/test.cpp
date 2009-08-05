// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;


class LearnerTesterModule : public Module, TypedReaderCallback<Vector> {
private:
    BufferedPort<Vector> outPos;
    BufferedPort<Vector> inPos;
    double v;
public:
    bool open(Searchable& config) {
        inPos.useCallback(*this);
        outPos.open(getName("o:vec"));
        inPos.open(getName("i:vec"));
        Network::connect(outPos.getName(),"/learner/i:vec");
        Network::connect("/learner/o:vec",inPos.getName());
        v = 0;
        return true;
    }

    void onRead(Vector& result) {
        if (result.size()>0) {
            printf("got prediction %g\n\n", result[0]);
        }
    }

    bool updateModule() {
        double x = 100*sin(v/25.0);

        Vector& data = outPos.prepare();
        data.resize(2);
        data[0] = x;
        data[1] = x*2;
        printf("sending example: %g %g\n", data[0], data[1]);
        outPos.write();
        Time::delay(0.1);

        Vector& query = outPos.prepare();
        query.resize(1);
        data[0] = x;
        printf("sending test: %g\n", data[0]);
        outPos.write();
        Time::delay(0.1);
        v += 1;
        return true;
    }
};

int main(int argc, char *argv[]) {
    Network yarp;

    printf("This is a tester for the learner module\n");
    printf("It assumes you have started the learner with options:\n");
    printf("   learner --dom 1 --cod 1 --ex NN\n");
    printf("It sends examples where y = 2x\n");

    LearnerTesterModule module;
    module.setName("/tester");
    return module.runModule(argc,argv);
}

