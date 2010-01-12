#include <stdio.h>

#include <rfwr.h>
#include <yarp/sig/Vector.h>

using namespace yarp::sig;

/**
*
@ingroup icub_rfwr
\defgroup icub_rfwr_test Rfwr test module.

Receptive field weighted regression test module.

\section intro_sec Description
Simple example to show how to train a RFWR network.
To be improved.

\author Lorenzo Natale

Copyright (C) 2007 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/rfwr/example.cpp
**/

// Simple example to show how to train a RFWR network.
// To be improved.
// May 07 -- nat

int main(int argc, const char **)
{
    RFWR net;
 
    Vector target(1);
    Vector input(1); 
    Vector pred(1); 
    Vector output(1);

    Vector norm_in(1);
    Vector norm_out(1); 

    norm_in(0)=100;
    norm_out(0)=100*100;

    net.initialize(1, 1, 
             false,false,
             0, 0.005, 0.05,
             norm_in, 
             norm_out); 

    /// train the network
    for(int ep=0;ep<30;ep++)
        for(int k=-100; k<100;k++)
            {
                input(0)=k;
                target(0)=input(0)*input(0);
        
                net.train(input, 
                          target,
                          pred);
            }

    /// now simulate the network
    for(int k=-100; k<100;k++)
    {
        input(0)=k;
        target(0)=k*k;

        net.simulate(input, 
                  output, 0.0);

        printf("In: %s Out: %s Des %s\n", 
            input.toString().c_str(), 
            output.toString().c_str(), 
            target.toString().c_str());
    }

    return 0;
}
