/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Executable for merging data from different ports into a single LM formatted
 * port (portablepair<vector>)
 *
 */
 
#include <iostream>
#include <map>

#include <yarp/sig/Vector.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>

using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
namespace learningmachine {


class MergeModule {

};

} // learningmachine
} // iCub

using namespace iCub::learningmachine;

int main(int argc, char *argv[]) {
  Network yarp;
}

