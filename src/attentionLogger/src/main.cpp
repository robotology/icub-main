// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/AttentionLoggerModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_attentionLogger attentionLogger
 *
 * Attention system logging.
 *
 * \see iCub::contrib::AttentionLogger
 *
 * \author Jonas Ruesch
 *
 */

int main(int argc, char *argv[]) {

    Network yarp;
    AttentionLoggerModule module;
    module.setName("/attentionLogger"); // set default name of module
    return module.runModule(argc,argv);
}
