// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Manuel Lopes
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/control_gaze.h>

using namespace std;
using namespace yarp::os;

using namespace iCub::contrib;

/**
 *
 *
 * Control gaze module.
 *
 * \dot
 * digraph module_camcalib_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_camcalib {
 *      color = "black"; style = "solid";
 *      label = "control gaze module";
 *       "/camcalib/image";
 *     }
 *     "/attention" -> "/control_gaze/gazingtarget"
 *     "/control_gaze/gazingtarget" -> "/controlboard/"
 * \enddot
 *
 * \see iCub::contrib::Control_GazeModule
 *
 * \author Manuel Lopes
 *
 */


int main(int argc, char *argv[]) {


    Network yarp;
    Control_GazeModule module;

    module.setName("/control_gaze"); // set default name of module

    return module.runModule(argc,argv);
}
