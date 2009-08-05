// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Hornstein, Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/SoundLocalizationModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_soundlocalization soundlocalization
 *
 * Sound Localization
 *
 * The module streams a vector of position and correlation values in neck coordinate system. 
 *
 *  \dot
 * digraph module_soundlocalization_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_soundlocalization {
 *      color = "black"; style = "solid";
 *      label = "soundlocalization module";
 *       "/soundlocalization/i:sound";
 *       "/soundlocalization/o:loc";
 *     }
 *     "/soundgrabber" -> "/soundlocalization/i:sound"
 *     "/soundlocalization/o:loc" -> "/egosphere/sound"
 * \enddot
 *
 * \see iCub::contrib::SoundLocalizationModule
 *
 * \author Jonas Hornstein, Jonas Ruesch
 *
 */

int main(int argc, char *argv[]) {

    Network yarp;
    SoundLocalizationModule module;
    module.setName("/soundlocalization"); // set default name of module
    return module.runModule(argc,argv);
}
