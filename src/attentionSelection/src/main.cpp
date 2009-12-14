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
#include <iCub/AttentionSelectionModule.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_attentionSelection attentionSelection
 *
 * Attention selection
 *
 * The module expects a spherical floating point image (the saliency map / egosphere
 * output map) and outputs
 * a target location in angles (azimuth, elevation).\n
 * In order to calculate the angles the module configuration needs to specify
 * horizontal and vertical view angles for the given input image.\n
 * Currently the target location is a simple maximum on the incomming saliency map. \n
 * Planned: More sophisticated attention selection between tracked objects/regions and current salience.\n
 *
 *  \dot
 * digraph module_attentionselection_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_attentionselection {
 *      color = "black"; style = "solid";
 *      label = "attentionselection module";
 *       "/attentionSelection/i:map";
 *       "/attentionSelection/o:position";
 *       "/attentionSelection/remoteEgoSphere";
 *     }
 *     "/egosphere/ego/map" -> "/attentionSelection/i:map"
 *     "/attentionSelection/o:position" -> "/gaze_controller/pos"
 *     "/attentionSelection/remoteEgoSphere" -> "/egoSphere/conf"
 * \enddot
 *
 * \see iCub::contrib::AttentionSelectionModule
 *
 * \author Jonas Ruesch
 *
 */

int main(int argc, char *argv[]) {

	Network yarp;
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("attentionSelection.ini"); //overridden by --from parameter
	rf.setDefaultContext("attentionSelection"); //overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
    AttentionSelectionModule module;
    module.setName("/attentionSelection");
	module.attachTerminal();
    return module.runModule(rf);  
}
