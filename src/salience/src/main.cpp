// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
 
#define YARP_CVTYPES_H_

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <yarp/os/Module.h>

using namespace yarp::os;
using namespace yarp::sig;

#include <iCub/SalienceModule.h>
#include <iCub/SalienceFactory.h>
#include <iCub/MotionSalience.h>
#include <iCub/RuddySalience.h>
#include <iCub/ColorSalience.h>
#include <iCub/GroupSalience.h>
#include <iCub/ProxySalience.h>
#include <iCub/EMDSalience.h>   
#include <iCub/CvFaceSalience.h>
#include <iCub/IntensitySalience.h>
#include <iCub/DirectionalSalience.h>
#include <iCub/AdvancedDirectionalSalience.h>


/**
 * @ingroup icub_module
 *
 * \defgroup icub_salience salience
 *
 * A simple implementation of some common preattentive filters.
 * Run without options to see help on usage.
 *
 * The module opens a port for reading and writing images, and
 * a port for writing filtered salience maps (type ImageOf<PixelFloat>).
 * You need to send images to the image port, or nothing will happen.
 * You can view the output by connecting the image port out to a viewer.\n
 * The underlying saliency filter can be configured through the conf port.
 *
 *  \dot
 * digraph module_salience_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_salience {
 *      color = "black"; style = "solid";
 *      label = "salience module";
 *       "/salience/view";
 *       "/salience/map";
 *       "/salience/conf";
 *       "/salience/peak";
 *     }
 *     "/salience/map" -> "/salience/aggregator"
 *     "/camera" -> "/salience/view"
 *     "/salience/view" -> "/image/viewer"
 *     "/salience/conf" -> "/salience/configurator"
 *     "/salience/peak" -> "/peakBottleReader"
 * \enddot
 *
 * \see iCub::contrib::SalienceModule
 * \see iCub::contrib::Salience
 *
 * \author Paul Fitzpatrick, Jonas Ruesch
 *
 */

int main(int argc, char *argv[]) {
    
    SalienceFactories& pool = SalienceFactories::getPool();
    pool.add(new SalienceFactoryOf<MotionSalience>("motion"));
    pool.add(new SalienceFactoryOf<EMDSalience>("emd"));
    pool.add(new SalienceFactoryOf<RuddySalience>("ruddy"));
    pool.add(new SalienceFactoryOf<ColorSalience>("color"));
    pool.add(new SalienceFactoryOf<GroupSalience>("group"));
    pool.add(new SalienceFactoryOf<CvFaceSalience>("face"));
    pool.add(new SalienceFactoryOf<IntensitySalience>("intensity"));
    pool.add(new SalienceFactoryOf<DirectionalSalience>("directional"));
    pool.add(new SalienceFactoryOf<AdvancedDirectionalSalience>("advdirectional"));
    pool.add(new SalienceFactoryOf<ProxySalience>("proxy"));

	Network yarp;
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("salience.ini"); //overridden by --from parameter
	rf.setDefaultContext("salience"); //overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
    SalienceModule module;
    module.setName("/salience");
    return module.runModule(rf);     
}

