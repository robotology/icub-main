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
#include <iCub/ProxySalience.h>

#include <iCub/vis/SalienceFactory.h>
#include <iCub/vis/MotionSalience.h>
#include <iCub/vis/RuddySalience.h>
#include <iCub/vis/ColorSalience.h>
#include <iCub/vis/GroupSalience.h>
#include <iCub/vis/EMDSalience.h>   
#include <iCub/vis/CvFaceSalience.h>
#include <iCub/vis/IntensitySalience.h>
#include <iCub/vis/DirectionalSalience.h>
#include <iCub/vis/AdvancedDirectionalSalience.h>


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
 * \see iCub::vis::SalienceModule
 * \see iCub::vis::Salience
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
    return module.runModule(rf);     
}

