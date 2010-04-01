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
#include <iCub/EgoSphereModule.h>

// OpenCV
#include <cv.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_egoSphere egoSphere
 *
 * Spherical aggregation of head perceptions.
 *
 * \dot
 * digraph module_egoSphere_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_egoSphere {
 *      color = "black"; style = "solid";
 *      label = "egoSphere module";
 *       "/egoSphere/map_out";
 *       "/egoSphere/conf";
 *       "/egoSphere/controlboard";
 *		 "/egoSphere/mapVisual/map_in";
 *       "/egoSphere/mapVisual/rgb_in";
 *       "/egoSphere/mapVisual/rgb_out";
 *       "/egoSphere/mapVisual/click_in";
 *       "/egoSphere/mapAuditory/vct_in";
 *     }
 *     "/salience/map" -> "/egoSphere/map_out"
 *     "/camCalib/out" -> "/egoSphere/mapVisual/rgb_in"
 *     "/soundLocalization/o:vct" -> "/egoSphere/mapAuditory/vct_in"
 *     "/egoSphere/map_out" -> "/attentionSelection/i:map"
 *     "/egoSphere/map_out" -> "/viewer/map"
 *     "/egoSphere/mapVisual/rgb_out" -> "/viewer/rgb"
 *     "/viewer/rgb/click" -> "/egoSphere/mapVisual/click_in"
 *     "/controlGaze/remoteEgoSphere" -> "/egoSphere/conf"
 *     "/iCubInterface/head" -> "/egoSphere/controlboard"
 * \enddot
 *
 * Parameters: \n
 * - name:    prefix to be appended to port names \n
 * - context: path to the $ICUB_ROOT/app directory where configuration file should be searched \n 
 * - from: configuration file to be used \n
 * - controlboard: head control board port \n
 * - controlboardTorso: (optional) torso control board port \n
 *
 * Ports:\n
 * - /egoSphere/map_out Output / floating point image: Spherical multimodal salience map; the main output from egoSphere.\n
 * - /egoSphere/conf RPC: Process runtime configuration calls to egoSphere.\n
 * - /egoSphere/controlboard RPC/stream: Remote controlboard connected to iCub head server controlboard.\n
 * - /egoSphere/mapVisual/map_in Input / floating point image: The visual saliency map to project to the sphere.\n
 * - /egoSphere/mapVisual/rgb_in Input / RGB image: Rgb input image to project to the sphere (for visualization purposes only).\n
 * - /egoSphere/mapVisual/rgb_out Output / RGB image: Spherical mosaic of rgb input images.\n
 * - /egoSphere/mapVisual/click_in Input / Bottle: Receives x/y information from a mouse-click in a yarp image-viewer and adds an artificial salience spot to the output map.\n
 * - /egoSphere/mapAuditory/vct_in Input / Vector of doubles: Reads a double vector of size 5 containing sound source information in the format [azimuth, sigmaAzimuth, elevation, sigmaElevation, intensity].\n
 *
 * \see iCub::contrib::EgoSphereModule
 *
 * \author Jonas Ruesch, Alexandre Bernardino
 *
 */

int main(int argc, char *argv[]) {

	Network yarp;
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("egoSphere.ini"); //overridden by --from parameter
	rf.setDefaultContext("egoSphere"); //overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
    EgoSphereModule module;
    return module.runModule(rf);
}
