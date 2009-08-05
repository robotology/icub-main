/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_flowserver FlowServer
 *
 *This FlowServer module connects to the RecServer module, receiving rectified Y (intensity) channels, and rectification parameters (mosaic positions) for either the left or right camera to perform optical flow calculation using a MMX-optimised SAD implementation. Because the flow calculation is conducted in rectified mosaic coordinates, the output flow maps reflect scene flow only, and not flow induced by the motion of the cameras themselves! (within tolerances induced by encoder error, backlash, etc). Two types of flow maps are generated: one map is rich, but with low confidences and is useful for probability propagation such as that conducted in the IORServer. The other is sparse and confident, useful for flow saliency calculation where it is preferred that attentional saccade does not transfer gaze to a location elicited by noise (can occur in bland/saturated image areas). The server also outputs a saliency map, where objects moving faster are deemed more salient (intentionally not centre-surround flow saliency). 
 *
 * This module should be called in the following way:\n \b flowserver \b -c \a flowL.cfg\n
 * where flowL.cfg is the config file.  Default configfile in primateVision/config/flowL.cfg is used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ye
 * <li> /recserver/rec_params
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /flowserverL/input/image
 * </li>
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /flowserverL/output/flowx\n
 * <li> /flowserverL/output/flowy\n
 * <li> /flowserverL/output/flowsx\n
 * <li> /flowserverL/output/flowsy\n
 * </li>
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/colcsserver/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
//MY INCLUDES
#include "flows.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/flowL.cfg");

  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
      exit(0);
    }
  }


  //Create & launch flow processing thread:
  FlowServer*f = new FlowServer(&fname);


  return a.exec();

  
}
