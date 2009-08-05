/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_sptzdfserver SpTZDFServer
 *
 *This SpTZDFServer (Spatio-Temporal Zero Disparity Filter server) module is based upon the ZDFServer, extending it to incorporate temporal as well as spatial (stereo) cues for boosted stereo performance and robust continued operation under gross and complete occlusion of one camera. The SpTZDFServer connects to the RecServer module, recieving the rectified Y (intensity) channel from both the left and right camera and performs brobabalistic zero disparity filtering based upon MRF theory and Graph Cuts optimisation. It effectively segments the object at zero temporal and/or stereo disparity.  Moreover, the server has been designed to instruct the recserver to move the cameras to keep the object that has been extracted at the zero disparity position. This implements coordinated stereo fixation and tracking and segmentation of arbitrary, deformable objects.  
 *
 * This module should be called in the following way:\n \b sptzdfserver \b -c \a sptzdf.cfg\n
 * where sptzdf.cfg is the config file.  Default configfile in primateVision/config/sptzdf.cfg is used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ye
 * <li> /recserver/right_ye
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /sptzdfserver/input/left
 * <li> /sptzdfserver/input/right
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /sptzdfserver/output/zdseg\n
 * <li> /sptzdfserver/output/fovl\n
 * <li> /sptzdfserver/output/fovr\n
 * <li> /sptzdfserver/output/zdprob\n
 * <li> /sptzdfserver/output/motion_cmds\n
 * <li> /sptzdfserver/output/yarpimg\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/sptzdfserver/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include "sptzdf.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/sptzdf.cfg");

  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
      exit(0);
    }
  }


  //Create zdf processing/GUI sync thread:
  SpTZDFServer*m = new SpTZDFServer(&fname);


  return a.exec();

  
}
