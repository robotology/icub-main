/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_ocsserver OCSServer
 *
 *This OCSServer module connects to the RecServer module, receiving the rectified Y (intensity) channel from either the left or right camera and performs complex log-Gabor phase analysis of the image using a bank or complex oriented log-Gabor kernels. It produces numerous cue outputs including: orientation response for any processed orientation at multiple scales, orientation saliency, phase congruency edges (edge map), phase congruency corners (corners map), phase symmetry (skeletal symmetry map), and a combined server saliency map.  
 *
 * This module should be called in the following way:\n \b ocsserver \b -c \a ocs0.cfg\n
 * where ocs0.cfg is the config file.  Default configfile in primateVision/config/ocs0.cfg is used if none specified. NOTE: because the module is multi-threaded and processing each orientation is a heavy task, 2 (or multiple) servers can be configured to share the processing of blocks of orientations for each camera image.  In Testing! The current config files ocs0.cfg and ocs1.cfg do independent left camera and right camera processing.

 * The implementation incorporates and is inspired by the work and MATLAB 
 * implementation of Peter Kovesi.
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ye
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /ocsserver0/input/image
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /ocsserver0/output/ocs\n
 * <li> /ocsserver0/output/pc_edges\n
 * <li> /ocsserver0/output/pc_corners\n
 * <li> /ocsserver0/output/pc_sym\n
 * <li> /ocsserver0/output/osal\n
 * <li> /ocsserver0/output/os\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/ocsserver/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.  
 *
 * An implementation inspired by the work and MATLAB 
 * implementation of Peter Kovesi.
 * 
 */



#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include "ocs.h"


using namespace std;
using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/ocs0.cfg");

  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
      exit(0);
    }
  }


  //Create & launch processing thread:
  OCSServer*ocs = new OCSServer(&fname);

  
  return a.exec();

}
