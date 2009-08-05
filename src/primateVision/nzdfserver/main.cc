/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_nzdfserver NZDFServer
 *
 *This NZDFServer module extends the ZDFServer module to additionally include visual surfaces connected continuously to the zero disparity surface in the returned segmentation. It connects to the RecServer module, recieving the rectified Y (intensity) channel from both the left and right camera and performs probabalistic near-zero disparity filtering based upon MRF theory and Graph Cuts optimisation. It effectively segments the object at stereo fixation, and the visual surfaces continuously connected to the zero disparity region. The standard ZDFServer would otherwise explicitly measure these regions as not being at zero disparity, and probablistically exclude them from segmentation. In this manner, an object that has been fixated upon, but which extends beyond or infront of the horopter can be segmented in its entirity. Moreover, the server has been designed to instruct the recserver to move the cameras to keep the object that has been extracted at the zero disparity position. This implements coordinated stereo fixation and tracking and segmentation of arbitrary, deformable objects.  
 *
 * This module should be called in the following way:\n \b nzdfserver \b -c \a nzdf.cfg\n
 * where nzdf.cfg is the config file.  Default configfile in primateVision/config/nzdf.cfg is used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ye
 * <li> /recserver/right_ye
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /nzdfserver/input/left
 * <li> /nzdfserver/input/right
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /nzdfserver/output/zdseg\n
 * <li> /nzdfserver/output/fovl\n
 * <li> /nzdfserver/output/fovr\n
 * <li> /nzdfserver/output/zdprob\n
 * <li> /nzdfserver/output/motion_cmds\n
 * <li> /nzdfserver/output/yarpimg\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/nzdfserver/main.cc 
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
#include "nzdf.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/nzdf.cfg");

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
  NZDFServer*m = new NZDFServer(&fname);


  return a.exec();

  
}
