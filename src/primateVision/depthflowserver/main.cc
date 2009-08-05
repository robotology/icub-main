/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_depthflowserver DepthflowServer
 *
 *This DepthflowServer module connects to the RecServer module, receiving the rectified Y (intensity) channel from both the left and right cameras, as well as rectification parameters.  From the epipolar rectified images, it produces 1)an image-frame disparity map that is converted into a 2)mosaic-frame disparity map such that absolute mosaic disparities are known regardless of camera motion; and 3)converts the absolute mosaic disparities into absolute depth measurements, based on the head parameters obtained from the RecServer. By analysing conseccutive disparity and depth maps, it produces 4) an estimate of flow in the depth direction. Finally, it also produces 5) a depth saliancy map according to the proximity of objects withing the overlapping stereo field of view. NB, it does NOT calculate optical flow. use FlowServer for that!
 *
 * This module should be called in the following way:\n \b depthflowserver \b -c \a dfcs.cfg\n
 * where zdf.cfg is the config file.  Default configfile in primateVision/config/zdf.cfg is used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ye
 * <li> /recserver/right_ye
 * <li> /recserver/rec_params
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /depthflowserver/input/left
 * <li> /depthflowserver/input/right
 * <li> /depthflowserver/input/rec_params
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /depthflowserver/output/disp
 * <li> /depthflowserver/output/mdisp
 * <li> /depthflowserver/output/depth
 * <li> /depthflowserver/output/depthflow
 * <li> /depthflowserver/output/depthsal
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/depthflowserver/main.cc 
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
#include "depthflows.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/dfcs.cfg");

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
  DepthflowServer*f = new DepthflowServer(&fname);


  return a.exec();

  
}
