/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_zdfserver ZDFServer
 *
 *This ZDFServer module connects to the RecServer module, recieving the rectified Y (intensity) channel from both the left and right camera and performs brobabalistic zero disparity filtering based upon MRF theory and Graph Cuts optimisation. It effectively segments the object at stereo fixation.  Moreover, the server has been designed to instruct the recserver to move the cameras to keep the object that has been extracted at the zero disparity position. This implements coordinated stereo fixation and tracking and segmentation of arbitrary, deformable objects.  
 * The ZDFServer uses the /utils/multiclass library to optimise the segmentation, based on providing it with a zero disparity probability map constructed using the Rank transform.
 *
 * This module should be called in the following way:\n \b zdfserver \b -c \a zdf.cfg\n
 * where zdf.cfg is the config file.  Default configfile in primateVision/config/zdf.cfg is used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ye
 * <li> /recserver/right_ye
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /zdfserver/input/left
 * <li> /zdfserver/input/right
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /zdfserver/output/zdseg\n
 * <li> /zdfserver/output/fovl\n
 * <li> /zdfserver/output/fovr\n
 * <li> /zdfserver/output/zdprob\n
 * <li> /zdfserver/output/motion_cmds\n
 * <li> /zdfserver/output/yarpimg\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/zdfserver/main.cc 
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
#include "zdf.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/zdf.cfg");

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
  ZDFServer*m = new ZDFServer(&fname);


  return a.exec();

  
}
