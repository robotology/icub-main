/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_tsbserver TSBServer
 *  
 *This TSBServer module connects to the RecServer, receiving rectification parameters to construct a mosaic-space Task-dependent Spatial Bias (TSB) map for subsequent modulation of mosaic-space saliency. 
 *
 * This module should be called in the following way:\n \b tsbserver \b -c \a tsbL.cfg\n
 * where tsbL.cfg is the config file.  Default configfile in primateVision/config/tsbL.cfg is used if none specified.
 *
 *
 *Currently, a radial mosaic space bias is implemented that can be used to bias the gaze to remaining on the task workspace immediately infront of the robot.
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/rec_params
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /tsbserverL/input/rec_params
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /tsbserverL/output/tsb\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/tsbserver/main.cc 
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
#include "tsb.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/tsbL.cfg");

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
  TSBServer*t = new TSBServer(&fname);


  return a.exec();

  
}
