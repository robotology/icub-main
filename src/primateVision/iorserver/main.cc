/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_iorserver IORServer
 *  
 *This IORServer module connects to the RecServer, receiving rectification parameters, and to the FLowServer, receiving scene flow data. It applies a Gaussian inhibition-of-return (IOR) kernel to the centre of the fovea, each frame, in mosaic space.  The IOR mosaic is acucmulated over subsequent frames.  All IOR is decayed over time. Rich flow data obtained from the FlowServer is used to propagate IOR according to the motion of objects in the scene, regardless of the morion of the cameras.  Propagated flow is spread and decayed in line with positional uncertainty. In this manner, objects that have been attended recieve IOR, but if they move, they take the accumulated IOR with them, regardless of camera motion, until it decays away to negligible levels.

That is, if the cameras move, or an object moves, it is still the same objected that was previously attended, and any IOR associated with that object sticks to that object despite its motion, or any camera motion! This has been termed 'Active-dynamic IOR', and is similar to the notion of 'efference copy' observed in primates.
 *
 * This module should be called in the following way:\n \b iorserver \b -c \a iorL.cfg\n
 * where iorL.cfg is the config file.  Default configfile in primateVision/config/iorL.cfg is used if none specified.
 *
 *
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/rec_params
 * <li> /flowserverL/flowx
 * <li> /flowserverL/flowy
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /iorserverL/input/flowx
 * <li> /iorserverL/input/flowy
 * <li> /iorserverL/input/rec_params
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /iorserverL/output/ior\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/iorserver/main.cc 
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
#include "ior.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/iorL.cfg");


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
  IORServer*ior = new IORServer(&fname);


  return a.exec();

  
}
