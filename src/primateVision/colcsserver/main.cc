/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_colcsserver ColCSServer
 *
 *This ColCSServer module connects to the RecServer module, receiving rectified U and V chrominance channels from either the left or right camera and performs centre-surround processing and construction of a colour chrominance uniqueness map (colour saliency) via a difference-of-Gaussian pyramid filter bank.  U and V colour chrominances are orthogonal colour axes independent of intensity; in this manner, they are similar to r-g and b-y colour opponent channels used to model the output of retinal ganglions (similar to a slight rotation of the axes).
 *
 * This module should be called in the following way:\n \b colcsserver \b -c \a colcsL.cfg\n
 * where colcsL.cfg is the config file.  Default configfile in primateVision/config/colcsL.cfg is used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ue
 * <li> /recserver/left_ve
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /colcsserverL/input/image
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /colcsserverL/output/colcs\n
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
#include <string>
#include <iostream>
#include <qapplication.h>
#include <stdlib.h>

//MY INCLUDES
#include "colcs.h"


using namespace std;
using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/colcsL.cfg");

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
  ColCSServer*ccs = new ColCSServer(&fname);

  
  return a.exec();

}
