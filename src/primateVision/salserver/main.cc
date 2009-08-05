/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_salserver SalServer
 *
 *This SalServer module can be configured to connect to any of the YCSServer, ColCSServer, FlowServer, OCSServer modules, receiving saliency input for the left or right camera to combine all saliency cues into a final saliency map.  
 *
 * This module should be called in the following way:\n \b salserver \b -c \a salL.cfg\n
 * where salL.cfg is the config file.  Default configfile in primateVision/config/salL.cfg is used if none specified. Please ensure all configured dependent servers are running before running a SalServer.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /ycsserverL/sal
 * <li> /colcsserverL/sal
 * <li> /ocsserver0/M
 * <li> /ocsserver0/m
 * <li> /ocsserver0/sym
 * <li> /ocsserver0/sal
 * <li> /flowserverL/sal
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /ocsserverL/input/ocs_M
 * <li> /ocsserverL/input/ocs_m
 * <li> /ocsserverL/input/ocs_s
 * <li> /ocsserverL/input/ocs_sal
 * <li> /ocsserverL/input/col
 * <li> /ocsserverL/input/ycs
 * <li> /ocsserverL/input/flow
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /salserverL/output/sal\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/salserver/main.cc 
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
#include "sal.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/salL.cfg");

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
  SalServer*s = new SalServer(&fname);


  return a.exec();

  
}
