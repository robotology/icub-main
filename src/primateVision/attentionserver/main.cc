/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_attentionserver AttnServer
 *  
 *This AttnServer module connects to the SalServer, IORServer, and TSBServer,  receiving saliency, inhibition-of-return, and task-dependent spatial bias data, for the construction of a monocular fixation map. Fixation map peaks are moderated by several mechanisms, and an attentional peak is selected and returned. The AttnServer can initiate attentional saccade by sending motion commands via the RecServer. An AttnClient can be initiated to view AttnServer processing output.
 *
 * This module should be called in the following way:\n \b attnserver \b -c \a attnL.cfg\n
 * where attnL.cfg is the config file.  Default configfile in primateVision/config/attnL.cfg is used if none specified.
 *
 *
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/motion
 * <li> /salserverL/sal
 * <li> /iorserverL/ior
 * <li> /tsbserverL/tsb
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /attnserverL/input/sal
 * <li> /attnserverL/input/ior
 * <li> /attnserverL/input/tsb
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /attnserverL/output/attn
 * <li> /attnserverL/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/attentionserver/main.cc 
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
#include "attn.h"

using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/attnL.cfg");


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
  AttnServer*s = new AttnServer(&fname);


  return a.exec();

  
}
