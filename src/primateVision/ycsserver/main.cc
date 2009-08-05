/**
 * @ingroup icub_primatevision
 * \defgroup icub_primatevision_ycsserver YCSServer
 *
 *This YCSServer module connects to the RecServer module, receiving the rectified Y (intensity) channel from either the left or right camera and performs centre-surround processing and construction of an intensity uniqueness map (colour saliency) via a difference-of-Gaussian pyramid filter bank.  
 *
 * This module should be called in the following way:\n \b ycsserver \b -c \a ycsL.cfg\n
 * where ycsL.cfg is the config file.  Default configfile in primateVision/config/ycsL.cfg is used if none specified.
 *\n
 
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/left_ye
 * </ul>
 *
 * Input ports:\n
 * <ul>
 * <li> /ycsserverL/input/image
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /ycsserverL/output/ycs\n
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/ycsserver/main.cc 
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
#include "ycs.h"


using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/ycsL.cfg");


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
  YCSServer*ycs = new YCSServer(&fname);

  
  return a.exec();

}
