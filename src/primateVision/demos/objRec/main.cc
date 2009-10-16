/**
 * @ingroup icub_primatevision_demos
 * \defgroup icub_primatevision_demos objrec
 *
 * Client of zdfserver.  Classifies ZDF DOG output. 
 * Outputs localised classifications (primarily for the objMan module).
 *
 */ 

/*
 * Copyright (C) 2003-2009 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include "objRec.h"


using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/objRec.cfg");


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
  ObjRecServer*o = new ObjRecServer(&fname);

  return a.exec();

}

