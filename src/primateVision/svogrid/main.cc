/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_svogrid SVOGrid
 *
 *This endclient module SVOGrid implements a Bayesian probabalistic space-variant occupancy grid designed specifically for active stereo vision. 3D display output of visual surfaces, and their motion, with texture rendering in realtime using OpenGL.
 *
 * This module should be called in the following way:\n \b svogrid \b\n
 *\n
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/output/left_ye
 * <li> /recserver/output/right_ye
 * <li> /recserver/output/left_ue
 * <li> /recserver/output/right_ue
 * <li> /recserver/output/left_ve
 * <li> /recserver/output/right_ve
 * <li> /recserver/output/rec_params
 * <li> /flowserver/output/flowx
 * <li> /flowserver/output/flowy
 * <li> /depthflowserver/output/depth
 * <li> /depthflowserver/output/depthflow
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /svogrid/input/left_ye
 * <li> /svogrid/input/right_ye
 * <li> /svogrid/input/left_ue
 * <li> /svogrid/input/right_ue
 * <li> /svogrid/input/left_ve
 * <li> /svogrid/input/right_ve
 * <li> /svogrid/input/rec_params
 * <li> /svogrid/input/flowx
 * <li> /svogrid/input/flowy
 * <li> /svogrid/input/depth
 * <li> /svogrid/input/depthflow
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> NONE! It's an endClient!!
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/svogrid/main.cc 
 * 
 */


/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <qapplication.h>
#include <q3vbox.h>
#include <qpushbutton.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

#include "world3d.h"
#include "svogrid.h"


using namespace std;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);


  string fname=string(getenv("ICUB_DIR"))+string("/src/primateVision/config/svogrid.cfg");

  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
      exit(0);
    }
  }




  //Create STOP button:
  Q3VBox box;
  box.resize(200, 120);  
  QPushButton quit("Quit", &box);
  QObject::connect(&quit, SIGNAL(clicked()), &a, SLOT(quit()));
  a.setMainWidget(&box);
  box.show();


  //Create 3D World widget in a blank window:
  Q3VBox *mainwin = new Q3VBox;
  mainwin->setCaption( "SVOGrid World" );
  
  SVOGRID*svogrid = new SVOGRID(&a,&fname);
  world3d*world   = new world3d(&a,mainwin);


  //Give the display world the data:
  svogrid->reg_world(world);
  world->reg_svogrid(svogrid);


  
  
  return a.exec();

}
