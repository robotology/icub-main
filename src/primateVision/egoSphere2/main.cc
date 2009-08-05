/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_egosphere2 EgoSphere2
 *
 *This endclient module EgoSphere2 displays output from the RecServer on a sphere surrounding the head. Rectifies projection across any motion in the 6 axes of the head, to align the projection continuously with the real world. Useful for multi-modal sensory integration and wide-field-of-view (WFOV) representations.
 *
 * This module should be called in the following way:\n \b egosphere2 \b\n
 *\n
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/output/left_ye
 * <li> /recserver/output/right_ye
 * <li> /recserver/output/rec_params
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /egosphere2/input/left_ye
 * <li> /egosphere2/input/right_ye
 * <li> /egosphere2/input/rec_params
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
 * This file can be edited at src/primateVision/egosphere2/main.cc 
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
#include <q3vbox.h>
#include <qpushbutton.h>

#include "world3d.h"
#include "proc.h"


using namespace std;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);



  //Create STOP button:
  Q3VBox box;
  box.resize(200, 120);  
  QPushButton quit("Quit", &box);
  QObject::connect(&quit, SIGNAL(clicked()), &a, SLOT(quit()));
  a.setMainWidget(&box);
  box.show();


  //Create 3D World widget in a blank window:
  Q3VBox *mainwin = new Q3VBox;
  mainwin->setCaption( "EgoSphere World" );
  world3d* world = new world3d(&a,mainwin);

  usleep(100000);

  
  PROC *p = new PROC(&a,world);
  
  
  return a.exec();

}
