/**
 * @ingroup icub_primatevision_utils_display
 * \defgroup icub_primatevision_utils_display_testdisplay TestDisplay
 *
 * A test application for the Display class.
 *
 * This module should be called in the following way:\n \b testdisplay \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/display/testdisplay/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <qapplication.h>
#include <display.h>
#include <qimage.h>
#include <ipp.h>

using namespace iCub::contrib::primateVision;

int main(int argc, char **argv)
{

  QApplication *a = new QApplication(argc, argv);
  
  //loag image:
  QImage *c = new QImage("mf.jpg","JPEG");

  IppiSize srcsize;
  srcsize.width = c->width();
  srcsize.height = c->height();
  int psb = c->width()*4;   

  printf("%d %d\n", c->width(),c->height());

  iCub::contrib::primateVision::Display *d_c = new iCub::contrib::primateVision::Display(srcsize,psb,D_QIM_RGB,"TESTDISPLAY");


  while(1){ 

    //printf("Main: displaying!\n");

    d_c->display(c);
    
    usleep(10000);
  }
  

  //never here, but clean up and exit
  delete c;
  return a->exec();
}

