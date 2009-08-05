/**
 * @ingroup icub_primatevision_utils_flow
 * \defgroup icub_primatevision_utils_flow_testflow TestFlow
 *
 * A test application for the Flow class.
 *
 * This module should be called in the following way:\n \b testflow \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/flow/testflow/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <iostream>
#include <qimage.h>
#include <qapplication.h>
#include <ipp.h>

#include <flow.h> 
#include <display.h>
 
using namespace std;
using namespace iCub::contrib::primateVision;


int main(int argc, char *argv[]) 
{

  QApplication *qa = new QApplication( argc, argv );


  int i = 1;
  QImage *qim = new QImage(QString::number(i)+".jpg","JPEG");

  IppiSize srcsize;
  srcsize.width  = qim->width();
  srcsize.height = qim->height();
 


  Flow*f = new Flow(srcsize,1);

  //PREPARE DISPLAY:
  iCub::contrib::primateVision::Display *disp_im  = new iCub::contrib::primateVision::Display(srcsize,srcsize.width,D_8U_NN,"IN");
  iCub::contrib::primateVision::Display *disp_fx = new iCub::contrib::primateVision::Display(srcsize,f->get_psb(),D_8U,"FX");
  iCub::contrib::primateVision::Display *disp_fy = new iCub::contrib::primateVision::Display(srcsize,f->get_psb(),D_8U,"FY");



  //MAIN EVENT LOOP:
  while (1){    

    //process:
    f->proc(qim->bits(),qim->width(),0,0);
  
    //display:
    disp_im->display(qim->bits());
    disp_fx->display(f->get_fx());
    disp_fy->display(f->get_fy());
    printf("%d\n",i);
    i++; if (i>=11){i=1;}

    //load next image:
    qim->load(QString::number(i)+".jpg","JPEG");

    //usleep(30000);
  }



}

