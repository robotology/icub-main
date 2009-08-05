/**
 * @ingroup icub_primatevision_utils_dog
 * \defgroup icub_primatevision_utils_flow_testdog TestDoG
 *
 * A test application for the DoG class.
 *
 * This module should be called in the following way:\n \b testdog \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/dog/testdog/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <iostream>
#include <display.h>
#include <qimage.h>
#include <qapplication.h>
#include <ipp.h>
#include <math.h> 
#include <dog.h> 

using namespace std;
using namespace iCub::contrib::primateVision;


int main(int argc, char *argv[])
{

  QApplication *qa = new QApplication( argc, argv );


  QImage *qim = new QImage("bike.jpg","JPEG");

  IppiSize srcsize;
  srcsize.width  = qim->width();
  srcsize.height = qim->height();
 


  DoG*d = new DoG(srcsize);

  //PREPARE DISPLAY:
  iCub::contrib::primateVision::Display *disp_im  = new iCub::contrib::primateVision::Display(srcsize,srcsize.width,D_8U_NN,"IN");
  iCub::contrib::primateVision::Display *disp_ad = new iCub::contrib::primateVision::Display(srcsize,d->get_psb(),D_8U,"DOG_ONOFF");
  iCub::contrib::primateVision::Display *disp_on = new iCub::contrib::primateVision::Display(srcsize,d->get_psb(),D_8U,"DOG_ON");
  iCub::contrib::primateVision::Display *disp_off = new iCub::contrib::primateVision::Display(srcsize,d->get_psb(),D_8U,"DOG_OFF");


  d->proc(qim->bits(),qim->width());


  while (1){    
    //display:
    disp_im->display(qim->bits());
    disp_ad->display(d->get_dog_onoff());
    disp_on->display(d->get_dog_on());
    disp_off->display(d->get_dog_off());
  }



}

