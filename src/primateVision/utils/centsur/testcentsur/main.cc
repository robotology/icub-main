/**
 * @ingroup icub_primatevision_utils_centsur
 * \defgroup icub_primatevision_utils_centsur_testcentsur TestCentsur
 *
 * A test application for the CentSur class.
 *
 * This module should be called in the following way:\n \b testcentsur \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/centsur/testcentsur/main.cc 
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
#include <math.h> 

#include <display.h>
#include <centsur.h> 
#include <timing.h> 

using namespace std;
using namespace iCub::contrib::primateVision;


int main(int argc, char *argv[])
{

  QApplication *qa = new QApplication( argc, argv );


  QImage *qim = new QImage("bike.jpg","JPEG");
  QImage *qim_inv = new QImage("bike_invert.jpg","JPEG");

  IppiSize srcsize;
  srcsize.width  = qim->width();
  srcsize.height = qim->height();
 


  CentSur*d = new CentSur(srcsize,5);
  CentSur*d_i = new CentSur(srcsize,5);

  //PREPARE DISPLAY:
  iCub::contrib::primateVision::Display *disp_im  = new iCub::contrib::primateVision::Display(srcsize,srcsize.width,D_8U_NN,"IN");
  iCub::contrib::primateVision::Display *disp_im_i  = new iCub::contrib::primateVision::Display(srcsize,srcsize.width,D_8U_NN,"IN_INVERT");
  iCub::contrib::primateVision::Display *disp_out = new iCub::contrib::primateVision::Display(srcsize,d->get_psb_32f(),D_32F,"CENTSUR(IN)");
  iCub::contrib::primateVision::Display *disp_out_i = new iCub::contrib::primateVision::Display(srcsize,d_i->get_psb_32f(),D_32F,"CENTSUR(IN_INVERT)");


  TCREATE

  while (1){    


    TSTART //just time one!
      d->proc_im_8u(qim->bits(),qim->width());
    TSTOP

      d_i->proc_im_8u(qim_inv->bits(),qim->width());


    //display:
    disp_im->display(qim->bits());
    disp_out->display(d->get_centsur_32f());
    //disp_out->save(d->get_centsur_32f(),"cs.jpg");
    disp_im_i->display(qim_inv->bits());
    disp_out_i->display(d_i->get_centsur_32f());

  }

  //never here..
  TEND

}

