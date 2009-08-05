/**
 * @ingroup icub_primatevision_utils_depth
 * \defgroup icub_primatevision_utils_depth_testdepth TestDepth
 *
 * A test application for the Depth class.
 *
 * This module should be called in the following way:\n \b testdepth \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/depth/testdepth/main.cc 
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
#include <depth.h> 

using namespace std;
using namespace iCub::contrib::primateVision;


int main(int argc, char *argv[]) 
{

  QApplication *qa = new QApplication( argc, argv );


  QImage *qiml = new QImage("caml.jpg","JPEG");
  QImage *qimr = new QImage("camr.jpg","JPEG");

  IppiSize srcsize;
  srcsize.width  = qiml->width();
  srcsize.height = qiml->height();
 

//        int filterSize_,
//        int filterCap_,
//        int windowSize_,
//        int minDisparity_,
//        int numDisparities_,
//        int threshold_,
//        int uniqueness_
  Depth*d = new Depth(srcsize,
                      17,20,9,-64,128,25,30);

  //PREPARE DISPLAY:
  iCub::contrib::primateVision::Display *disp_im_l = new iCub::contrib::primateVision::Display(srcsize,srcsize.width,D_8U_NN,"IN L");
  iCub::contrib::primateVision::Display *disp_im_r = new iCub::contrib::primateVision::Display(srcsize,srcsize.width,D_8U_NN,"IN R");
  iCub::contrib::primateVision::Display *disp_disp = new iCub::contrib::primateVision::Display(srcsize,d->get_psb(),D_8U,"DISP");




  int i = 0;
  while (1){    

    d->proc(qiml->bits(),qimr->bits(),qimr->width());
    
    //display:
    disp_im_l->display(qiml->bits());
    disp_im_r->display(qimr->bits());
    disp_disp->display(d->get_disp());
    printf("%d\n",i);
    i++;
  }
   
}

