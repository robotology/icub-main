/**
 * @ingroup icub_primatevision_utils_mutlicalss
 * \defgroup icub_primatevision_utils_multiclass_testmulticlass TestMultiClass
 *
 * A test application for the MultiClass class. Uses config file 'multiclass.cfg' located in the testmulticlass src directory.
 *
 * This module should be called in the following way:\n \b testmulticlass \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/multicalss/testmulticlass/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <string>
#include <stdio.h>
#include <iostream>
#include <qimage.h>
#include <qapplication.h>
#include <ipp.h>
#include <math.h> 
//MY INCLUDES:
#include <display.h>
#include <convert_rgb.h>
#include <multiclass.h> 
#include <timing.h> 
#include <convert_bitdepth.h>

#include <yarp/os/Property.h>

using namespace yarp::os;
using namespace std;
using namespace iCub::contrib::primateVision;

int main(int argc, char *argv[])
{

  QApplication *qa = new QApplication( argc, argv );


  //get config file parameters:
  string fname=string("multiclass.cfg");

  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
      exit(0);
    }
  }  



  Property prop;
  prop.fromConfigFile(fname.c_str());
  struct MultiClass::Parameters params;

  params.iter_max = prop.findGroup("MRF").find("MAX_ITERATIONS").asInt();
  params.randomize_every_iteration = prop.findGroup("MRF").find("RANDOMIZE_EVERY_ITER").asInt();
  
  params.smoothness_penalty = prop.findGroup("MRF").find("SMOOTHNESS_PENALTY").asInt();
  params.smoothness_3sigmaon2 = prop.findGroup("MRF").find("SMOOTHNESS_3SIGMAON2").asInt();
  params.data_penalty = prop.findGroup("MRF").find("DATA_PENALTY").asInt();


  QImage *qim = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/in_g.jpg","JPEG");
  QImage *qp1 = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/arms.jpg","JPEG");
  QImage *qp2 = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/chest.jpg","JPEG");
  QImage *qp3 = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/feet.jpg","JPEG");
  QImage *qp4 = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/hands.jpg","JPEG");
  QImage *qp5 = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/head.jpg","JPEG");
  QImage *qp6 = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/legs.jpg","JPEG");
  QImage *qpother = new QImage("/media/Data/Data/Doug/new/probmaps/jpg/background.jpg","JPEG");


  printf("%d,%d\n",qim->width(),qim->height());
  IppiSize isize;
  isize.width  = qim->width();
  isize.height = qim->height();


  int nclasses=7; //1-6,other.



  MultiClass *m = new MultiClass(isize,qim->width(),nclasses,&params);


  //Prepare:  
  Ipp8u**p_pr = (Ipp8u**) malloc(nclasses*sizeof(Ipp8u*));
  p_pr[0] = qpother->bits();
  p_pr[1] = qp1->bits();
  p_pr[2] = qp2->bits();
  p_pr[3] = qp3->bits();
  p_pr[4] = qp4->bits();
  p_pr[5] = qp5->bits();
  p_pr[6] = qp6->bits();
  
#if 1
  //normalise classes:
  for (int y=0;y<qim->height();y++){
    for (int x=0;x<qim->width();x++){
      p_pr[0][y*qim->width()+x] =   p_pr[0][y*qim->width()+x]/3 + 255/25; //other
      p_pr[1][y*qim->width()+x] = 2*p_pr[1][y*qim->width()+x]/3 + 255/4; //arm
      p_pr[2][y*qim->width()+x] = 2*p_pr[2][y*qim->width()+x]/3 + 255/6; //chest
      p_pr[3][y*qim->width()+x] = 2*p_pr[3][y*qim->width()+x]/3 + 255/6; //feet
      p_pr[4][y*qim->width()+x] = 2*p_pr[4][y*qim->width()+x]/3 + 255/4; //hands
      p_pr[5][y*qim->width()+x] = 2*p_pr[5][y*qim->width()+x]/3 + 255/5; //head
      p_pr[6][y*qim->width()+x] = 2*p_pr[6][y*qim->width()+x]/3 + 255/6; //legs
    }
  }
#endif  



  //Refine:
TCREATE
TSTART
  printf("Doing MRF MULTICLASS Refinement...\n");
  m->proc(qim->bits(),p_pr);         
  printf("Done.\n");
TSTOP

  //DISPLAY RESULTS:
  iCub::contrib::primateVision::Display*disp_1 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"IN");
  iCub::contrib::primateVision::Display*disp_2 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"P1");
  iCub::contrib::primateVision::Display*disp_3 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"P2");
  iCub::contrib::primateVision::Display*disp_4 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"P3");
  iCub::contrib::primateVision::Display*disp_5 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"P4");
  iCub::contrib::primateVision::Display*disp_6 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"P5");
  iCub::contrib::primateVision::Display*disp_7 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"P6");
  iCub::contrib::primateVision::Display*disp_8 = new iCub::contrib::primateVision::Display(isize,qim->width(),D_QIM_G,"OTHER");
  iCub::contrib::primateVision::Display*disp_9 = new iCub::contrib::primateVision::Display(isize,m->get_psb(),D_8U,"OUT");

 

  //DISPLAY LOOP:
  while (1){
    disp_1->display(qim);
    disp_2->display(qp1);
    disp_3->display(qp2);
    disp_4->display(qp3);
    disp_5->display(qp4);
    disp_6->display(qp5);
    disp_7->display(qp6);
    disp_8->display(qpother);
    disp_9->display(m->get_class());
    disp_9->save(m->get_class(),"refined.jpg");
    usleep(50000);
  }


TEND


}

