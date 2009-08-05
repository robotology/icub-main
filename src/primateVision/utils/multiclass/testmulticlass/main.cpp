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

#include <yarp/os/Property.h>

using namespace yarp::os;
using namespace std;
using namespace iCub::contrib::primateVision;

int main(int argc, char *argv[])
{

  QApplication *qa = new QApplication( argc, argv );


  //get config file parameters:
  string fname=string(getenv("ICUB_DIR"))+string("/../iCub/src/primateVision/utils/multiclass/testmulticlass/multiclass.cfg");

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


  int psb_i,psb_i_4;
  QImage *qim = new QImage((string(getenv("ICUB_DIR"))+string("/../iCub/src/primateVision/utils/multiclass/testmulticlass/test.jpg")).c_str(),"JPEG");
  QImage *qp1 = new QImage((string(getenv("ICUB_DIR"))+string("/../iCub/src/primateVision/utils/multiclass/testmulticlass/p1.jpg")).c_str(),"JPEG");
  QImage *qp2 = new QImage((string(getenv("ICUB_DIR"))+string("/../iCub/src/primateVision/utils/multiclass/testmulticlass/p2.jpg")).c_str(),"JPEG");
  QImage *qpother = new QImage((string(getenv("ICUB_DIR"))+string("/../iCub/src/primateVision/utils/multiclass/testmulticlass/pother.jpg")).c_str(),"JPEG");
  printf("%d,%d\n",qim->width(),qim->height());
  IppiSize isize;
  isize.width  = qim->width();
  isize.height = qim->height();

  Ipp8u *edge_map = ippiMalloc_8u_C1(isize.width,isize.height,&psb_i);
  Ipp8u *in       = ippiMalloc_8u_C4(isize.width,isize.height,&psb_i_4);

  int nclasses=3; //class a,b,other.


  MultiClass *m = new MultiClass(isize,psb_i,nclasses,&params);

  Convert_RGB *ci = new Convert_RGB(isize);
  ippiCopy_8u_C4R(qim->bits(),qim->width()*4,in,psb_i_4,isize);

 

  //get y,u,v:
  printf("CONVERTING TO YUV\n");
  ci->proc(in,psb_i_4);


  //DO MRF REFINEMENT:  
  Ipp8u**p_pr = (Ipp8u**) malloc(3*sizeof(Ipp8u*));
  p_pr[0] = qpother->bits();
  p_pr[1] = qp1->bits();
  p_pr[2] = qp2->bits();

  printf("Doing MRF MULTICLASS Refinement...\n");
  m->proc(ci->get_y(),p_pr);       
  printf("Done.\n");


  //DISPLAY RESULTS:
  iCub::contrib::primateVision::Display*disp_1 = new iCub::contrib::primateVision::Display(isize,psb_i_4,D_QIM_RGB,"IN");
  iCub::contrib::primateVision::Display*disp_2 = new iCub::contrib::primateVision::Display(isize,psb_i,D_QIM_G,"P1");
  iCub::contrib::primateVision::Display*disp_3 = new iCub::contrib::primateVision::Display(isize,psb_i,D_QIM_G,"P2");
  iCub::contrib::primateVision::Display*disp_4 = new iCub::contrib::primateVision::Display(isize,psb_i,D_QIM_G,"POTHER");
  iCub::contrib::primateVision::Display*disp_5 = new iCub::contrib::primateVision::Display(isize,psb_i,D_8U,"OUT");

 

  //DISPLAY LOOP:
  while (1){
    disp_1->display(qim);
    disp_2->display(qp1);
    disp_3->display(qp2);
    disp_4->display(qpother);
    disp_5->display(m->get_class());
    usleep(50000);
  }

}

