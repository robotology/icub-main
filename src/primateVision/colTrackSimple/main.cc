/**
 * @ingroup icub_primatevision_utils
 * \defgroup icub_primatevision_utils_coltracksimple ColTrackSimple
 *
 * A very simple example colour chrominance tracker implementation using IPP.
 *
 * This module should be called in the following way:\n \b colTrackSimple \b\n
 *\n
 *
 * Ports accessed:\n
 * <ul>
 * <li> /icub/cam/left
 * <li> /icub/cam/right
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /tts/input/left
 * <li> /tts/input/right
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> None.
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/colTrackSimple/main.cc 
 * 
 *//*
 * Copyright (C) 2007 Andrew Dankers
 * 
 */



#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>

//MY INCLUDES
#include <convert_rgb.h>
#include <display.h>

//YARP2 INCLUDES
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>


using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std; 
using namespace iCub::contrib::primateVision;




int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  string fname="col.cfg";

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
  int u1  = prop.findGroup("COL").find("U1").asInt();
  int v1  = prop.findGroup("COL").find("V1").asInt();
  int u2  = prop.findGroup("COL").find("U2").asInt();
  int v2  = prop.findGroup("COL").find("V2").asInt();
  int set = prop.findGroup("COL").find("SET").asInt();
  double thresh1 = prop.findGroup("COL").find("THRESH1").asDouble();
  double thresh2 = prop.findGroup("COL").find("THRESH2").asDouble();


  printf("SEEKING COL1  U:%d V:%d THRESH: %f\n",u1,v1,thresh1);
  printf("SEEKING COL2  U:%d V:%d THRESH: %f\n",u2,v2,thresh2);




  //YARP
  // Initialize network
  Network::init();




  //Video Input:
  BufferedPort<ImageOf<PixelBgr> > pcl;     // Create a ports
  ImageOf<PixelBgr> *imgl;                // Buffered image pointers
  pcl.open("/coltracksimple/input/image/left");      // Give it a name on the network.
  Network::connect("/icub/cam/left" , "/coltracksimple/input/image/left");

 

  //get first RGB images to get width, height:
  imgl = pcl.read(); //blocking buffered
  int width = imgl->width();
  int height = imgl->height();

  printf("\n\nReceived camera dimensions: w:%d, h:%d\n\n",width,height);


  int psb,psb4;
  Ipp8u *colourl = ippiMalloc_8u_C4(width,height,&psb4);
  Ipp8u *yl = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *ul = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *vl = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *out = ippiMalloc_8u_C1(width,height,&psb);


  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;


  //Image conversion classes:
  Convert_RGB *c_rgb2yuv_l = new Convert_RGB(srcsize);

  //Display class:
  iCub::contrib::primateVision::Display* d_out= new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"colTrackSimple");


  //main event loop:
  while(1){

    //get RGB images:
    imgl = pcl.read(); //blocking buffered

    //convert to RGBA:
    ippiCopy_8u_C3AC4R(imgl->getPixelAddress(0,0),width*3,colourl,psb4,srcsize);

    //convert to Y,U,V image channels:
    c_rgb2yuv_l->proc(colourl,psb4);


    ippiCopy_8u_C1R(c_rgb2yuv_l->get_y(),psb,out,psb,srcsize);


    if (set){
      printf("CENTRAL U,V = %d,%d\n", 
	     c_rgb2yuv_l->get_u()[psb*height/2 + width/2],
	     c_rgb2yuv_l->get_v()[psb*height/2 + width/2]);

      out[(height/2)*psb + width/2] = 0;

      //DISPLAY!!!!      
      d_out->display(out);
    }
    else{

      
      for (int y=0;y<height;y++){
	for (int x=0;x<width;x++){
	  if ( sqrt( (c_rgb2yuv_l->get_u()[y*psb + x]-u1) * (c_rgb2yuv_l->get_u()[y*psb + x]-u1) +
		   (c_rgb2yuv_l->get_v()[y*psb + x]-v1) * (c_rgb2yuv_l->get_v()[y*psb + x]-v1) ) 
	     <= thresh1) {
	  out[y*psb + x] = 255;
	}
	else if ( sqrt( (c_rgb2yuv_l->get_u()[y*psb + x]-u2) * (c_rgb2yuv_l->get_u()[y*psb + x]-u2) +
		   (c_rgb2yuv_l->get_v()[y*psb + x]-v2) * (c_rgb2yuv_l->get_v()[y*psb + x]-v2) ) 
	     <= thresh2) {
	  out[y*psb + x] = 0;
	}
      }
    }
    
    //DISPLAY!!!!
    d_out->display(out);

    }




  }



  //never here!  
  Network::fini();
  return a.exec();

}
