/**
 * @ingroup icub_primatevision_utils
 * \defgroup icub_primatevision_utils_temptracksimple TempTrackSimple
 *
 * A very simple example template tracker implementation using IPP.
 *
 * This module should be called in the following way:\n \b tempTrackSimple \b\n
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
 * This file can be edited at src/primateVision/tempTrackSimple/main.cc 
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
#include <ipp.h>

//MY INCLUDES
#include <convert_rgb.h>
#include <display.h>
#include <mydraw.h>
#include <ncurses.h>

//YARP2 INCLUDES
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>


using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std; 
using namespace iCub::contrib::primateVision;

#define SIZE 32

#define BLACK 0.01
#define GREY  0.3
#define WHITE 0.99


int main( int argc, char **argv )
{

  QApplication a(argc, argv);



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


  int psb,psb4,psb_t;
  Ipp8u *colourl = ippiMalloc_8u_C4(width,height,&psb4);
  Ipp8u *tmp = ippiMalloc_8u_C1(SIZE,SIZE,&psb_t);
  Ipp8u *out = ippiMalloc_8u_C1(width,height,&psb);


  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;
  IppiSize tmpsize;
  tmpsize.width = SIZE;
  tmpsize.height = SIZE;


  //Image conversion classes:
  Convert_RGB *c_rgb2yuv_l = new Convert_RGB(srcsize);

  //Display class:
  iCub::contrib::primateVision::Display* d_out= new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"tmpTrackSimple");
  iCub::contrib::primateVision::Display* d_tmp= new iCub::contrib::primateVision::Display(tmpsize,psb_t,D_8U_NN,"tmpTrackSimple");


  char ch;
  
  WINDOW *win;
  cbreak();
  initscr();
  timeout(0);
  nodelay(win,true);
  





  int psb_res;
  IppiSize rsize;
  rsize.width  = srcsize.width;
  rsize.height = srcsize.height;

  Ipp32f *res = ippiMalloc_32f_C1(rsize.width,rsize.height,&psb_res);
  int sx,sy;
  Ipp32f max;





  printw("press SPACE to update tracked template! Ctrl-C to exit :) \n");

  
  //main event loop:
  while(1){
    

    //get RGB images:
    imgl = pcl.read(); //blocking buffered
    
    //convert to RGBA:
    ippiCopy_8u_C3AC4R(imgl->getPixelAddress(0,0),width*3,colourl,psb4,srcsize);
    
    //convert to Y,U,V image channels:
    c_rgb2yuv_l->proc(colourl,psb4);
    
    
    ippiCopy_8u_C1R(c_rgb2yuv_l->get_y(),psb,out,psb,srcsize);
    
    

    //NCC: find the template coords:
    //search for template in centre of current left fov in current right fov, within isize:                        
    ippiCrossCorrSame_NormLevel_8u32f_C1R(c_rgb2yuv_l->get_y(),                           
                                           psb,
					   srcsize,
                                           tmp,
                                           psb_t,
					   tmpsize,
                                           res, 
					   psb_res);

    ippiMaxIndx_32f_C1R(res,psb_res,rsize,&max,&sx,&sy);
 
    
    

    
    //SPACE UPDATES TEMPLATE:
    
    ch = getch();

    if (ch == ' '){
      
      ippiCopy_8u_C1R(&c_rgb2yuv_l->get_y()[width/2-SIZE/2 + (height/2-SIZE/2)*psb],psb, tmp,psb_t, tmpsize);
      
      
      d_tmp->display(tmp);
      
    }    
    



    //DISPLAY!!!!      
    MyDrawLine(out,psb,srcsize,width/2,height/2,BLACK);
    //target
    MyDrawLine(out,psb,srcsize,sx,sy,WHITE);
    d_out->display(out);
    



    
  }
  



  //never here!  
  endwin();
  Network::fini();
  return a.exec();
  
}
