/*
 * Copyright (C) 2007 Andrew Dankers
 * 
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <qapplication.h>
#include <q3vbox.h>
#include <qimage.h>
#include <ipp.h>

#include "proc.h"
#include <recio.h>

#define  PI 3.14159265



using namespace iCub::contrib::primateVision;

PROC::PROC(QApplication*a_,world3d*w_)
{

  a=a_;
  world=w_;

  start();

}


PROC::~PROC()
{

}


void PROC::run()
{


  Port inPort_s;
  inPort_s.open("/egosphere2/input/serv_params");     // Give it a name on the network.
  Network::connect("/egosphere2/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/egosphere2/input/serv_params");
  BinPortable<RecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;


  //RIGHT IMS PORT:
  BufferedPort<Bottle> inPort_ry;      // Create a ports
  inPort_ry.open("/egosphere2/input/rec_ry");     // Give it a name on the network.
  Network::connect("/recserver/output/right_yb" , "/egosphere2/input/rec_ry");
  Bottle *inBot_ry;
  Ipp8u* rec_im_ry;


  RecResultParams* rec_res; 



  int width,height,psb;
  width = rsp.width;
  height = rsp.height;
  psb = rsp.psb;

  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;


  QImage *qim = new QImage(width,height,8,256);
  for(unsigned int ui=0;ui<256;ui++) //set to B&W
    {
      qim->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
    }



  //main event loop:
  while(1){


    //GET BARREL RECT RIGHT IM:
    inBot_ry = inPort_ry.read();
    rec_im_ry = (Ipp8u*) inBot_ry->get(0).asBlob();
    rec_res = (RecResultParams*) inBot_ry->get(1).asBlob();


    //set new image & pos:
    ippiCopy_8u_C1R(rec_im_ry,width,
		    qim->bits(),qim->width(),srcsize);
    world->setim(qim,
		 rec_res->deg_rx,
		 rec_res->deg_ry,
		 rec_res->head_r,
		 rec_res->head_p,
		 rec_res->head_y
		 );

    //remote callbacks:
    a->lock();
    world->callback();
    a->unlock();

  }


  
}

