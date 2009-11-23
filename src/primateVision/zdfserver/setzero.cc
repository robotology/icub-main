/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_setzero SetZero
 *
 *This endclient module SetZero has been created merely for the convenience of setting the gaze position to the calibrated home position.
 *
 * This module should be called in the following way:\n \b setzero \b\n
 *\n
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/output/rec_params
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /setzero/input/rec_params
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /setzero/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/zdfserver/setzero.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <string>
#include <qstring.h>
#include <iostream>

//MY INCLUDES
#include <recio.h>


using namespace iCub::contrib::primateVision;
 
int main( int argc, char **argv )
{

  QString arg = argv[1];
  QString sim;
  if (arg=="sim"){
    sim = "Sim";
  }


  //CONTACT RECSERVER:
  Port inPort_s;
  inPort_s.open("/recreset/input/serv_params");
  Network::connect("/recreset/input/serv_params", "/recserver"+sim+"/output/serv_params");
  Network::connect("/recserver"+sim+"/output/serv_params", "/recreset/input/serv_params");
  BinPortable<RecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;

  //ACCESS MOTION LIKE THIS!
  Port outPort_mot;
  outPort_mot.open("/recreset/output/mot"); 
  Network::connect("/recreset/output/mot", "/recserver"+sim+"/input/motion");
  Network::connect("/recserver"+sim+"/input/motion", "/recreset/output/mot");
  BinPortable<RecMotionRequest> motion_request;

  //initalise:
  motion_request.content().pix_y  = 0.0;
  motion_request.content().pix_xl = 0.0;
  motion_request.content().pix_xr = 0.0;
  motion_request.content().deg_r = 0.0;
  motion_request.content().relative = false; //gonna send absolute move!
  motion_request.content().suspend = 100; 
  motion_request.content().lockto = NO_LOCK; 
  motion_request.content().unlock = true; 
  outPort_mot.write(motion_request);

}
