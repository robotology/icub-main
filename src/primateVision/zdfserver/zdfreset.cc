/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_zdfreset ZDFReset
 *
 *This endclient module ZDFReset has been created merely for the convenience of setting the gaze position to a position convenient for testing/initialising the zero desparity servers.  It places the stereo fixation point about 1m infront of the head.  This is useful as the head calibration procedure usually leaves the gaze fixated at infinity.
 *
 * This module should be called in the following way:\n \b zdfreset \b\n
 *\n
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/output/rec_params
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /zdfreset/input/rec_params
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /zdfreset/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/zdfserver/zdfreset.cc 
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
  inPort_s.open("/zdfreset/input/serv_params");     // Give it a name on the network.
  Network::connect("/zdfreset/input/serv_params", "/recserver"+sim+"/output/serv_params");
  Network::connect("/recserver"+sim+"/output/serv_params", "/zdfreset/input/serv_params");
  BinPortable<RecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;

  //ACCESS MOTION LIKE THIS!
  Port outPort_mot;
  outPort_mot.open("/zdfreset/output/mot");     // Give it a name on the network.
  Network::connect("/zdfreset/output/mot", "/recserver"+sim+"/input/motion");
  Network::connect("/recserver"+sim+"/input/motion", "/zdfreset/output/mot");
  BinPortable<RecMotionRequest> motion_request;

  //initalise:
  motion_request.content().pix_y  = 0.0;
  motion_request.content().pix_xl = 20.0;
  motion_request.content().pix_xr = -20.0;
  motion_request.content().deg_r = 0.0;
  motion_request.content().relative = false; //gonna send absolute move!
  motion_request.content().suspend = 100;
  motion_request.content().lockto = NO_LOCK;
  motion_request.content().unlock = true;
  outPort_mot.write(motion_request);

}
