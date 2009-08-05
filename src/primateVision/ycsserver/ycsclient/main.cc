/**
 * @ingroup icub_primatevision_ycsserver
 * \defgroup icub_primatevision_ycsserver_ycsclient YCSClient
 *
 *This endclient module YCSClient displays output from the YCSServer, as well as providing sample code showing how to make a useful YCSServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b ycsclient l \b\n
 *\n where 'l' starts a YCSClient of the left YCSServer.
 * Ports accessed:\n
 * <ul>
 * <li> /ycsserverL/output/sal
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /ycsclientL/input/sal
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> NONE! It's an endClient!!
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/ycsserver/ycsclient/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <qapplication.h>
#include <qstring.h>

//MY INCLUDES
#include <display.h>
#include <ycsio.h>

using namespace iCub::contrib::primateVision;
 
int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  if (argc!=2){
    printf("usage: %s [l,r]\n\n",argv[0]);
    exit(0);
  }
  QString input = argv[1];

  Port inPort_s;
  inPort_s.open("/ycsclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/ycsclient_"+input+"/input/serv_params", "/ycsserver_"+input+"/output/serv_params");
  Network::connect("/ycsserver_"+input+"/output/serv_params", "/ycsclient_"+input+"/input/serv_params");
  BinPortable<YCSServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  YCSServerParams csp = server_response.content();
  std::cout << "YCSServer Probe Response: " << csp.toString() << std::endl;

  int width = csp.width;
  int height = csp.height;
  int psb = csp.psb;

  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;



  BufferedPort<Bottle> inPort;      // Create a ports
  inPort.open("/ycsclient_"+input+"/input/ycs");     // Give it a name on the network.
  Network::connect("/ycsserver_"+input+"/output/ycs" , "/ycsclient_"+input+"/input/ycs");
  Bottle *inBot;
  Ipp8u* c;



  iCub::contrib::primateVision::Display *d_c = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"YCS CLIENT "+input);

 

  
  while (1){

    inBot = inPort.read();
    if (inBot!=NULL){
      c = (Ipp8u*) inBot->get(0).asBlob();
      d_c->display(c);
    }
    else{
      printf("No Input\n");
      usleep(5000); //dont blow out port
    }
    
  }


  //never here! 
}
