/**
 * @ingroup icub_primatevision_salserver
 * \defgroup icub_primatevision_salserver_salclient SalClient
 *
 *This endclient module SalClient displays output from the SalServer. It also provides sample code showing how to make a useful SalServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b salclient l\b\n
 *\n where 'l' creates a SalClient of the left SalServerL
 * Ports accessed:\n
 * <ul>
 * <li> /salserverL/output/sal
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /salclientL/input/sal
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
 * This file can be edited at src/primateVision/salserver/salclient/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */ 


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
//MY INCLUDES
#include <display.h>
#include <salio.h>


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
  inPort_s.open("/salclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/salclient_"+input+"/input/serv_params", "/salserver_"+input+"/output/serv_params");
  Network::connect("/salserver_"+input+"/output/serv_params", "/salclient_"+input+"/input/serv_params");
  BinPortable<SalServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  SalServerParams ssp = server_response.content();
  std::cout << "SalServer Probe Response: " << ssp.toString() << std::endl;


  int width = ssp.width;
  int height = ssp.height;
  int psb = ssp.psb;

  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;



  BufferedPort<Bottle> inPort;      // Create a ports
  inPort.open("/salclient_"+input+"/input/sal");     // Give it a name on the network.
  Network::connect("/salserver_"+input+"/output/sal" , "/salclient_"+input+"/input/sal");
  Bottle *inBot;
  Ipp8u* s;


  iCub::contrib::primateVision::Display *d_s = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"SAL "+input);

  
  while (1){
    inBot = inPort.read();
    if(inBot!=NULL){
      s = (Ipp8u*) inBot->get(0).asBlob();
      d_s->display(s);
    }
    else{
      printf("No Input\n");
      usleep(1000); //don't blow out port
    }

  }

  //never here!
  
}
