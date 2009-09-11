/**
 * @ingroup icub_primatevision_iorserver
 * \defgroup icub_primatevision_iorserver_iorclient IORClient
 *
 *This endclient module IORClient displays output from the IORServer, as well as providing sample code showing how to make a useful IORServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b iorclient l \b\n
 *\n where 'l' starts a IORClient of the left IORServerL.
 * Ports accessed:\n
 * <ul>
 * <li> /iorserverL/output/ior
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /iorclientL/input/ior
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
 * This file can be edited at src/primateVision/iorserver/iorclient/main.cc 
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
#include <iorio.h>


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
  inPort_s.open("/iorclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/iorclient_"+input+"/input/serv_params", "/iorserver_"+input+"/output/serv_params");
  Network::connect("/iorserver_"+input+"/output/serv_params", "/iorclient_"+input+"/input/serv_params");
  BinPortable<IORServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  IORServerParams fsp = server_response.content();
  std::cout << "IORServer Probe Response: " << fsp.toString() << std::endl;

  int width = fsp.width;
  int height = fsp.height;
  int psb = fsp.psb;

  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;



  BufferedPort<Bottle> inPort_ior;      // Create a port
  inPort_ior.open("/iorclient_"+input+"/input/ior");     // Give it a name on the network.
  Network::connect("/iorserver_"+input+"/output/ior" , "/iorclient_"+input+"/input/ior");
  Bottle *inBot_ior;
  Ipp8u* ior;



  iCub::contrib::primateVision::Display *d_ior = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U,"IOR "+input);

  
  int frame=0;

  while (1){
    inBot_ior = inPort_ior.read();
    if (inBot_ior!=NULL){
      ior = (Ipp8u*) inBot_ior->get(0).asBlob();
      d_ior->display(ior);

      frame++;
      d_ior->save(ior,"ior"+input+QString::number(frame)+".jpg"); 
      
    }
    else{
      printf("No Input\n");
      usleep(1000); //don't blow out port
    }
  }

  //never here!
  
}
