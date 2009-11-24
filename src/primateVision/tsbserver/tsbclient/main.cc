/**
 * @ingroup icub_primatevision_tsbserver
 * \defgroup icub_primatevision_tsbserver_tsbclient TSBClient
 *
 *This endclient module TSBClient displays output from the TSBServer. It also provides sample code showing how to make a useful TSBServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b tsbclient l\b\n
 *\n where 'l' creates a TSBClient of the left TSBServerL
 * Ports accessed:\n
 * <ul>
 * <li> /tsbserverL/output/tsb
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /tsbclientL/input/tsb
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
 * This file can be edited at src/primateVision/tsbserver/tsbclient/main.cc 
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
#include <tsbio.h>


using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

  if (argc!=2){
    printf("usage: %s [l,r]\n\n",argv[0]);
    exit(0);
  }
  QString input = argv[1];

 
  Port inPort_s;
  inPort_s.open("/tsbclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/tsbclient_"+input+"/input/serv_params", "/tsbserver_"+input+"/output/serv_params");
  Network::connect("/tsbserver_"+input+"/output/serv_params", "/tsbclient_"+input+"/input/serv_params");
  BinPortable<TSBServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  TSBServerParams fsp = server_response.content();
  std::cout << "TSBServer Probe Response: " << fsp.toString() << std::endl;


  int width = fsp.width;
  int height = fsp.height;
  int psb = fsp.psb;

  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;



  BufferedPort<Bottle> inPort_y;      // Create a ports
  inPort_y.open("/tsbclient_"+input+"/input/tsb");     // Give it a name on the network.
  Network::connect("/tsbserver_"+input+"/output/tsb" , "/tsbclient_"+input+"/input/tsb");
  Bottle *inBot_y;
  Ipp8u* y;


  iCub::contrib::primateVision::Display *d_y = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"TSB "+input);

  
  int frame=0;

  while (1){

    inBot_y = inPort_y.read();
    if (inBot_y!=NULL){
      y = (Ipp8u*) inBot_y->get(0).asBlob();
      d_y->display(y);

   //   frame++;
    //  d_y->save(y,"tsb"+input+QString::number(frame)+".jpg"); 
      
    }
    
    else{
      printf("No Input\n");
      usleep(1000);//don't blow out port
    }

  }

  //never here!  
}
