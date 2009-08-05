/**
 * @ingroup icub_primatevision_attentionserver
 * \defgroup icub_primatevision_attentionserver_attentionclient AttnClient
 *
 *This endclient module AttnClient displays output from the AttnServer and initiates attentional saccades to the desired peaks via motion requests sent to the RecServer. It also provides sample code showing how to make a useful AttnServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b attnclient l \b\n
 *\n where 'l' starts a AttnClient of AttnServerL.
 * Ports accessed:\n
 * <ul>
 * <li> /attnserverL/output/params
 * <li> /attnserverL/output/attn
 * <li> /recserver/input/motion
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /attnclientL/input/params
 * <li> /attnclientL/input/attn
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /attnclientL/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/attentionserver/attentionclient/main.cc 
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
#include <mydraw.h>
//client of:
#include <attnio.h>


#define BLACK 0.01
#define GREY  0.3
#define WHITE 0.99


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
  inPort_s.open("/attnclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/attnclient_"+input+"/input/serv_params", "/attnserver_"+input+"/output/serv_params");
  Network::connect("/attnserver_"+input+"/output/serv_params", "/attnclient_"+input+"/input/serv_params");
  BinPortable<AttnServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  AttnServerParams ssp = server_response.content();
  std::cout << "AttnServer Probe Response: " << ssp.toString() << std::endl;


  IppiSize srcsize;
  srcsize.width = ssp.width;
  srcsize.height = ssp.height;
  int psb = ssp.psb;


  BufferedPort<Bottle> inPort_y;      // Create a ports
  inPort_y.open("/attnclient_"+input+"/input/attn");     // Give it a name on the network.
  Network::connect("/attnserver_"+input+"/output/attn" , "/attnclient_"+input+"/input/attn");
  Bottle *inBot_y;
  Ipp8u* y;
  int fx,fy;



  iCub::contrib::primateVision::Display *d_y = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"ATTN "+input);

  


  while (1){

    inBot_y = inPort_y.read();
    if (inBot_y!=NULL){

      y = (Ipp8u*) inBot_y->get(0).asBlob();
      fx = (int) inBot_y->get(1).asInt();
      fy = (int) inBot_y->get(2).asInt();
      

      MyDrawLine(y,psb,srcsize,fx,fy,WHITE);
      MyDrawLine(y,psb,srcsize,srcsize.width/2,srcsize.height/2,BLACK);

      d_y->display(y);


    }
    else{
      usleep(5000);//don't blow out port
    }

  }

  //never here!
  
}
