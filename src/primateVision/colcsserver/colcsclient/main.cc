/**
 * @ingroup icub_primatevision_colcsserver
 * \defgroup icub_primatevision_colcsserver_colcsclient ColCSClient
 *
 *This endclient module ColCSClient displays output from the ColCSServer, as well as providing sample code showing how to make a useful ColCSServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b colcsclient l \b\n
 *\n where 'l' starts a ColCSClient of the left ColCSServer.
 * Ports accessed:\n
 * <ul>
 * <li> /colcsserverL/output/sal
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /colcsclientL/input/sal
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
 * This file can be edited at src/primateVision/colcsserver/colcsclient/main.cc 
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
#include <qstring.h>

//MY INCLUDES
#include <display.h>
#include <colcsio.h>
 

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
  inPort_s.open("/colcsclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/colcsclient_"+input+"/input/serv_params", "/colcsserver_"+input+"/output/serv_params");
  Network::connect("/colcsserver_"+input+"/output/serv_params", "/colcsclient_"+input+"/input/serv_params");
  BinPortable<ColCSServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  ColCSServerParams csp = server_response.content();
  std::cout << "ColCSServer Probe Response: " << csp.toString() << std::endl;


  int width = csp.width;
  int height = csp.height;
  int psb = csp.psb;




  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;



  BufferedPort<Bottle> inPort;      // Create a ports
  inPort.open("/colcsclient_"+input+"/input/colcs");     // Give it a name on the network.
  Network::connect("/colcsserver_"+input+"/output/colcs" , "/colcsclient_"+input+"/input/colcs");
  Bottle *inBot;
  Ipp8u* c;





  iCub::contrib::primateVision::Display *d_c = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"COLCS CLIENT "+input);

 

  
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
