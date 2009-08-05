/**
 * @ingroup icub_primatevision_ocsserver
 * \defgroup icub_primatevision_ocsserver_ocsclient OCSClient
 *
 *This endclient module OCSClient displays output from the OCSServer, as well as providing sample code showing how to make a useful OCSServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b ocsclient 0 \b\n
 *\n where '0' starts a OCSClient of OCSServer0.
 * Ports accessed:\n
 * <ul>
 * <li> /ocsserver0/output/M
 * <li> /ocsserver0/output/m
 * <li> /ocsserver0/output/sym
 * <li> /ocsserver0/output/sal
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /ocsclient0/input/M
 * <li> /ocsclient0/input/m
 * <li> /ocsclient0/input/sym
 * <li> /ocsclient0/input/sal
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
 * This file can be edited at src/primateVision/ocsserver/ocsclient/main.cc 
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

//This app is an ocsclient!
#include <ocsio.h>

using namespace iCub::contrib::primateVision;



int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  if (argc!=2){
    printf("usage: %s n \n\n",argv[0]);
    exit(0);
  }
  QString input = argv[1];

  Port inPort_s;
  inPort_s.open("/ocsclient_"+input+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/ocsclient_"+input+"/input/serv_params", "/ocsserver_"+input+"/output/serv_params");
  Network::connect("/ocsserver_"+input+"/output/serv_params", "/ocsclient_"+input+"/input/serv_params");
  BinPortable<OCSServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  OCSServerParams csp = server_response.content();
  std::cout << "OCSServer Probe Response: " << csp.toString() << std::endl;


  int width = csp.width;
  int height = csp.height;
  int psb = csp.psb;




  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;



  BufferedPort<Bottle> inPort_ps;      // Create a ports
  inPort_ps.open("/ocsclient_"+input+"/input/phaseSym");     // Give it a name on the network.
  Network::connect("/ocsserver_"+input+"/output/phaseSym" , "/ocsclient_"+input+"/input/phaseSym");
  Bottle *inBot_ps;
  Ipp8u* ps;

  BufferedPort<Bottle> inPort_m;      // Create a ports
  inPort_m.open("/ocsclient_"+input+"/input/m");     // Give it a name on the network.
  Network::connect("/ocsserver_"+input+"/output/m" , "/ocsclient_"+input+"/input/m");
  Bottle *inBot_m;
  Ipp8u* m;

  BufferedPort<Bottle> inPort_M;      // Create a ports
  inPort_M.open("/ocsclient_"+input+"/input/M");     // Give it a name on the network.
  Network::connect("/ocsserver_"+input+"/output/M" , "/ocsclient_"+input+"/input/M");
  Bottle *inBot_M;
  Ipp8u* M;

  BufferedPort<Bottle> inPort_ocs;      // Create a ports
  inPort_ocs.open("/ocsclient_"+input+"/input/ocs");     // Give it a name on the network.
  Network::connect("/ocsserver_"+input+"/output/ocs" , "/ocsclient_"+input+"/input/ocs");
  Bottle *inBot_ocs;
  Ipp8u* ocs;






  iCub::contrib::primateVision::Display *d_ps = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"PhaseSym"+input);
  iCub::contrib::primateVision::Display *d_m = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"m"+input);
  iCub::contrib::primateVision::Display *d_M = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"M"+input);
  iCub::contrib::primateVision::Display *d_ocs = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U,"OCS"+input);
 


  
  while (1){


    inBot_ocs = inPort_ocs.read(false);
    inBot_M = inPort_M.read(false);
    inBot_m = inPort_m.read(false);
    inBot_ps = inPort_ps.read();


    if (inBot_ps!=NULL){
      d_ps->display((Ipp8u*) inBot_ps->get(0).asBlob());
    }
    
    if (inBot_m!=NULL){
      d_m->display((Ipp8u*) inBot_m->get(0).asBlob());
    }
    
    if (inBot_M!=NULL){
      d_M->display((Ipp8u*) inBot_M->get(0).asBlob());
    }
    
    if (inBot_ocs!=NULL){
      d_ocs->display((Ipp8u*) inBot_ocs->get(0).asBlob());
    }
    


    if (inBot_ps==NULL && inBot_m==NULL && inBot_M ==NULL && inBot_ocs==NULL){
      printf("No Input\n");
      usleep(5000); //dont blow out port
    }
    
    
  }
  
  //never here!
  
}
