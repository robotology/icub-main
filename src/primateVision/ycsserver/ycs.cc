/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "ycs.h"

#include <stdio.h>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>
#include <qstring.h>

//My includes:
#include <centsur.h>  
#include <timing.h>
#include <convert_bitdepth.h>
// THIS APP IS A RECCLIENT!!!
#include <recio.h>


iCub::contrib::primateVision::YCSServer::YCSServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::YCSServer::~YCSServer()
{
  
}



void iCub::contrib::primateVision::YCSServer::run(){

  Property prop;
  prop.fromConfigFile(cfg->c_str());

  int ncsscale = prop.findGroup("YCS").find("NCSSCALE").asInt();
  int input    = prop.findGroup("YCS").find("INPUT").asInt();

  IppiSize srcsize;
  int psb_in;

  QString in_source,rec_name;
  if (input==0){in_source="l";rec_name="left";}
  else{in_source="r";rec_name="right";}

  //IN PORTS:
  Port inPort_s;
  inPort_s.open("/ycsserver_"+in_source+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/ycsserver_"+in_source+"/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/ycsserver_"+in_source+"/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 
  srcsize.width = rsp.width;
  srcsize.height = rsp.height;
  psb_in = rsp.psb;

  BufferedPort<Bottle> inPort_y;      // Create a port
  inPort_y.open("/ycsserver_"+in_source+"/input/rec_y");     // Give it a name on the network.
  Network::connect("/recserver/output/"+rec_name+"_ye" , "/ycsserver_"+in_source+"/input/rec_y");
  Bottle *inBot_y;
  Ipp8u* rec_im_y;


  // OUTPUT PORTS:
  //server params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/ycsserver_"+in_source+"/output/serv_params");
  //image:
  BufferedPort<Bottle> outPort_ycs;
  outPort_ycs.open("/ycsserver_"+in_source+"/output/ycs");



  //PROCESSING CLASS SETUP:
  CentSur*c_y  = new CentSur(srcsize,ncsscale);

  int psb;
  Ipp8u *ycs_out = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);


  // Make replier for server param probes on server port:
  //Set Params:
  YCSServerParams csp;
  csp.width=srcsize.width;
  csp.height=srcsize.height;
  csp.psb=psb;
  //Replier:
  YCSReplyParamProbe server_replier;
  server_replier.reply=csp;
  outPort_s.setReplier(server_replier);




  TCREATE
  


  //MAIN EVENT LOOP:
  while (1){



    TSTART


    //get image:
    inBot_y = inPort_y.read();

    if (inBot_y!=NULL){

      rec_im_y = (Ipp8u*) inBot_y->get(0).asBlob();

      //process:
      c_y->proc_im_8u(rec_im_y,psb_in);
      ippiCopy_8u_C1R(c_y->get_centsur_norm8u(),c_y->get_psb_8u(),ycs_out,psb,srcsize);
      
      //OUTPUT:
      Bottle& tmpBot_ycs = outPort_ycs.prepare();
      tmpBot_ycs.clear();
      tmpBot_ycs.add(Value::makeBlob( ycs_out, psb*srcsize.height));
      tmpBot_ycs.addString("ycs");
      outPort_ycs.write();  // Send it on its way
  
    }
    else{
      printf("No Input\n");
      usleep(5000); //dont blow out port
    } 
  
    TSTOP  

  }
  
  //never here..
  TEND

}
