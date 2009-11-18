/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "colcs.h"

#include <stdio.h>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>
#include <qstring.h>

//My includes
#include <centsur.h>  
#include <timing.h>  
#include <convert_bitdepth.h>
// THIS APP IS A RECCLIENT!!!
#include <recio.h>

#include <yarp/os/Property.h>



iCub::contrib::primateVision::ColCSServer::ColCSServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::ColCSServer::~ColCSServer()
{
  
}



void iCub::contrib::primateVision::ColCSServer::run(){

  Property prop;
  prop.fromConfigFile(cfg->c_str());
  int ncsscale = prop.findGroup("COLCS").find("NCSSCALE").asInt();
  int input    = prop.findGroup("COLCS").find("INPUT").asInt();
  yarp::os::ConstString temp_port    = prop.findGroup("COLCS").find("INPORT").asString();
  QString in_port =  temp_port.c_str();


  QString in_source,rec_name;
  if (input==0){in_source="l";rec_name="left";}
  else{in_source="r";rec_name="right";}

  //IN PORTS:
  Port inPort_s;
  inPort_s.open("/colcsserver_"+in_source+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/colcsserver_"+in_source+"/input/serv_params", in_port + "/output/serv_params");
  Network::connect(in_port + "/output/serv_params", "/colcsserver_"+in_source+"/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 
  IppiSize srcsize;
  srcsize.width = rsp.width;
  srcsize.height = rsp.height;
  int psb_in = rsp.psb;


  BufferedPort<Bottle> inPort_u;      // Create a port
  inPort_u.open("/colcsserver_"+in_source+"/input/rec_u");     // Give it a name on the network.
  Network::connect(in_port + "/output/"+rec_name+"_ue" , "/colcsserver_"+in_source+"/input/rec_u");
  Bottle *inBot_u;
  Ipp8u* rec_im_u;

  BufferedPort<Bottle> inPort_v;      // Create a port
  inPort_v.open("/colcsserver_"+in_source+"/input/rec_v");     // Give it a name on the network.
  Network::connect(in_port + "/output/"+rec_name+"_ve" , "/colcsserver_"+in_source+"/input/rec_v");
  Bottle *inBot_v;
  Ipp8u* rec_im_v;





  // OUTPUT PORTS:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/colcsserver_"+in_source+"/output/serv_params");

  //output im:
  BufferedPort<Bottle> outPort_colcs;
  outPort_colcs.open("/colcsserver_"+in_source+"/output/colcs");




  //PROCESSING CLASS SETUP:
  CentSur*c_u  = new CentSur(srcsize,ncsscale);
  CentSur*c_v  = new CentSur(srcsize,ncsscale);

  int psb,psb_32f;
  Ipp32f *cs_tot_32f = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp8u *cs_tot_8u = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);


  // Make replier for server param probes on server port:
  //Set Params:
  ColCSServerParams csp;
  csp.width=srcsize.width;
  csp.height=srcsize.height;
  csp.psb=psb;
  //Replier:
  ColCSReplyParamProbe server_replier;
  server_replier.reply=csp;
  outPort_s.setReplier(server_replier);



  bool got_u=false,got_v=false;



  TCREATE

  //MAIN EVENT LOOP:
  while (1){


    TSTART


    inBot_v = inPort_v.read(false);
    inBot_u = inPort_u.read();


    if (inBot_u!=NULL && !got_u){
      rec_im_u = (Ipp8u*) inBot_u->get(0).asBlob();
      c_u->proc_im_8u(rec_im_u, psb_in);
      ippiAdd_32f_C1IR(c_u->get_centsur_32f(),c_u->get_psb_32f(),cs_tot_32f,psb_32f,srcsize);
      got_u = true;
    }


    if (inBot_v!=NULL && !got_v){
      rec_im_v = (Ipp8u*) inBot_v->get(0).asBlob();
      c_v->proc_im_8u(rec_im_v, psb_in);
      ippiAdd_32f_C1IR(c_v->get_centsur_32f(),c_v->get_psb_32f(),cs_tot_32f,psb_32f,srcsize);
      got_v = true;
    }
    
    
    if (got_u && got_v){
      
      conv_32f_to_8u(cs_tot_32f,psb_32f,cs_tot_8u,psb,srcsize);
      
      //OUTPUT:
      Bottle& tmpBot_colcs = outPort_colcs.prepare();
      tmpBot_colcs.clear();
      tmpBot_colcs.add(Value::makeBlob( cs_tot_8u, srcsize.width*srcsize.height));
      tmpBot_colcs.addString("ccs");
      outPort_colcs.write();  // Send it on its way
      
      //Reset:
      ippiSet_32f_C1R(0.0,cs_tot_32f,psb_32f,srcsize); 
      got_u = false;
      got_v = false;
    }


    else if (inBot_u==NULL && inBot_v==NULL){
      printf("No Input\n");
      usleep(5000); //dont blow out port
    }
    
    TSTOP

  }
  

  //never here..
  TEND
  
}
