/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>

//My includes
#include <timing.h>
#include <convert_bitdepth.h>
// THIS APP IS A RECCLIENT!!!
#include <recio.h>
#include "tsb.h"


iCub::contrib::primateVision::TSBServer::TSBServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::TSBServer::~TSBServer()
{
  
}


void iCub::contrib::primateVision::TSBServer::run(){

  Property prop;
  prop.fromConfigFile(cfg->c_str());

  int input        = prop.findGroup("TSB").find("INPUT").asInt();
  double sigma_tsb = prop.findGroup("TSB").find("TSB_SIGMA").asDouble();
  

  QString in_source,rec_name;
  if (input==0){in_source="l";rec_name="left";}
  else{in_source="r";rec_name="right";}


  //IN PORTS:
  Port inPort_s;
  inPort_s.open("/tsbserver_"+in_source+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/tsbserver_"+in_source+"/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/tsbserver_"+in_source+"/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 
  IppiSize srcsize;
  srcsize.width = rsp.width;
  srcsize.height = rsp.height;
  int mos_width = rsp.mos_width;
  int mos_height = rsp.mos_height;
  int psb_in = rsp.psb;


  BufferedPort<BinPortable<RecResultParams> > inPort_p;
  inPort_p.open("/tsbserver_"+in_source+"/input/rec_params");     // Give it a name on the network.
  Network::connect("/recserver/output/rec_params", "/tsbserver_"+in_source+"/input/rec_params");
  BinPortable<RecResultParams> *rec_res; 



  // OUTPUT PORTS:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/tsbserver_"+in_source+"/output/serv_params");

  BufferedPort<Bottle> outPort_tsb;
  outPort_tsb.open("/tsbserver_"+in_source+"/output/tsb");


  //MOS WIDTH SHOULD COME FROM REC SERVER!!
  int psb_32f_m,psb;
  int p_xl,p_xr,p_y,ior_pos_x,ior_pos_y;
  Ipp32f*tsb        = ippiMalloc_32f_C1(mos_width,mos_height,&psb_32f_m);
  Ipp32f*radius_tsb = ippiMalloc_32f_C1(mos_width,mos_height,&psb_32f_m);
  Ipp8u*tsb_out     = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);


  IppiSize mroi,roi;
  mroi.width = mos_width;
  mroi.height = mos_height;
  roi.width = srcsize.width;
  roi.height = srcsize.height;


  // Make replier for server param probes on server port:
  //Set Params:
  TSBServerParams fsp;
  fsp.width=srcsize.width;
  fsp.height=srcsize.height;
  fsp.psb=psb;
  //Replier:
  TSBReplyParamProbe server_replier;
  server_replier.reply=fsp;
  outPort_s.setReplier(server_replier);


 

  //Static TSB:

  if (sigma_tsb!=0.0){
    
    //make TSB gauss:
    for (int y=0;y<mos_height;y++){ 
      for (int x=0;x<mos_width;x++){
	radius_tsb[y*mos_width+x] = sqrt((float) (x-mos_width/2)*(x-mos_width/2) + (y-mos_height/2)*(y-mos_height/2) );
	tsb[y*mos_width+x] = exp(-(radius_tsb[y*mos_width+x]*radius_tsb[y*mos_width+x])/(2.0*sigma_tsb*sigma_tsb));
      }
    } 
 
  }




  TCREATE

 //MAIN EVENT LOOP:
  while (1){



    TSTART


    rec_res = inPort_p.read(); //blocking buffered
 


    if (&rec_res!=NULL){

      p_xl = rec_res->content().lx;
      p_xr = rec_res->content().rx;
      p_y  = rec_res->content().ly;

      //return relevant region of TSB at requested coords:
      if (input==0){//left
	ior_pos_x = mos_width/2  + p_xl - srcsize.width/2;
	ior_pos_y = mos_height/2 + p_y - srcsize.height/2;
      }
      else{//right
	ior_pos_x = mos_width/2  + p_xr - srcsize.width/2;
	ior_pos_y = mos_height/2 + p_y - srcsize.height/2;
      }
      
      conv_32f_to_8u(&tsb[ior_pos_x + ior_pos_y*mos_width],psb_32f_m,tsb_out,psb,roi);
      
      
      //prepare output maps:
      Bottle& tmpBot_tsb = outPort_tsb.prepare();
      tmpBot_tsb.clear();
      tmpBot_tsb.add(Value::makeBlob( tsb_out, psb*srcsize.height));
      tmpBot_tsb.addString("tsb");
      outPort_tsb.write();  // Send it on its way
    }

    else{
      printf("No Input\n");
      usleep(5000); //don't blow out port
    }
        
    TSTOP

   }

  //never here...
  TEND

}
