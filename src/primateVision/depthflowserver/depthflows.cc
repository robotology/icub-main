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
#include <depthflow.h> 
#include <convert_bitdepth.h> 
// THIS APP IS A RECCLIENT!!!
#include <recio.h>
#include "depthflows.h"



iCub::contrib::primateVision::DepthflowServer::DepthflowServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::DepthflowServer::~DepthflowServer()
{
  
}


void iCub::contrib::primateVision::DepthflowServer::run(){
  

  Property prop;
  prop.fromConfigFile(cfg->c_str());
  int test_depth  = prop.findGroup("DEPTHFLOWS").find("TEST_DEPTH").asInt();
  int range       = prop.findGroup("DEPTHFLOWS").find("RANGE").asInt();
  int offset      = prop.findGroup("DEPTHFLOWS").find("OFFSET").asInt();
  int filterSize  = prop.findGroup("DEPTHFLOWS").find("FILTERSIZE").asInt();
  int filterCap   = prop.findGroup("DEPTHFLOWS").find("FILTERCAP").asInt();
  int windowSize  = prop.findGroup("DEPTHFLOWS").find("WINDOWSIZE").asInt();
  int threshold   = prop.findGroup("DEPTHFLOWS").find("THRESHOLD").asInt();
  int uniqueness  = prop.findGroup("DEPTHFLOWS").find("UNIQUENESS").asInt();



  //IN PORTS:
  Port inPort_s;
  inPort_s.open("/depthflowserver/input/serv_params");
  Network::connect("/depthflowserver/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/depthflowserver/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 
  IppiSize srcsize;
  srcsize.width = rsp.width;
  srcsize.height = rsp.height;
  int psb_in = rsp.psb;
  double baseline = rsp.baseline;


  RecResultParams *rec_res_l; 
  RecResultParams *rec_res_r; 

  BufferedPort<Bottle> inPort_yl;
  inPort_yl.open("/depthflowserver/input/rec_yl");
  Network::connect("/recserver/output/left_ye" , "/depthflowserver/input/rec_yl");
  Bottle *inBot_yl;
  Ipp8u* rec_im_yl;

  BufferedPort<Bottle> inPort_yr; 
  inPort_yr.open("/depthflowserver/input/rec_yr");  
  Network::connect("/recserver/output/right_ye" , "/depthflowserver/input/rec_yr");
  Bottle *inBot_yr;
  Ipp8u* rec_im_yr;





  // OUTPUT PORTS:
  BufferedPort<Bottle> outPort_s;
  BufferedPort<Bottle> outPort_disp;
  BufferedPort<Bottle> outPort_depth;
  BufferedPort<Bottle> outPort_depthflow;
  BufferedPort<Bottle> outPort_sal;
  outPort_s.open("/depthflowserver/output/serv_params");
  outPort_disp.open("/depthflowserver/output/disp");
  outPort_depth.open("/depthflowserver/output/depth");
  outPort_depthflow.open("/depthflowserver/output/depthflow");
  outPort_sal.open("/depthflowserver/output/sal");



  int psb;
  Ipp8u* sal = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);


  Depthflow*df  = new Depthflow(srcsize,
				filterSize,
				filterCap,
				windowSize,
				offset,
				range,
				threshold,
				uniqueness,
				baseline,
				rsp.focus);

  DepthflowResultParams df_res_params;


  //Make replier for server param probes on server port:
  //Set Params:
  DepthflowServerParams dfsp;
  dfsp.width=srcsize.width;
  dfsp.height=srcsize.height;
  dfsp.psb=df->get_psb();
  dfsp.psb32f=df->get_psb_32f();
  dfsp.range=range;
  dfsp.offset=offset;
  dfsp.baseline=baseline;
  //Replier:
  DepthflowReplyParamProbe server_replier;
  server_replier.reply=dfsp;
  outPort_s.setReplier(server_replier);



 
  TCREATE


  //MAIN EVENT LOOP:
  while (1){
    
    TSTART

    inBot_yr = inPort_yr.read(false);   //non-blocking
    inBot_yl = inPort_yl.read();        //blocking

    
    if (inBot_yl!=NULL && inBot_yr!=NULL){

      rec_im_yl = (Ipp8u*) inBot_yl->get(0).asBlob();
      rec_res_l = (RecResultParams*) inBot_yl->get(1).asBlob(); 
      rec_im_yr = (Ipp8u*) inBot_yr->get(0).asBlob();
      rec_res_r = (RecResultParams*) inBot_yr->get(1).asBlob(); 


      df->proc(rec_im_yl, 
	       rec_im_yr, 
	       psb_in,
	       rec_res_l->lx,
	       rec_res_l->ly,
	       rec_res_r->rx,
	       rec_res_r->ry);
      
      df_res_params.px   = df->get_px();
      df_res_params.py   = df->get_py();
      df_res_params.hd   = df->get_hd();
      df_res_params.mind = df->get_mind();
      df_res_params.maxd = df->get_maxd();



      //CONSTRUCT SAL:

      //Make a left and right map!!!
      //ADDME!!
      //closer things more salient.
      //moving towards cams more salient.
      ippiSet_8u_C1R(100,sal,psb,srcsize);







      //disp:
      Bottle& tmpBot_disp = outPort_disp.prepare();
      tmpBot_disp.clear();
      tmpBot_disp.add(Value::makeBlob( df->get_disp(), df->get_psb()*srcsize.height));
      tmpBot_disp.add(Value::makeBlob(&df_res_params,sizeof(DepthflowResultParams)));
      outPort_disp.write();  // Send it on its way
      
      //depth:
      Bottle& tmpBot_depth = outPort_depth.prepare();
      tmpBot_depth.clear();
      tmpBot_depth.add(Value::makeBlob( df->get_depth(), df->get_psb_32f()*srcsize.height));
      tmpBot_depth.add(Value::makeBlob(&df_res_params,sizeof(DepthflowResultParams)));
      outPort_depth.write();  // Send it on its way
      
      //depthflow
      Bottle& tmpBot_depthflow = outPort_depthflow.prepare();
      tmpBot_depthflow.clear();
      tmpBot_depthflow.add(Value::makeBlob( df->get_depthflow(), df->get_psb_32f()*srcsize.height));
      tmpBot_depthflow.add(Value::makeBlob(&df_res_params,sizeof(DepthflowResultParams)));
      outPort_depthflow.write();  // Send it on its way
      

      //sal, made locally:
      Bottle& tmpBot_sal = outPort_sal.prepare();
      tmpBot_sal.clear();
      tmpBot_sal.add(Value::makeBlob( sal, psb*srcsize.height));
      tmpBot_sal.addString("dfcs");
      outPort_sal.write();  // Send it on its way
      

    }
    else{
      printf("No Input\n");
      usleep(1000); // Dont blow out port
    }

    TSTOP

  }

  //never here...  
  TEND

}
