/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <iostream>
#include <string>
#include <qapplication.h>
#include <ipp.h>

// My includes
#include <timing.h>
#include <convert_bitdepth.h>
//client of:
#include <flowsio.h>
#include <ocsio.h>
#include <ycsio.h>
#include <colcsio.h>

#include "sal.h"

iCub::contrib::primateVision::SalServer::SalServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::SalServer::~SalServer()
{
  
}


void iCub::contrib::primateVision::SalServer::run(){

  Property prop;
  prop.fromConfigFile(cfg->c_str());

  int input       = prop.findGroup("SAL").find("INPUT").asInt();
  double gain_ccs = prop.findGroup("SAL").find("GAIN_CCS").asDouble();
  double gain_ycs = prop.findGroup("SAL").find("GAIN_YCS").asDouble();
  double gain_ocs = prop.findGroup("SAL").find("GAIN_OCS").asDouble();
  double gain_sym = prop.findGroup("SAL").find("GAIN_SYM").asDouble();
  double gain_edg = prop.findGroup("SAL").find("GAIN_EDG").asDouble();
  double gain_cor = prop.findGroup("SAL").find("GAIN_COR").asDouble();
  double gain_fcs = prop.findGroup("SAL").find("GAIN_FCS").asDouble();

  bool input_fcs = (bool) prop.findGroup("SAL").find("INPUT_FCS").asInt();
  bool input_ccs = (bool) prop.findGroup("SAL").find("INPUT_CCS").asInt();
  bool input_ycs = (bool) prop.findGroup("SAL").find("INPUT_YCS").asInt();
  bool input_ocs = (bool) prop.findGroup("SAL").find("INPUT_OCS").asInt();
  bool input_edg = (bool) prop.findGroup("SAL").find("INPUT_EDG").asInt();
  bool input_cor = (bool) prop.findGroup("SAL").find("INPUT_COR").asInt();
  bool input_sym = (bool) prop.findGroup("SAL").find("INPUT_SYM").asInt();


  IppiSize srcsize;
  srcsize.width = 0;
  srcsize.height = 0;
  int psb_in = 0;



  QString in_source,rec_name;
  if (input==0){in_source="l";}
  else{in_source="r";}


  Port inPort_s;
  inPort_s.open("/salserver_"+in_source+"/input/serv_params");
  Bottle empty;


  //Flow:
  Network::connect("/salserver_"+in_source+"/input/serv_params", "/flowserver_"+in_source+"/output/serv_params");
  Network::connect("/flowserver_"+in_source+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BinPortable<FlowServerParams> f_server_response; 
  inPort_s.write(empty,f_server_response);
  FlowServerParams fsp = f_server_response.content();
  std::cout << "FlowServer Probe Response: " << fsp.toString() << std::endl;
  Network::disconnect("/salserver_"+in_source+"/input/serv_params", "/flowserver_"+in_source+"/output/serv_params");
  Network::disconnect("/flowserver_"+in_source+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BufferedPort<Bottle> inPort_fcs;
  inPort_fcs.open("/salserver_"+in_source+"/input/flow");
  Network::connect("/flowserver_"+in_source+"/output/sal", "/salserver_"+in_source+"/input/flow");
  Bottle *inBot_fcs;
  if (srcsize.width == 0){
    srcsize.width = fsp.width;
    srcsize.height = fsp.height;
    psb_in = fsp.psb;
  }

  //Col:
  Network::connect("/salserver_"+in_source+"/input/serv_params", "/colcsserver_"+in_source+"/output/serv_params");
  Network::connect("/colcsserver_"+in_source+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BinPortable<ColCSServerParams> c_server_response; 
  inPort_s.write(empty,c_server_response);
  ColCSServerParams csp = c_server_response.content();
  std::cout << "ColCSServer Probe Response: " << csp.toString() << std::endl;
  Network::disconnect("/salserver_"+in_source+"/input/serv_params", "/colcsserver_"+in_source+"/output/serv_params");
  Network::disconnect("/colcsserver_"+in_source+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BufferedPort<Bottle> inPort_ccs;
  inPort_ccs.open("/salserver_"+in_source+"/input/colcs");
  Network::connect("/colcsserver_"+in_source+"/output/colcs", "/salserver_"+in_source+"/input/colcs");
  Bottle *inBot_ccs;
  if (srcsize.width == 0){
    srcsize.width = csp.width;
    srcsize.height = csp.height;
    psb_in = csp.psb;
  }

  //Y;
  Network::connect("/salserver_"+in_source+"/input/serv_params", "/ycsserver_"+in_source+"/output/serv_params");
  Network::connect("/ycsserver_"+in_source+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BinPortable<YCSServerParams> y_server_response; 
  inPort_s.write(empty,y_server_response);
  YCSServerParams ysp = y_server_response.content();
  std::cout << "YCSServer Probe Response: " << ysp.toString() << std::endl;
  Network::disconnect("/salserver_"+in_source+"/input/serv_params", "/ycsserver_"+in_source+"/output/serv_params");
  Network::disconnect("/ycsserver_"+in_source+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BufferedPort<Bottle> inPort_ycs;
  inPort_ycs.open("/salserver_"+in_source+"/input/ycs");
  Network::connect("/ycsserver_"+in_source+"/output/ycs", "/salserver_"+in_source+"/input/ycs");
  Bottle *inBot_ycs;
  if (srcsize.width == 0){
    srcsize.width = ysp.width;
    srcsize.height = ysp.height;
    psb_in = ysp.psb;
  }

  //Orient:
  Network::connect("/salserver_"+in_source+"/input/serv_params", "/ocsserver_"+QString::number(input)+"/output/serv_params");
  Network::connect("/ocsserver_"+QString::number(input)+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BinPortable<OCSServerParams> o_server_response; 
  inPort_s.write(empty,o_server_response);
  OCSServerParams osp = o_server_response.content();
  std::cout << "OCSServer Probe Response: " << osp.toString() << std::endl;
  Network::disconnect("/salserver_"+in_source+"/input/serv_params", "/ocsserver_"+QString::number(input)+"/output/serv_params");
  Network::disconnect("/ocsserver_"+QString::number(input)+"/output/serv_params", "/salserver_"+in_source+"/input/serv_params");
  BufferedPort<Bottle> inPort_ocs,inPort_cor,inPort_edg,inPort_sym;
  inPort_ocs.open("/salserver_"+in_source+"/input/ocs");
  inPort_cor.open("/salserver_"+in_source+"/input/M");
  inPort_edg.open("/salserver_"+in_source+"/input/m");
  inPort_sym.open("/salserver_"+in_source+"/input/phaseSym");
  Network::connect("/ocsserver_"+QString::number(input)+"/output/ocs", "/salserver_"+in_source+"/input/ocs");
  Network::connect("/ocsserver_"+QString::number(input)+"/output/M", "/salserver_"+in_source+"/input/M");
  Network::connect("/ocsserver_"+QString::number(input)+"/output/m", "/salserver_"+in_source+"/input/m");
  Network::connect("/ocsserver_"+QString::number(input)+"/output/phaseSym", "/salserver_"+in_source+"/input/phaseSym");
  Bottle *inBot_ocs;
  Bottle *inBot_edg;
  Bottle *inBot_cor;
  Bottle *inBot_sym;
  if (srcsize.width == 0){
    srcsize.width = osp.width;
    srcsize.height = osp.height;
    psb_in = osp.psb;
  }


  // OUTPUT PORTS:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/salserver_"+in_source+"/output/serv_params");

  BufferedPort<Bottle> outPort_sal;
  outPort_sal.open("/salserver_"+in_source+"/output/sal");


  int psb_32f,psb;
  Ipp8u*  sal_in;
  Ipp32f* tmp_32f = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* sal     = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp8u*  sal_out = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);


  // Make replier for server param probes on server port:
  //Set Params:
  SalServerParams ssp;
  ssp.width=srcsize.width;
  ssp.height=srcsize.height;
  ssp.psb=psb;
  //Replier:
  SalReplyParamProbe server_replier;
  server_replier.reply=ssp;
  outPort_s.setReplier(server_replier);


  bool got_ocs = !input_ocs;
  bool got_edg = !input_edg;
  bool got_cor = !input_cor;
  bool got_sym = !input_sym;
  bool got_ccs = !input_ccs;
  bool got_ycs = !input_ycs;
  bool got_fcs = !input_fcs; 
  


  TCREATE



    printf("begin...\n");
  //MAIN EVENT LOOP:
  while (1){

    TSTART



    inBot_ccs = inPort_ccs.read(false);
    inBot_ycs = inPort_ycs.read(false);
    inBot_ocs = inPort_ocs.read(false);
    inBot_edg = inPort_edg.read(false);
    inBot_cor = inPort_cor.read(false);
    inBot_sym = inPort_sym.read(false);
    inBot_fcs = inPort_fcs.read();


    //we wait for all cues so that saliency is not skewed in
    //favour of the fastest server, or by the most recent update!
    //only add each server result to saliency once.

    if (!got_fcs && inBot_fcs!=NULL){
      sal_in = (Ipp8u*) inBot_fcs->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMulC_32f_C1IR((Ipp32f)gain_fcs,tmp_32f,psb_32f,srcsize); 
      ippiAdd_32f_C1IR(tmp_32f,psb_32f,sal,psb_32f,srcsize);  
      got_fcs=true; 
     }

    if (!got_ccs && inBot_ccs!=NULL){
      sal_in = (Ipp8u*) inBot_ccs->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMulC_32f_C1IR((Ipp32f)gain_ccs,tmp_32f,psb_32f,srcsize); 
      ippiAdd_32f_C1IR(tmp_32f,psb_32f,sal,psb_32f,srcsize);  
      got_ccs=true; 
     }

    if (!got_ycs && inBot_ycs!=NULL){
      sal_in = (Ipp8u*) inBot_ycs->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMulC_32f_C1IR((Ipp32f)gain_ycs,tmp_32f,psb_32f,srcsize); 
      ippiAdd_32f_C1IR(tmp_32f,psb_32f,sal,psb_32f,srcsize);  
      got_ycs=true; 
     }

    if (!got_ocs && inBot_ocs!=NULL){
      sal_in = (Ipp8u*) inBot_ocs->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMulC_32f_C1IR((Ipp32f)gain_ocs,tmp_32f,psb_32f,srcsize); 
      ippiAdd_32f_C1IR(tmp_32f,psb_32f,sal,psb_32f,srcsize);  
      got_ocs=true; 
     }

    if (!got_edg && inBot_edg!=NULL){
      sal_in = (Ipp8u*) inBot_edg->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMulC_32f_C1IR((Ipp32f)gain_edg,tmp_32f,psb_32f,srcsize); 
      ippiAdd_32f_C1IR(tmp_32f,psb_32f,sal,psb_32f,srcsize);  
      got_edg=true; 
     }

    if (!got_cor && inBot_cor!=NULL){
      sal_in = (Ipp8u*) inBot_cor->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMulC_32f_C1IR((Ipp32f)gain_cor,tmp_32f,psb_32f,srcsize); 
      ippiAdd_32f_C1IR(tmp_32f,psb_32f,sal,psb_32f,srcsize);  
      got_cor=true; 
     }

    if (!got_sym && inBot_sym!=NULL){
      sal_in = (Ipp8u*) inBot_sym->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMulC_32f_C1IR((Ipp32f)gain_sym,tmp_32f,psb_32f,srcsize); 
      ippiAdd_32f_C1IR(tmp_32f,psb_32f,sal,psb_32f,srcsize);  
      got_sym=true;
     }


    //Do we have all EXPECTED maps?:
    if ( got_ocs &&
	 got_edg &&
	 got_cor &&
	 got_sym &&
	 got_fcs &&
	 got_ccs &&
	 got_ycs ){
      
      //prepare output:
      conv_32f_to_8u(sal,psb_32f,sal_out,psb,srcsize);
    
      //send it on its way:
      Bottle& tmpBot = outPort_sal.prepare();
      tmpBot.clear();
      tmpBot.add(Value::makeBlob( sal_out, psb*srcsize.height));
      outPort_sal.write();

      //reset checkers:
      got_ocs = !input_ocs;
      got_edg = !input_edg;
      got_cor = !input_cor;
      got_sym = !input_sym;
      got_ccs = !input_ccs;
      got_ycs = !input_ycs;
      got_fcs = !input_fcs;

      //set sal to 0.0:
      ippiSet_32f_C1R(0.0,sal,psb_32f,srcsize); 

    }
    else if (inBot_ocs == NULL && 
	     inBot_edg == NULL && 
	     inBot_cor == NULL && 
	     inBot_sym == NULL && 
	     inBot_ccs == NULL && 
	     inBot_ycs == NULL &&
	     inBot_fcs == NULL )
      {
	printf("No Input\n");
	usleep(5000);//don't blow out port
      }

    TSTOP

  }

  //never here..
  TEND

}




