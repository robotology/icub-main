/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>

//My includes
#include <flow.h>  
#include <dog.h>  
#include <timing.h>  
#include <convert_bitdepth.h>
// THIS APP IS A RECCLIENT!!!
#include <recio.h>
#include "flows.h"


#define PAD_B 32
#define PAD_INT  16

iCub::contrib::primateVision::FlowServer::FlowServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::FlowServer::~FlowServer()
{
  
}



void iCub::contrib::primateVision::FlowServer::run(){
  
  Property prop;
  prop.fromConfigFile(cfg->c_str());

  int flow_scale       = prop.findGroup("FLOWS").find("FLOW_SCALE").asInt();
  int bland_dog_thresh = prop.findGroup("FLOWS").find("BLAND_DOG_THRESH").asInt();
  int input            = prop.findGroup("FLOWS").find("INPUT").asInt();
  yarp::os::ConstString temp_port    = prop.findGroup("FLOWS").find("INPORT").asString();
  QString in_port =  temp_port.c_str();


  QString in_source,rec_name;
  if (input==0){in_source="l";rec_name="left";}
  else{in_source="r";rec_name="right";}


  //IN PORTS:
  Port inPort_s;
  inPort_s.open("/flowserver_"+in_source+"/input/serv_params");     // Give it a name on the network.
  Network::connect("/flowserver_"+in_source+"/input/serv_params", in_port + "/output/serv_params");
  Network::connect( in_port + "/output/serv_params", "/flowserver_"+in_source+"/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 
  IppiSize srcsize;
  srcsize.width = rsp.width;
  srcsize.height = rsp.height;
  int psb_in = rsp.psb;


  RecResultParams *rec_res; 


  BufferedPort<Bottle> inPort_y;      // Create a port
  inPort_y.open("/flowserver_"+in_source+"/input/rec_y");     // Give it a name on the network.
  Network::connect(in_port + "/output/"+rec_name+"_ye" , "/flowserver_"+in_source+"/input/rec_y");
  Bottle *inBot_y;
  Ipp8u* rec_im_y;





  // OUTPUT PORTS:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/flowserver_"+in_source+"/output/serv_params");

  BufferedPort<Bottle> outPort_x;
  outPort_x.open("/flowserver_"+in_source+"/output/x");

  BufferedPort<Bottle> outPort_x_filt;
  outPort_x_filt.open("/flowserver_"+in_source+"/output/x_filt");

  BufferedPort<Bottle> outPort_y;
  outPort_y.open("/flowserver_"+in_source+"/output/y");

  BufferedPort<Bottle> outPort_y_filt;
  outPort_y_filt.open("/flowserver_"+in_source+"/output/y_filt");


  BufferedPort<Bottle> outPort_sal;
  outPort_sal.open("/flowserver_"+in_source+"/output/sal");







  //PROCESSING CLASS SETUP:
  DoG*d = new DoG(srcsize);
  Flow*f  = new Flow(srcsize,flow_scale);


  int psb,psb_32f,psb_pad;
  Ipp8u* x_filt     = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* y_filt     = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* x_filt_tmp = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* y_filt_tmp = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* sal        = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u* sal_p      = ippiMalloc_8u_C1(srcsize.width+2*PAD_B,srcsize.height+2*PAD_B,&psb_pad);
  Ipp8u* sal_p_filt = ippiMalloc_8u_C1(srcsize.width+2*PAD_B,srcsize.height+2*PAD_B,&psb_pad);
  Ipp32f* tmp1_32f  = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* tmp2_32f  = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);




  // Make replier for server param probes on server port:
  //Set Params:
  FlowServerParams fsp;
  fsp.width=srcsize.width;
  fsp.height=srcsize.height;
  fsp.psb=psb;
  fsp.subflow=flow_scale;
  //Replier:
  FlowReplyParamProbe server_replier;
  server_replier.reply=fsp;
  outPort_s.setReplier(server_replier);

  IppiSize psize;
  psize.width  = srcsize.width  + 2*PAD_B;
  psize.height = srcsize.height + 2*PAD_B;

  IppiSize pintsize;
  pintsize.width  = srcsize.width  - 2*PAD_INT;
  pintsize.height = srcsize.height - 2*PAD_INT;


  TCREATE

  //MAIN EVENT LOOP:
  while (1){

    TSTART

    //get image:
    inBot_y = inPort_y.read();//false



    if (inBot_y!=NULL){
      rec_im_y = (Ipp8u*) inBot_y->get(0).asBlob();
      rec_res = (RecResultParams*) inBot_y->get(1).asBlob(); 
      
      //flow is done on DOG:
      d->proc(rec_im_y,psb_in);
      //lx,ly hold mosaic coords of window.
      //get flow:
      f->proc(d->get_dog_onoff(),d->get_psb(),rec_res->lx,rec_res->ly);
      //fx,fy appear:
      //value:   0  1  2  3  4  5  6  7 
      //meaning: N -3 -2 -1  0  1  2  3 pixels
      
      ippiCopy_8u_C1R(f->get_fx(),f->get_psb(),x_filt,psb,srcsize);
      ippiCopy_8u_C1R(f->get_fy(),f->get_psb(),y_filt,psb,srcsize);
      
      
      //remove noisy flow in bland regions:
      for (int y=0;y<srcsize.height;y++){ 
	for (int x=0;x<srcsize.width;x++){
	  if (d->get_dog_onoff()[x + y*d->get_psb()] < bland_dog_thresh ){ 
	    x_filt[x + y*psb] = 0; //N
	    y_filt[x + y*psb] = 0; //N
	  }
	}
      }
      
      
      
      //FLOW SALIENCY (faster things are more 'eye-catching'):
      //remove Ns (set 0s to 4s):
      ippiThreshold_LTVal_8u_C1R(x_filt,psb,x_filt_tmp,psb,srcsize,(Ipp8u)1,(Ipp8u)4);
      ippiThreshold_LTVal_8u_C1R(y_filt,psb,y_filt_tmp,psb,srcsize,(Ipp8u)1,(Ipp8u)4);
      //fx,fy are now:
      //  1  2  3  4  5  6  7 
      // -3 -2 -1  0  1  2  3  
      //convert to 32f:
      ippiConvert_8u32f_C1R(x_filt_tmp,psb,tmp1_32f,psb_32f,srcsize);
      ippiConvert_8u32f_C1R(y_filt_tmp,psb,tmp2_32f,psb_32f,srcsize);
      //subtract 4.0 to obtain absolute pixel flow:
      ippiSubC_32f_C1IR(4.0,tmp1_32f,psb_32f,srcsize);
      ippiSubC_32f_C1IR(4.0,tmp2_32f,psb_32f,srcsize);
      //get magnitude of speed: sqrt(x^2 + y^2)
      ippiSqr_32f_C1IR(tmp1_32f,psb_32f,srcsize);
      ippiAddSquare_32f_C1IR(tmp2_32f,psb_32f,tmp1_32f,psb_32f,srcsize);
      ippiSqrt_32f_C1IR(tmp1_32f,psb_32f,srcsize); 
      //normalise to 255.0 so same range as intensity cs cue:
      //norm255_32f(tmp1_32f,psb_32f,tmp2_32f,psb_32f,srcsize); 
      //convert to 8u:    
      conv_32f_to_8u(tmp1_32f,psb_32f,sal,psb,srcsize);
      
#if 1
      //BLUR SAL:
      ippiCopyReplicateBorder_8u_C1R(sal,psb,srcsize,sal_p,psb_pad,psize,PAD_B,PAD_B);
      ippiFilterGauss_8u_C1R(sal_p,psb_pad,sal_p_filt,psb_pad,psize,ippMskSize5x5);
      //remove any edge effects:
      ippiSet_8u_C1R(0,sal,psb,srcsize);
      ippiCopy_8u_C1R(&sal_p_filt[PAD_B+PAD_INT+(PAD_B+PAD_INT)*psb_pad],psb_pad,sal,psb,pintsize);
#endif
      
      
      
      //prepare output maps:
      Bottle& tmpBot_sal = outPort_sal.prepare();
      tmpBot_sal.clear();
      tmpBot_sal.add(Value::makeBlob( sal, psb*srcsize.height));
      tmpBot_sal.addString("fcs");
      outPort_sal.write();  // Send it on its way

      Bottle& tmpBot_x = outPort_x.prepare();
      tmpBot_x.clear();
      tmpBot_x.add(Value::makeBlob( f->get_fx(), f->get_psb()*srcsize.height));
      tmpBot_x.addString("fx");
      tmpBot_x.add(Value::makeBlob(rec_res,sizeof(RecResultParams)));
      outPort_x.write();  // Send it on its way
      
      Bottle& tmpBot_y = outPort_y.prepare();
      tmpBot_y.clear();
      tmpBot_y.add(Value::makeBlob( f->get_fy(), f->get_psb()*srcsize.height));
      tmpBot_y.addString("fy");
      tmpBot_y.add(Value::makeBlob(rec_res,sizeof(RecResultParams)));
      outPort_y.write();  // Send it on its way
      
      Bottle& tmpBot_x_filt = outPort_x_filt.prepare();
      tmpBot_x_filt.clear();
      tmpBot_x_filt.add(Value::makeBlob( x_filt, psb*srcsize.height));
      tmpBot_x_filt.addString("fx_filt");
      tmpBot_x_filt.add(Value::makeBlob(rec_res,sizeof(RecResultParams)));
      outPort_x_filt.write();  // Send it on its way
      
      Bottle& tmpBot_y_filt = outPort_y_filt.prepare();
      tmpBot_y_filt.clear();
      tmpBot_y_filt.add(Value::makeBlob( y_filt, psb*srcsize.height));
      tmpBot_y_filt.addString("fy_filt");
      tmpBot_y_filt.add(Value::makeBlob(rec_res,sizeof(RecResultParams)));
      outPort_y_filt.write();  // Send it on its way
            
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

