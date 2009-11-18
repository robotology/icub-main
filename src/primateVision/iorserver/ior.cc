/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <string>
#include <iostream>

//MY INCLUDES
#include <timing.h>
#include <convert_bitdepth.h>
#include <norm.h>
//this is a REC CLIENT!!!
#include <recio.h>
//this is a FLOW CLIENT!!!
#include <flowsio.h>
#include "ior.h"
 

iCub::contrib::primateVision::IORServer::IORServer(string*cfg_)
{

  cfg=cfg_;

  start();

}

iCub::contrib::primateVision::IORServer::~IORServer()
{

}

void iCub::contrib::primateVision::IORServer::run(){

  Property prop;
  prop.fromConfigFile(cfg->c_str());

  string name         = "IOR";
  int    input        = prop.findGroup("IOR").find("INPUT").asInt();
  bool   dynamic_flow = (bool) prop.findGroup("IOR").find("DYNAMIC_FLOW").asInt();
  int    g_width      = prop.findGroup("IOR").find("G_WIDTH").asInt();
  int    g_height     = prop.findGroup("IOR").find("G_HEIGHT").asInt();
  double g_sigma      = prop.findGroup("IOR").find("G_SIGMA").asDouble();
  double decay        = prop.findGroup("IOR").find("DECAY").asDouble();
  yarp::os::ConstString temp_port    = prop.findGroup("IOR").find("INPORT").asString();
  QString in_port =  temp_port.c_str();

  QString in_source,rec_name;
  if (input==0){in_source="l";rec_name="left";}
  else{in_source="r";rec_name="right";}

  IppiSize groi;
  groi.width = g_width;
  groi.height = g_height;



  // INPUT Servers:
  //rec server (for mos_width/mos_height/width/height/psb_in)
  Port inPort_r;
  inPort_r.open("/iorserver_"+in_source+"/input/r_serv_params");     // Give it a name on the network.
  Network::connect("/iorserver_"+in_source+"/input/r_serv_params", in_port + "/output/serv_params");
  Network::connect( in_port + "/output/serv_params", "/iorserver_"+in_source+"/input/r_serv_params");
  BinPortable<RecServerParams> rserver_response; 
  Bottle rempty;
  inPort_r.write(rempty,rserver_response);
  RecServerParams rsp = rserver_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;
  int mos_width  = rsp.mos_width;
  int mos_height = rsp.mos_height;
  int width  = rsp.width;
  int height = rsp.height;
  int psb_in = rsp.psb;



  //flow server
  Port inPort_f;
  BinPortable<FlowServerParams> fserver_response; 
  Bottle fempty;
  int subflow;
  if(dynamic_flow){
      inPort_f.open("/iorserver_"+in_source+"/input/f_serv_params");     // Give it a name on the network.
      Network::connect("/iorserver_"+in_source+"/input/f_serv_params", "/flowserver_"+in_source+"/output/serv_params");
      Network::connect("/flowserver_"+in_source+"/output/serv_params", "iorserver_"+in_source+"/input/f_serv_params");
      inPort_f.write(fempty,fserver_response);

      FlowServerParams fsp = fserver_response.content();
      std::cout << "FlowServer Probe Response: " << fsp.toString() << std::endl;
      subflow = fsp.subflow;
  }



  //INPUT PORTS:
  //flow results:  
  BufferedPort<Bottle> inPort_x;
  BufferedPort<Bottle> inPort_y;
  Bottle *inBot_x;
  Bottle *inBot_y;
  //rec status:
  BufferedPort<BinPortable<RecResultParams> > inPort_p;
  RecResultParams *rec_res_x,*rec_res_y; 
  BinPortable<RecResultParams> *rec_res; 

  if (dynamic_flow){
      //connect to flow servers:
      inPort_y.open("/iorserver_"+in_source+"/input/fy");     // Give it a name on the network.
      Network::connect("/flowserver_"+in_source+"/output/y" , "/iorserver_"+in_source+"/input/fy");
      inPort_x.open("/iorserver_"+in_source+"/input/fx");     // Give it a name on the network.
      Network::connect("/flowserver_"+in_source+"/output/x" , "/iorserver_"+in_source+"/input/fx");
  }
  else{
      //just connect to recserver:
      inPort_p.open("/iorserver_"+in_source+"/input/rec_params");     // Give it a name on the network.
      Network::connect(in_port + "/output/rec_params", "/iorserver_"+in_source+"/input/rec_params");
  }




  // OUTPUT PORTS:
  //this server's params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/iorserver_"+in_source+"/output/serv_params");

  //computed output image:
  BufferedPort<Bottle> outPort_ior;
  outPort_ior.open("/iorserver_"+in_source+"/output/ior");



  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;
  IppiSize roi;
  roi.width = width;
  roi.height = height;
  IppiSize mroi;
  mroi.width = mos_width;
  mroi.height = mos_height;
 


  int psb,psb_m,psb_32,psb_g;
  Ipp32f*ior     = ippiMalloc_32f_C1(mos_width,mos_height,&psb_m);
  Ipp32f*tmp_ior = ippiMalloc_32f_C1(width,height,&psb_32);
  Ipp32f*ior_out_32f = ippiMalloc_32f_C1(width,height,&psb_32);
  Ipp32f*gauss   = ippiMalloc_32f_C1(g_width,g_height,&psb_g);
  Ipp32f*radius  = ippiMalloc_32f_C1(g_width,g_height,&psb_g);
  Ipp32f*f_ior   = ippiMalloc_32f_C1(width,height,&psb_32);
  Ipp32f*f_ior_b = ippiMalloc_32f_C1(width,height,&psb_32);
  Ipp32f*tmp_32f = ippiMalloc_32f_C1(width,height,&psb_32);
  Ipp8u*ior_out = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u*flow_x = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u*flow_y = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);


  ippiSet_32f_C1R(0.0,ior,psb_m,mroi);

  //make gauss IOR dec. pattern:
  for (int y=0;y<g_height;y++){ 
    for (int x=0;x<g_width;x++){
      radius[y*g_width+x] = sqrt((float) ( (x-g_width/2)*(x-g_width/2) + (y-g_height/2)*(y-g_height/2) ) );
      gauss[y*g_width+x]  = exp((float) ( -(radius[y*g_width+x]*radius[y*g_width+x])/(2.0*g_sigma*g_sigma) ) );
    }
  } 



  // Make replier for server param probes on server port:
  //Set Params: 
  IORServerParams isp;
  isp.width=width;
  isp.height=height;
  isp.psb=psb;
  isp.mos_width=mos_width;
  isp.mos_height=mos_height;
  IORReplyParamProbe server_replier;
  //Replier:
  server_replier.reply=isp;
  outPort_s.setReplier(server_replier);

 

  int p_x,p_y;
  int ior_pos_x,ior_pos_y;
  int fx,fy;
  bool got_x=false,got_y=false;



  bool got_input;


  TCREATE
      
      
  //MAIN EVENT LOOP:
  while (1){



      TSTART
    


      got_input=false;
        
      if (dynamic_flow){
          
          //
          //Propagate IOR dynamically according to flow.
          //
          
          //fx,fy appear:
          //value:   0  1  2  3  4  5  6  7
          //meaning: N -3 -2 -1  0  1  2  3 pixels
          
          inBot_y = inPort_y.read(false);
          inBot_x = inPort_x.read(); 
          
          
          if (inBot_x!=NULL){
              ippiCopy_8u_C1R((Ipp8u*) inBot_x->get(0).asBlob(),psb_in,flow_x,psb,srcsize);
              rec_res_x = (RecResultParams*)inBot_x->get(2).asBlob();
              if (input==0){
                  p_x = rec_res_x->lx;
              }
              else{
                  p_x = rec_res_x->rx;
              }
              got_x = true;
          }
          
          if(inBot_y!=NULL){
              ippiCopy_8u_C1R((Ipp8u*) inBot_y->get(0).asBlob(),psb_in,flow_y,psb,srcsize);
              rec_res_y = (RecResultParams*)inBot_y->get(2).asBlob();
              if (input==0){
                  p_y = rec_res_y->ly;
              }
              else{
                  p_y = rec_res_y->ry;
              }
              got_y = true;
          }
          
          
          
          //READY TO PROCESS??!!
          if(got_x && got_y){ 
              
              //make copy of relevant region of last IOR:
              ior_pos_x = mos_width/2  + p_x - width/2;
              ior_pos_y = mos_height/2 + p_y - height/2;
              ippiCopy_32f_C1R(&ior[ior_pos_x + ior_pos_y*psb_m/4],psb_m,tmp_ior,psb_32,roi);
              
              //propagate last IOR according to flow: 
              for (int y=0;y<height;y++){ 
                  for (int x=0;x<width;x++){
                      
                      //if non-zero flow:
                      if ( flow_x[x+y*psb_in] != 4 ||
                           flow_y[x+y*psb_in] != 4 ){
                          
                          //move previous IOR to new pos according to flow:
                          fx = x - (flow_x[x+y*psb_in]-4)*subflow;
                          fy = y - (flow_y[x+y*psb_in]-4)*subflow;
                          if (fx>=0 && fx<width && fy>=0 && fy<height){
                              //copy previous IOR to new flow pos:
                              f_ior[fx + fy*psb_32/4] = tmp_ior[x + y*psb_32/4];
                              //reduce 'copied from' area
                              //(we haven't seen what's behind it, so no inhibition):
                              tmp_ior[x + y*psb_32/4] = 0.0;
                          }
                      }
                  }
              }
              
              //Construct propagated IOR:
              for (int y=0;y<height;y++){ 
                  for (int x=0;x<width;x++){
                      {
                          //copy in any propagated IOR:
                          if ( flow_x[x+y*psb_in] != 4 ||
                               flow_y[x+y*psb_in] != 4 ){
                              tmp_ior[x + y*psb_32/4] = f_ior[x + y*psb_32/4];
                          }
                      }
                  }
              }
              
              //smear propagated IOR slightly with gaussian as certainty of 
              //localisation spreads and reduces over time:
              
              ippiFilterGauss_32f_C1R(tmp_ior,psb_32, 
                                      f_ior_b,psb_32,
                                      roi,ippMskSize5x5);
              
              //paste back into IOR:
              roi.width = width-4;
              roi.height = height-4;
              ippiCopy_32f_C1R(&f_ior_b[2+(psb_32/4)*2],psb_32,
                               &ior[ior_pos_x+2+(ior_pos_y+2)*(psb_m/4)],
                               psb_m,
                               roi);
              roi.width = width;
              roi.height = height;
              
              //reset propagated IOR:
              ippiSet_32f_C1R(0.0,f_ior,psb_32,roi);      

              got_input=true;  
          }
          else{
              got_input=false;
          }   
      }
  
  
      //NO DYNAMIC FLOW:
      else{
          //just get params from recserver:
          rec_res = inPort_p.read(); //blocking buffered
          
          if (input==0){//left:
              p_x = rec_res->content().lx;
          }
          else{ //right:
              p_x = rec_res->content().rx;
          }
          p_y  = rec_res->content().ly;
          got_input=true;
      }
  
  
  
  
  
      if (got_input){
          
          //
          //static IOR component:
          //
          
          
          //Inc current fixation with gauss pattern - (the longer 
          //we've looked at something, the less we are inclined to look again):
          ior_pos_x = mos_width/2  + p_x - g_width/2;
          ior_pos_y = mos_height/2 + p_y - g_height/2;
          
          //inc fixation point:
          ippiAdd_32f_C1IR(gauss,psb_g,&ior[ior_pos_x + ior_pos_y*psb_m/4],psb_m,groi);

          //tend everything back to zero:
          ippiMulC_32f_C1IR(decay,ior,psb_m,mroi);

          ior_pos_x = mos_width/2  + p_x - width/2;
          ior_pos_y = mos_height/2 + p_y - height/2;
          
          //send imframe-sized output!!
          ippiCopy_32f_C1R(&ior[ior_pos_x + ior_pos_y*psb_m/4],psb_m,ior_out_32f,psb_32,roi);
          //negate:
          ippiMulC_32f_C1IR(-1.0,ior_out_32f,psb_32,roi);   
          //convert to 8u:
          conv_32f_to_8u(ior_out_32f,psb_32,ior_out,psb,roi);

          
          
          
          //OUTPUT:
          Bottle& tmpBot_ior = outPort_ior.prepare();
          tmpBot_ior.clear();
          tmpBot_ior.add(Value::makeBlob( ior_out, psb*srcsize.height));
          tmpBot_ior.addString("ior");
          outPort_ior.write();  // Send it on its way
          
          got_x = false;
          got_y = false;
          
      }
      else{
          printf("No Input\n");
          usleep(5000); //Don't blow out port: 
      }
   
   
      TSTOP    

      
   }

//never here..  
TEND

}
