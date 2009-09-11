/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>
#include <string>

//My includes:
#include <timing.h>
#include <convert_bitdepth.h>
//Client of:
#include <iorio.h>
#include <tsbio.h>
#include <salio.h>
//for motion & xcheck:
#include <recio.h>

#include "attn_st.h"


#define INFO 1  //print info?



iCub::contrib::primateVision::AttnServer_Stereo::AttnServer_Stereo(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::AttnServer_Stereo::~AttnServer_Stereo()
{
  
}


void iCub::contrib::primateVision::AttnServer_Stereo::run(){


  //ADD MRFZDF "tracking" boolean input from zdfserver.. will stop it 
  //saccading about while mrfzdf wants to keep track. Then remove 1sec sleep.



  Property prop;
  prop.fromConfigFile(cfg->c_str());

  int    num_v           = prop.findGroup("ATTN").find("NUM_CLUST").asInt();
  int    spread          = prop.findGroup("ATTN").find("SPREAD_CLUST").asInt();
  int    wait_time       = prop.findGroup("ATTN").find("TIMED_SHIFT").asInt();
  int    suspend_cycles  = prop.findGroup("ATTN").find("SUSPEND_CYCLES").asInt();
  bool   filter_saccades = (bool) prop.findGroup("ATTN").find("FILTER_SACCADES").asInt();
  bool   use_ior         = (bool) prop.findGroup("ATTN").find("IOR").asInt();
  bool   use_tsb         = (bool) prop.findGroup("ATTN").find("TSB").asInt();
  bool   require_xcheck  = (bool) prop.findGroup("ATTN").find("REQUIRE_XCHECK").asInt();
  bool   allow_mot       = (bool) prop.findGroup("ATTN").find("MOTION").asInt();
  double fixvaldec       = prop.findGroup("ATTN").find("FIX_VAL_DEC").asDouble();
  double sup_n           = prop.findGroup("ATTN").find("SUPSAL_N").asDouble();
  xtm_sim                = (Ipp32f) prop.findGroup("ATTN").find("XTM_SIM").asDouble();
  xisize.height          = prop.findGroup("ATTN").find("XI_HEIGHT").asInt();
  xtsize.width           = prop.findGroup("ATTN").find("XT_WIDTH").asInt();
  xtsize.height          = prop.findGroup("ATTN").find("XT_HEIGHT").asInt();




  //IN PORTS:
  //reused server check port:
  Bottle empty;
  Port inPort_s;
  inPort_s.open("/attnserver_st/input/serv_params");


  //LEFT

  //Sal:
  BufferedPort<Bottle> inPort_sal_l;
  inPort_sal_l.open("/attnserver_st/input/sal_l");
  Network::connect("/attnserver_st/input/serv_params", "/salserver_l/output/serv_params");
  Network::connect("/salserver_l/output/serv_params", "/attnserver_st/input/serv_params");
  BinPortable<SalServerParams> s_server_response_l; 
  inPort_s.write(empty,s_server_response_l);
  SalServerParams ssp_l = s_server_response_l.content();
  std::cout << "SalServer L Probe Response: " << ssp_l.toString() << std::endl;
  Network::disconnect("/attnserver_st/input/serv_params", "/salserver_l/output/serv_params");
  Network::disconnect("/salserver_l/output/serv_params", "/attnserver_st/input/serv_params");
  Network::connect("/attnserver_st/input/sal_l", "/salserver_l/output/sal");
  Network::connect("/salserver_l/output/sal", "/attnserver_st/input/sal_l");

  //IOR:
  BufferedPort<Bottle> inPort_ior_l;
  inPort_ior_l.open("/attnserver_st/input/ior_l");
  Network::connect("/attnserver_st/input/serv_params", "/iorserver_l/output/serv_params");
  Network::connect("/iorserver_l/output/serv_params", "/attnserver_st/input/serv_params");
  BinPortable<IORServerParams> i_server_response_l; 
  inPort_s.write(empty,i_server_response_l);
  IORServerParams isp_l = i_server_response_l.content();
  std::cout << "IORServer L Probe Response: " << isp_l.toString() << std::endl;
  Network::disconnect("/attnserver_st/input/serv_params", "/iorserver_l/output/serv_params");
  Network::disconnect("/iorserver_l/output/serv_params", "/attnserver_st/input/serv_params");
  Network::connect("/attnserver_st/input/ior_l", "/iorserver_l/output/ior");
  Network::connect("/iorserver_l/output/ior", "/attnserver_st/input/ior_l");

  //TSB:
  BufferedPort<Bottle> inPort_tsb_l;
  inPort_tsb_l.open("/attnserver_st/input/tsb_l");
  Network::connect("/attnserver_st/input/serv_params", "/tsbserver_l/output/serv_params");
  Network::connect("/tsbserver_l/output/serv_params", "/attnserver_st/input/serv_params");
  BinPortable<TSBServerParams> t_server_response_l; 
  inPort_s.write(empty,t_server_response_l);
  TSBServerParams tsp_l = t_server_response_l.content();
  std::cout << "TSBServer L Probe Response: " << tsp_l.toString() << std::endl;
  Network::disconnect("/attnserver_st/input/serv_params", "/tsbserver_l/output/serv_params");
  Network::disconnect("/tsbserver_l/output/serv_params", "/attnserver_st/input/serv_params");
  Network::connect("/attnserver_st/input/tsb_l", "/tsbserver_l/output/tsb");
  Network::connect("/tsbserver_l/output/tsb", "/attnserver_st/input/tsb_l");


  //RIGHT

  //Sal:
  BufferedPort<Bottle> inPort_sal_r;
  inPort_sal_r.open("/attnserver_st/input/sal_r");
  Network::connect("/attnserver_st/input/serv_params", "/salserver_r/output/serv_params");
  Network::connect("/salserver_r/output/serv_params", "/attnserver_st/input/serv_params");
  BinPortable<SalServerParams> s_server_response_r; 
  inPort_s.write(empty,s_server_response_r);
  SalServerParams ssp_r = s_server_response_r.content();
  std::cout << "SalServer R Probe Response: " << ssp_r.toString() << std::endl;
  Network::disconnect("/attnserver_st/input/serv_params", "/salserver_r/output/serv_params");
  Network::disconnect("/salserver_r/output/serv_params", "/attnserver_st/input/serv_params");
  Network::connect("/attnserver_st/input/sal_r", "/salserver_r/output/sal");
  Network::connect("/salserver_r/output/sal", "/attnserver_st/input/sal_r");

  //IOR:
  BufferedPort<Bottle> inPort_ior_r;
  inPort_ior_r.open("/attnserver_st/input/ior_r");
  Network::connect("/attnserver_st/input/serv_params", "/iorserver_r/output/serv_params");
  Network::connect("/iorserver_r/output/serv_params", "/attnserver_st/input/serv_params");
  BinPortable<IORServerParams> i_server_response_r; 
  inPort_s.write(empty,i_server_response_r);
  IORServerParams isp_r = i_server_response_r.content();
  std::cout << "IORServer R Probe Response: " << isp_r.toString() << std::endl;
  Network::disconnect("/attnserver_st/input/serv_params", "/iorserver_r/output/serv_params");
  Network::disconnect("/iorserver_r/output/serv_params", "/attnserver_st/input/serv_params");
  Network::connect("/attnserver_st/input/ior_r", "/iorserver_r/output/ior");
  Network::connect("/iorserver_r/output/ior", "/attnserver_st/input/ior_r");

  //TSB:
  BufferedPort<Bottle> inPort_tsb_r;
  inPort_tsb_r.open("/attnserver_st/input/tsb_r");
  Network::connect("/attnserver_st/input/serv_params", "/tsbserver_r/output/serv_params");
  Network::connect("/tsbserver_r/output/serv_params", "/attnserver_st/input/serv_params");
  BinPortable<TSBServerParams> t_server_response_r; 
  inPort_s.write(empty,t_server_response_r);
  TSBServerParams tsp_r = t_server_response_r.content();
  std::cout << "TSBServer R Probe Response: " << tsp_r.toString() << std::endl;
  Network::disconnect("/attnserver_st/input/serv_params", "/tsbserver_r/output/serv_params");
  Network::disconnect("/tsbserver_r/output/serv_params", "/attnserver_st/input/serv_params");
  Network::connect("/attnserver_st/input/tsb_r", "/tsbserver_r/output/tsb");
  Network::connect("/tsbserver_r/output/tsb", "/attnserver_st/input/tsb_r");



  //Original ims for XCHECK:
  //REC:
  Network::connect("/attnserver_st/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/attnserver_st/input/serv_params");
  BinPortable<RecServerParams> r_server_response; 
  inPort_s.write(empty,r_server_response);
  RecServerParams rsp = r_server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;
  Network::disconnect("/attnserver_st/input/serv_params", "/recserver/output/serv_params");
  Network::disconnect("/recserver/output/serv_params", "/attnserver_st/input/serv_params");
  BufferedPort<Bottle> inPort_rec_l;
  inPort_rec_l.open("/attnserver_st/input/rec_l");
  Network::connect("/attnserver_st/input/rec_l", "/recserver/output/left_ye");
  Network::connect("/recserver/output/left_ye", "/attnserver_st/input/rec_l");
  BufferedPort<Bottle> inPort_rec_r;
  inPort_rec_r.open("/attnserver_st/input/rec_r");
  Network::connect("/attnserver_st/input/rec_r", "/recserver/output/right_ye");
  Network::connect("/recserver/output/right_ye", "/attnserver_st/input/rec_r");








 

  // OUTPUT PORTS:

  //server params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/attnserver_st/output/serv_params");

  //ATTN MAP & SACCADE COORDS L:
  BufferedPort<Bottle> outPort_attn_l;
  outPort_attn_l.open("/attnserver_st/output/attn_l");

  //ATTN MAP & SACCADE COORDS R:
  BufferedPort<Bottle> outPort_attn_r;
  outPort_attn_r.open("/attnserver_st/output/attn_r");

  //Motion request port:
  Port outPort_mot;
  outPort_mot.open("/attnserver_st/output/mot");
  Network::connect("/attnserver_st/output/mot", "/recserver/input/motion");
  Network::connect("/recserver/input/motion", "/attnserver_st/output/mot");
  BinPortable<RecMotionRequest> motion_request;


  //initalise:
  motion_request.content().pix_y  = 0;
  motion_request.content().pix_xl = 0;
  motion_request.content().pix_xr = 0;
  motion_request.content().deg_r = 0.0;
  motion_request.content().deg_p = 0.0;
  motion_request.content().deg_y = 0.0;
  motion_request.content().relative = true; //gonna send relative moves!
  motion_request.content().suspend = suspend_cycles; //these moves are high-piriorty, avoid conflicts so pause for this many recserver updates after issue.
  motion_request.content().lockto = NO_LOCK;
  motion_request.content().unlock = true;

  srcsize.width  = ssp_l.width;
  srcsize.height = ssp_l.height;
  xisize.width   = srcsize.width;

  int psb_in = ssp_l.psb;
  int psb_32f,psb;
  Ipp32f* tmp_32f   = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* attn_l    = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* attn_r    = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp8u*  attn_l_8u = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  Ipp8u*  attn_r_8u = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
  xres              = ippiMalloc_32f_C1(xisize.width,xisize.height, &psbxres); //for 'same' as xisize.

  ippiSet_32f_C1R(1.0,attn_l,psb_32f,srcsize); 
  ippiSet_32f_C1R(1.0,attn_r,psb_32f,srcsize); 


  // Make replier for server param probes on server port:
  //Set Params:
  AttnStServerParams asp;
  asp.width=srcsize.width;
  asp.height=srcsize.height;
  asp.psb=psb;
  //Replier:
  AttnStReplyParamProbe server_replier;
  server_replier.reply=asp;
  outPort_s.setReplier(server_replier);


  Bottle *inBot_sal_l,*inBot_tsb_l,*inBot_ior_l,*inBot_rec_l;
  Bottle *inBot_sal_r,*inBot_tsb_r,*inBot_ior_r,*inBot_rec_r;
  Ipp8u *sal_in_l,*tsb_in_l,*ior_in_l,*rec_in_l;
  Ipp8u *sal_in_r,*tsb_in_r,*ior_in_r,*rec_in_r;

  //fixation & filtering vars:
  int fix_x,fix_y,fix_x_l,fix_y_l,fix_x_r,fix_y_r;
  Ipp32f fix_val_l,fix_val_r;
  Ipp32f fix_val,old_fix_val=999.0;
  int spread_x_l, spread_y_l,spread_x_r, spread_y_r;
  int max_x_l,min_x_l,max_y_l,min_y_l,max_x_r,min_x_r,max_y_r,min_y_r;
  int lr_win;
  int targ_x_l=srcsize.width/2,targ_y_l=srcsize.height/2,targ_x_r=srcsize.width/2,targ_y_r=srcsize.height/2;
  double non_z_fv_l=0.0,non_z_fv_r=0.0;
  int non_z_x_l=srcsize.width/2,non_z_y_l=srcsize.height/2;
  int non_z_x_r=srcsize.width/2,non_z_y_r=srcsize.height/2;
  double ang_xl=0.0,ang_xr=0.0,ang_y=0.0;
  int des_xl=0,des_xr=0,des_y=0; 
  double tmp;
  int pf_ind=0;  
  int past_fix_x_l[num_v];
  int past_fix_y_l[num_v];
  int past_fix_x_r[num_v];
  int past_fix_y_r[num_v];
  Ipp32f past_fix_val_l[num_v];
  Ipp32f past_fix_val_r[num_v];
  for (int i=0;i<num_v;i++){
    past_fix_x_l[i] = srcsize.width/2;
    past_fix_y_l[i] = srcsize.height/2;
    past_fix_x_r[i] = srcsize.width/2;
    past_fix_y_r[i] = srcsize.height/2;
  }




  //timing:
  struct timeval t0;
  struct timeval t1;
  struct timeval result;
  struct timezone tz;
  struct timeval t_save;
  struct timezone tz_save;
  gettimeofday(&t0,&tz);


  bool in_vicinity_l = false;
  bool in_vicinity_r = false;
  bool got_sal_l = false;
  bool got_ior_l = false;
  bool got_tsb_l = false;
  bool got_rec_l = false;
  bool got_sal_r = false;
  bool got_ior_r = false;
  bool got_tsb_r = false;
  bool got_rec_r = false;
  bool xcheck    = false;
  bool xcheck_ok = false;









  if (allow_mot){
    //initial position:
    outPort_mot.write(motion_request);
  }






  RecResultParams* rec_res;
  int dpix_y=0;
  int sac=0;





  TCREATE





  //MAIN EVENT LOOP:
  printf("Begin..\n");
  while (1){


    TSTART 



    //try to cache ims until full set arrives:
    while (!got_rec_l ||
           !got_rec_r ||
           !got_sal_l ||
           !got_sal_r ||
           (use_ior && !got_ior_l) ||
           (use_ior && !got_ior_r) ||
           (use_tsb && !got_tsb_l) ||
           (use_tsb && !got_tsb_r) ){
        
        

        //request all required images:
        if (use_tsb){
            if (!got_tsb_r){inBot_tsb_r = inPort_tsb_r.read(false);}
            if (!got_tsb_l){inBot_tsb_l = inPort_tsb_l.read(false);}
        }
        if (use_ior){
            if (!got_ior_r){inBot_ior_r = inPort_ior_r.read(false);}
            if (!got_ior_l){inBot_ior_l = inPort_ior_l.read(false);}
        }
        if (!got_sal_r){inBot_sal_r = inPort_sal_r.read(false);}
        if (!got_sal_l){inBot_sal_l = inPort_sal_l.read(false);}
        if (!got_rec_r){inBot_rec_r = inPort_rec_r.read(false);}
        if (!got_rec_l){inBot_rec_l = inPort_rec_l.read(false);}
        

        usleep(5000);
        
        
        //cache any images that arrive ONCE:
        if (!got_tsb_r && inBot_tsb_r!=NULL && use_tsb){
            tsb_in_r = (Ipp8u*) inBot_tsb_r->get(0).asBlob();
            ippiConvert_8u32f_C1R(tsb_in_r,psb_in,tmp_32f,psb_32f,srcsize); 
            ippiMul_32f_C1IR(tmp_32f,psb_32f,attn_r,psb_32f,srcsize); 
            got_tsb_r=true; 
        }
        if (!got_tsb_l && inBot_tsb_l!=NULL && use_tsb){
            tsb_in_l = (Ipp8u*) inBot_tsb_l->get(0).asBlob();
            ippiConvert_8u32f_C1R(tsb_in_l,psb_in,tmp_32f,psb_32f,srcsize); 
            ippiMul_32f_C1IR(tmp_32f,psb_32f,attn_l,psb_32f,srcsize); 
            got_tsb_l=true; 
        }
        if (!got_ior_r && inBot_ior_r!=NULL && use_ior){
            ior_in_r = (Ipp8u*) inBot_ior_r->get(0).asBlob();
            ippiConvert_8u32f_C1R(ior_in_r,psb_in,tmp_32f,psb_32f,srcsize);
            ippiMul_32f_C1IR(tmp_32f,psb_32f,attn_r,psb_32f,srcsize); 
            got_ior_r=true; 
        }
        if (!got_ior_l && inBot_ior_l!=NULL && use_ior){
            ior_in_l = (Ipp8u*) inBot_ior_l->get(0).asBlob();
            ippiConvert_8u32f_C1R(ior_in_l,psb_in,tmp_32f,psb_32f,srcsize);
            ippiMul_32f_C1IR(tmp_32f,psb_32f,attn_l,psb_32f,srcsize); 
            got_ior_l=true; 
        }
        if (!got_sal_r && inBot_sal_r!=NULL){
            sal_in_r = (Ipp8u*) inBot_sal_r->get(0).asBlob();
            ippiConvert_8u32f_C1R(sal_in_r,psb_in,tmp_32f,psb_32f,srcsize);
            ippiMul_32f_C1IR(tmp_32f,psb_32f,attn_r,psb_32f,srcsize); 
            got_sal_r=true; 
        }
        if (!got_sal_l && inBot_sal_l!=NULL){
            sal_in_l = (Ipp8u*) inBot_sal_l->get(0).asBlob();
            ippiConvert_8u32f_C1R(sal_in_l,psb_in,tmp_32f,psb_32f,srcsize); 
            ippiMul_32f_C1IR(tmp_32f,psb_32f,attn_l,psb_32f,srcsize); 
            got_sal_l=true; 
        }

        if (!got_rec_r && inBot_rec_r!=NULL){
            rec_in_r = (Ipp8u*) inBot_rec_r->get(0).asBlob();
            rec_res = (RecResultParams*) inBot_rec_r->get(1).asBlob();
            dpix_y = rec_res->ly-rec_res->ry;
            got_rec_r=true; 
        }
        if (!got_rec_l && inBot_rec_l!=NULL){
            rec_in_l = (Ipp8u*) inBot_rec_l->get(0).asBlob();
            rec_res = (RecResultParams*) inBot_rec_l->get(1).asBlob();
            dpix_y = rec_res->ly-rec_res->ry;
            got_rec_l=true; 
        }
        
    }
    





    //If here, we have cached all required maps:

    //
    // POSTPROCESSING: saccade filtering and coordinates.
    //
    
    
    
    
    
    //FIND FIXATION PEEK FOR CONSIDERATION:
    //max Fixation:
    ippiMaxIndx_32f_C1R(attn_r,psb_32f,srcsize,&fix_val_r,&fix_x_r,&fix_y_r);
    ippiMaxIndx_32f_C1R(attn_l,psb_32f,srcsize,&fix_val_l,&fix_x_l,&fix_y_l);
#if INFO
    printf("MAX_FIXVAL_L: %f  @(%d,%d)\n",(float)fix_val_l,fix_x_l,fix_y_l);
    printf("MAX_FIXVAL_R: %f  @(%d,%d)\n",(float)fix_val_r,fix_x_r,fix_y_r);
#endif
    
    if (fix_val_l!=0.0){
        non_z_fv_l = fix_val_l;
        non_z_x_l  = fix_x_l;
        non_z_y_l  = fix_y_l;
    }
    if (fix_val_r!=0.0){
        non_z_fv_r = fix_val_r;
        non_z_x_r  = fix_x_r;
        non_z_y_r  = fix_y_r;
    }
    
    
    //keep last num_v locations in ring buffer:
    past_fix_x_l[pf_ind]   = fix_x_l;
    past_fix_y_l[pf_ind]   = fix_y_l;
    past_fix_val_l[pf_ind] = fix_val_l;
    past_fix_x_r[pf_ind]   = fix_x_r;
    past_fix_y_r[pf_ind]   = fix_y_r;
    past_fix_val_r[pf_ind] = fix_val_r;
    pf_ind++;
    if (pf_ind >= num_v){ pf_ind=0; }
    
    
    //check spread of last num_v conseccutive most salient locations:
    max_x_l = -999;
    min_x_l = 999;
    max_y_l = -999;
    min_y_l = 999;
    max_x_r = -999;
    min_x_r = 999;
    max_y_r = -999;
    min_y_r = 999;
    for (int i=0;i<num_v;i++){
        if (past_fix_x_l[i]>max_x_l ){ max_x_l = past_fix_x_l[i];}
        if (past_fix_x_l[i]<min_x_l ){ min_x_l = past_fix_x_l[i];}
        if (past_fix_y_l[i]>max_y_l ){ max_y_l = past_fix_y_l[i];}
        if (past_fix_y_l[i]<min_y_l ){ min_y_l = past_fix_y_l[i];}
        if (past_fix_x_r[i]>max_x_r ){ max_x_r = past_fix_x_r[i];}
        if (past_fix_x_r[i]<min_x_r ){ min_x_r = past_fix_x_r[i];}
        if (past_fix_y_r[i]>max_y_r ){ max_y_r = past_fix_y_r[i];}
        if (past_fix_y_r[i]<min_y_r ){ min_y_r = past_fix_y_r[i];}
    }
    spread_x_l = max_x_l - min_x_l;
    spread_y_l = max_y_l - min_y_l;
    in_vicinity_l = false;
    if (spread_x_l<=spread && 
        spread_y_l<=spread ){
        in_vicinity_l = true;
    }
    spread_x_r = max_x_r - min_x_r;
    spread_y_r = max_y_r - min_y_r;
    in_vicinity_r = false;
    if (spread_x_r<=spread && 
        spread_y_r<=spread ){
        in_vicinity_r = true;
    }
    
    
    
    
    
    //record whichever is higher, left or right:
    if (fix_val_l>fix_val_r){
        lr_win = ALEFT;
        fix_val = fix_val_l;
        fix_x = fix_x_l;
        fix_y = fix_y_l;
    }
    else{
        lr_win = ARIGHT;
        fix_val = fix_val_r;
        fix_x = fix_x_r;
        fix_y = fix_y_r;
    }
    
    
    
    
    
    
    
    
    
    
    
    
    //
    //SACCADE FILTERING
    //
    
    if (filter_saccades){   
        //WE DON'T JUST WANT TO GAZE AT MOST SALIENT POINT,
        //BECAUSE IT JUMPS AROUND ALOT.
        //SO WE FILTER A BIT:
        
        //WE MRFZDF TRACK WHILE WE CAN, BUT IF 
        //SOMETHING SALIENT CROPS UP, WE SACCADE GAZE TO THAT!
        //IF NO SACCADE WITHIN T SECS, JUST SACCADE TO THE LAST
        //WINNING FIXVAL
        
        //get time since last saccade:
        gettimeofday(&t1,&tz);
        timeval_subtract(&result, &t1, &t0);
        
        
        //VERY SALIENT DISTRACTIONS:
        //if a point is N TIMES as salient as the last thing that 
        //caused saccade (don't forget, we decay old_fix_val!):
        if ( fix_val > sup_n*old_fix_val && old_fix_val!=0.0){
#if INFO
            printf("**SUPERSALIENT -  new:%f old:%f\n",fix_val,old_fix_val);
#endif
            //if so, schedule saccade to its coords!:  
            old_fix_val = fix_val;	    
            //params still set, so just schedule crosscheck:
            xcheck=true;
        }
        
        
        //CLUSTERED DISTRACTIONS:
        //move after num_v conseccutive salient distractions 
        //in the same vicinity:
        else if ((in_vicinity_l && fix_val_l!=0.0)||(in_vicinity_r && fix_val_r!=0.0)){
            
            //first, if both l and r in vicinity, pick highest fix_val:
            if (in_vicinity_l && in_vicinity_r){
                if (fix_val_l > fix_val_r){
                    in_vicinity_r = false;
                }
                else{
                    in_vicinity_l = false;
                }
            }
            
            if (in_vicinity_l){
#if INFO
                printf("**CLUSTER L - %d consec peaks within %d pix. fix:%f\n",num_v,spread,fix_val_l);
#endif
                //set old_fix_val to highest in cluster:
                tmp=0.0;
                for (int i=0;i<num_v;i++){
                    if (past_fix_val_l[i]>tmp){tmp=past_fix_val_l[i];}
                }
                
                //schedule saccade to left peek:
                lr_win = ALEFT;
                fix_x = fix_x_l;
                fix_y = fix_y_l;
                fix_val = fix_val_l;
                old_fix_val = tmp;
            }
            
            else{//in_vicinity_r
#if INFO
                printf("**CLUSTER R - %d consec peaks within %d pix. fix:%f\n",num_v,spread,fix_val_r);
#endif
                //set old_fix_val to highest in cluster:
                tmp=0.0;
                for (int i=0;i<num_v;i++){
                    if (past_fix_val_r[i]>tmp){tmp=past_fix_val_r[i];}
                }
                
                //schedule saccade to right peek:
                lr_win = ARIGHT;
                fix_x = fix_x_r;
                fix_y = fix_y_r;
                fix_val = fix_val_r;
                old_fix_val = tmp;
            }
            
            //schedule xcheck:
            xcheck=true;
        }
        
        //OTHERWISE, IF NO SHIFTS FOR A WHILE
        //DO A TIMED GAZE SHIFT:
        else if (result.tv_sec >= wait_time){
            
            //move to the last non-0.0 fix peak:
            if (non_z_fv_l>non_z_fv_r){
                lr_win = ALEFT;
                fix_x = non_z_x_l;
                fix_y = non_z_y_l;
                fix_val = 0.0;
                old_fix_val = non_z_fv_l;
#if INFO
                printf("**TIMEDSHIFT - %ds %dus > %ds fix:%f\n",
                       result.tv_sec,result.tv_usec,wait_time,non_z_fv_l);
#endif	  
            }
            else{
                lr_win = ARIGHT;
                fix_x = non_z_x_r;
                fix_y = non_z_y_r;
                fix_val = 0.0;
                old_fix_val = non_z_fv_r;
#if INFO
                printf("**TIMEDSHIFT - %ds %dus > %ds fix:%f\n",
                       result.tv_sec,result.tv_usec,wait_time,non_z_fv_r);
#endif	  
            }
            
            //schedule xcheck:
            xcheck=true;
        }
        
    }//END: filter saccades 
    
    
    
    else{//Don't Filter Saccades:
        //no saccade filtering, just look at the most salient
        //point if it's bigger than that which caused previous saccade (less decay):
        if (fix_val!=0.0 && fix_val>=old_fix_val){
            old_fix_val = fix_val;
            //params still set, so just schedule xcheck:
            xcheck = true;
            sleep(1); //sleep a little so we don't go saccade-crazy!
        }
        //otherwise, don't saccade.
    }//END: no saccade filtering
    
    
    
    
    
    //X-CHECKING:
    //if a saccade destination candidate has been scheduled for pre-saccade xchecking: 
    if(xcheck){
        
        //xcheck and determine targ_x_l, targ_y, targ_x_r:
        //left is a bit higher, sub a little for init point.
        if (lr_win==ALEFT){
            //do stereo cross-checking for des l: ONLY DO THIS IF NO STEREO MAP! (LATER)
            targ_x_l = fix_x_l;
            targ_y_l = fix_y_l;
            targ_y_r = targ_y_l+dpix_y;
            xcheck_ok = crosscheck(rec_in_l,targ_x_l,targ_y_l,rec_in_r,&targ_x_r,targ_y_r,psb_in);
        }
        else{
            //do stereo cross-checking for des l: ONLY DO THIS IF NO STEREO MAP! (LATER)
            targ_x_r = fix_x_r;
            targ_y_r = fix_y_r;
            targ_y_l = targ_y_r-dpix_y;
            xcheck_ok = crosscheck(rec_in_r,targ_x_r,targ_y_r,rec_in_l,&targ_x_l,targ_y_l,psb_in);
        }
        
        
        
        
        
        if (!xcheck_ok){
            if(require_xcheck){
                lr_win = ANONE;
                //no motion as xcheck_ok == false;
#if INFO
                printf("XCHECK FAIL, STIMULUS IGNORED\n");
#endif
            }
            else{
                //just move both eyes same amount:
                if (lr_win==ALEFT){
                    targ_x_r = targ_x_l;
                    targ_y_r = targ_y_l;
                }
                else{
                    targ_x_l = targ_x_r;
                    targ_y_l = targ_y_r;
                }
                //cause motion:
                xcheck_ok = true;
#if INFO
                printf("XCHECK FAIL, MOVING BOTH EYES SAME\n");
#endif
            }
        }
        
        
    }
    //no peek worth checking:
    else{
        lr_win = ANONE;
        //prevent motion:
        xcheck_ok = false;
    }
    
    
    
    
    
    
    
    
    //MOTION!!!
    if (xcheck_ok){
        
        //set desired relative angular saccades:
        des_xl = targ_x_l-srcsize.width/2;
        des_xr = targ_x_r-srcsize.width/2;
        if (lr_win==ALEFT){
            des_y  = targ_y_l-srcsize.height/2;
        }
        else{
            des_y  = targ_y_r-srcsize.height/2;
        }


#if INFO
        printf("DESIRED RELATIVE SACCADE: xl:%d xr:%d y:%d\n", des_xl,des_xr,des_y);
#endif     
        //MOVE!! 
        if (allow_mot){
            motion_request.content().pix_xl = des_xl;
            motion_request.content().pix_xr = des_xr;
            motion_request.content().pix_y  = des_y;
            outPort_mot.write(motion_request);
            sac++;
        
            //because we did a saccade, we need to adjust 
            //filter cache entries to new origin: 
            if (filter_saccades){
                //adjust cluster stuff:
                for (int i=0;i<num_v;i++){
                    past_fix_x_l[i] += des_xl;
                    past_fix_y_l[i] += des_y;
                    past_fix_x_r[i] += des_xr;
                    past_fix_y_r[i] += des_y;
                }
                //adjust last non-zero to previous point:
                non_z_x_l += targ_x_l-srcsize.width/2;
                non_z_y_l += targ_y_l-srcsize.height/2;
                non_z_x_r += targ_x_r-srcsize.width/2;
                non_z_y_r += targ_y_r-srcsize.height/2;
            }
        }

        //reset 'time-since-saccade' timer
        gettimeofday(&t0,&tz);
        
    }
    
    
    
    
    //YARP OUTPUT:
    
    //prepare output:
    conv_32f_to_8u(attn_l,psb_32f,attn_l_8u,psb,srcsize);
    //send it on its way:
    Bottle& tmpBot_l = outPort_attn_l.prepare();
    tmpBot_l.clear();
    tmpBot_l.add(Value::makeBlob( attn_l_8u, psb*srcsize.height));
    tmpBot_l.addInt(fix_x_l);
    tmpBot_l.addInt(fix_y_l);
    tmpBot_l.addInt(targ_x_l);
    tmpBot_l.addInt(targ_y_l);
    tmpBot_l.addInt(lr_win);
    tmpBot_l.addInt(sac);
    outPort_attn_l.write();
    
    //prepare output:
    conv_32f_to_8u(attn_r,psb_32f,attn_r_8u,psb,srcsize);
    //send it on its way:
    Bottle& tmpBot_r = outPort_attn_r.prepare();
    tmpBot_r.clear();
    tmpBot_r.add(Value::makeBlob( attn_r_8u, psb*srcsize.height));
    tmpBot_r.addInt(fix_x_r);
    tmpBot_r.addInt(fix_y_r);
    tmpBot_r.addInt(targ_x_r);
    tmpBot_r.addInt(targ_y_r);
    tmpBot_r.addInt(lr_win);
    tmpBot_r.addInt(sac);
    outPort_attn_r.write();
    
    
    
    
    //Prepare for next iteration:
    
    //decay old_fix_val so gaze doesn't get 
    //stuck if no fix change for a while:
    old_fix_val*=fixvaldec;
    
    //reset:
    ippiSet_32f_C1R(1.0,attn_l,psb_32f,srcsize); 
    ippiSet_32f_C1R(1.0,attn_r,psb_32f,srcsize); 
    got_tsb_l = false;
    got_ior_l = false;
    got_sal_l = false;
    got_rec_l = false;
    got_tsb_r = false;
    got_ior_r = false;
    got_sal_r = false;
    got_rec_r = false;
    xcheck    = false;
    xcheck_ok = false;
    
    
    
    TSTOP
      
  }
  
  
  //never here...
  TEND
}







bool iCub::contrib::primateVision::AttnServer_Stereo::crosscheck(Ipp8u*ima,int x_a,int y_a,Ipp8u*imb,int*x_b,int y_b,int psbx){

  bool success=false;
  Ipp32f max_32f;
  int foo;

  //if not outside image bounds:
  if ( (x_a-xtsize.width/2-1  > 0)              && 
       (x_a+xtsize.width/2+1  < srcsize.width)  && 
       (y_b-xisize.height/2-1 > 0)              && 
       (y_b+xisize.height/2+1 < srcsize.height) ){ 

    //cut out template from image A at max fix point,
    //search for it in image B across full image in band around im A fix coords:
    ippiCrossCorrSame_NormLevel_8u32f_C1R(&imb[0 + (y_b-xisize.height/2)*psbx],
					  psbx,
					  xisize,
					  &ima[(x_a-xtsize.width/2) + (y_a-xtsize.height/2)*psbx],
					  psbx,
					  xtsize,
					  xres,
					  psbxres);

    //peek:  
    ippiMaxIndx_32f_C1R(xres,psbxres,xisize,&max_32f,x_b,&foo);
    
#if INFO
    printf("XCHECK - ax:%d ay:%d bx:%d res:%f\n",x_a,y_a,*x_b,max_32f);
#endif
    
    if (max_32f>xtm_sim){
      success = true;
    }
  }
  else{
    *x_b=x_a;
#if INFO
    printf("XCHECK - OUT OF BOUNDS\n");
#endif
  }
  
  return success;

}
