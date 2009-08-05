/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <qapplication.h>
#include <qstring.h>
#include <ipp.h>
#include <string>
//My includes:
#include <timing.h>
#include <convert_bitdepth.h>
//Client of:
#include <iorio.h>
#include <tsbio.h>
#include <salio.h>
//for motion:
#include <recio.h>

#include "attn.h"


#define INFO 1  //print info?




iCub::contrib::primateVision::AttnServer::AttnServer(string*c_)
{
  

  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::AttnServer::~AttnServer()
{
  
}


void iCub::contrib::primateVision::AttnServer::run(){


  Property prop;
  prop.fromConfigFile(cfg->c_str());

  int    input           = prop.findGroup("ATTN").find("INPUT").asInt();
  int    num_v           = prop.findGroup("ATTN").find("NUM_CLUST").asInt();
  int    spread          = prop.findGroup("ATTN").find("SPREAD_CLUST").asInt();
  int    wait_time       = prop.findGroup("ATTN").find("TIMED_SHIFT").asInt();;
  bool   filter_saccades = (bool) prop.findGroup("ATTN").find("FILTER_SACCADES").asInt();
  double fixvaldec       = prop.findGroup("ATTN").find("FIX_VAL_DEC").asDouble();
  double sup_n           = prop.findGroup("ATTN").find("SUPSAL_N").asDouble();
  int    allow_mot       = prop.findGroup("ATTN").find("MOTION").asInt();
  bool    use_ior        = (bool) prop.findGroup("ATTN").find("IOR").asInt();
  bool    use_tsb        = (bool) prop.findGroup("ATTN").find("TSB").asInt();



  QString in_source;
  if (input==0){in_source="l";}
  else{in_source="r";}


  //IN PORTS:
  //reused server check port:
  Bottle empty;
  Port inPort_s;
  inPort_s.open("/attnserver_"+in_source+"/input/serv_params");


  //Sal:
  BufferedPort<Bottle> inPort_sal;
  inPort_sal.open("/attnserver_"+in_source+"/input/sal");
  Network::connect("/attnserver_"+in_source+"/input/serv_params", "/salserver_"+in_source+"/output/serv_params");
  Network::connect("/salserver_"+in_source+"/output/serv_params", "/attnserver_"+in_source+"/input/serv_params");
  BinPortable<SalServerParams> s_server_response; 
  inPort_s.write(empty,s_server_response);
  SalServerParams ssp = s_server_response.content();
  std::cout << "SalServer Probe Response: " << ssp.toString() << std::endl;
  Network::disconnect("/attnserver_"+in_source+"/input/serv_params", "/salserver_"+in_source+"/output/serv_params");
  Network::disconnect("/salserver_"+in_source+"/output/serv_params", "/attnserver_"+in_source+"/input/serv_params");

  Network::connect("/attnserver_"+in_source+"/input/sal", "/salserver_"+in_source+"/output/sal");
  Network::connect("/salserver_"+in_source+"/output/sal", "/attnserver_"+in_source+"/input/sal");

  //IOR:
  BufferedPort<Bottle> inPort_ior;
  inPort_ior.open("/attnserver_"+in_source+"/input/ior");
  Network::connect("/attnserver_"+in_source+"/input/serv_params", "/iorserver_"+in_source+"/output/serv_params");
  Network::connect("/iorserver_"+in_source+"/output/serv_params", "/attnserver_"+in_source+"/input/serv_params");
  BinPortable<IORServerParams> i_server_response; 
  inPort_s.write(empty,i_server_response);
  IORServerParams isp = i_server_response.content();
  std::cout << "IORServer Probe Response: " << isp.toString() << std::endl;
  Network::disconnect("/attnserver_"+in_source+"/input/serv_params", "/iorserver_"+in_source+"/output/serv_params");
  Network::disconnect("/iorserver_"+in_source+"/output/serv_params", "/attnserver_"+in_source+"/input/serv_params");
  
  Network::connect("/attnserver_"+in_source+"/input/ior", "/iorserver_"+in_source+"/output/ior");
  Network::connect("/iorserver_"+in_source+"/output/ior", "/attnserver_"+in_source+"/input/ior");
  
  //TSB:
  BufferedPort<Bottle> inPort_tsb;
  inPort_tsb.open("/attnserver_"+in_source+"/input/tsb");
  Network::connect("/attnserver_"+in_source+"/input/serv_params", "/tsbserver_"+in_source+"/output/serv_params");
  Network::connect("/tsbserver_"+in_source+"/output/serv_params", "/attnserver_"+in_source+"/input/serv_params");
  BinPortable<TSBServerParams> t_server_response; 
  inPort_s.write(empty,t_server_response);
  TSBServerParams tsp = t_server_response.content();
  std::cout << "TSBServer Probe Response: " << tsp.toString() << std::endl;
  Network::disconnect("/attnserver_"+in_source+"/input/serv_params", "/tsbserver_"+in_source+"/output/serv_params");
  Network::disconnect("/tsbserver_"+in_source+"/output/serv_params", "/attnserver_"+in_source+"/input/serv_params");
  
  Network::connect("/attnserver_"+in_source+"/input/tsb", "/tsbserver_"+in_source+"/output/tsb");
  Network::connect("/tsbserver_"+in_source+"/output/tsb", "/attnserver_"+in_source+"/input/tsb"); 







  // OUTPUT PORTS:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/attnserver_"+in_source+"/output/serv_params");

  //ATTN MAP & SACCADE COORDS:
  BufferedPort<Bottle> outPort_attn;
  outPort_attn.open("/attnserver_"+in_source+"/output/attn");


  //Motion request port:
  Port outPort_mot;
  outPort_mot.open("/attnserver_"+in_source+"/output/mot");
  Network::connect("/attnserver_"+in_source+"/output/mot", "/recserver/input/motion");
  Network::connect("/recserver/input/motion", "/attnserver_"+in_source+"/output/mot");
  BinPortable<RecMotionRequest> motion_request;

  //initalise:
  motion_request.content().pix_y  = 0;
  motion_request.content().pix_xl = 0;
  motion_request.content().pix_xr = 0;
  motion_request.content().deg_r = 0.0;
  motion_request.content().deg_p = 0.0;
  motion_request.content().deg_y = 0.0;
  motion_request.content().relative = true; //gonna send relative moves!
  motion_request.content().suspend = 100;
  motion_request.content().lockto = NO_LOCK;
  motion_request.content().unlock = true;



  IppiSize srcsize;
  srcsize.width = ssp.width;
  srcsize.height = ssp.height;
  int psb_in = ssp.psb;

  int psb_32f,psb;
  Ipp32f* tmp_32f  = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp32f* attn     = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
  Ipp8u*  attn_out = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);





  // Make replier for server param probes on server port:
  //Set Params:
  AttnServerParams asp;
  asp.width=srcsize.width;
  asp.height=srcsize.height;
  asp.psb=psb;
  //Replier:
  AttnReplyParamProbe server_replier;
  server_replier.reply=asp;
  outPort_s.setReplier(server_replier);


  Bottle *inBot_sal,*inBot_tsb,*inBot_ior;
  Ipp8u *sal_in,*tsb_in,*ior_in;

  //fixation & filtering vars:
  int fix_x,fix_y;
  Ipp32f fix_val,old_fix_val=999.0;
  int spread_x, spread_y;
  int max_x,min_x,max_y,min_y;
  int des_xl=0,des_xr=0,des_y=0;
  double ang_xl=0.0,ang_xr=0.0,ang_y=0.0;
  double tmp;
  int pf_ind=0;
  bool in_vicinity = false;
  double non_z_fv=0.0;
  int non_z_x=srcsize.width/2,non_z_y=srcsize.height/2;
  int past_fix_x[num_v];
  int past_fix_y[num_v];
  Ipp32f past_fix_val[num_v];
  for (int i=0;i<num_v;i++){
    past_fix_x[i] = srcsize.width/2;
    past_fix_y[i] = srcsize.height/2;
  }




  //timing:
  struct timeval t0;
  struct timeval t1;
  struct timeval result;
  struct timezone tz;
  struct timeval t_save;
  struct timezone tz_save;
  gettimeofday(&t0,&tz);


  bool got_sal = false;
  bool got_ior = false;
  bool got_tsb = false;
  bool sac=false;




  ippiSet_32f_C1R(1.0,attn,psb_32f,srcsize); 






  if (allow_mot){
    //initial position:
    outPort_mot.write(motion_request);
  }










  TCREATE



  //MAIN EVENT LOOP:
  printf("Begin..\n");
  while (1){


    TSTART 


    if (use_tsb){ inBot_tsb = inPort_tsb.read(false);}
    if (use_ior){ inBot_ior = inPort_ior.read(false);}   
    inBot_sal = inPort_sal.read();


    //get sal image:
     if (!got_sal && inBot_sal!=NULL){
      sal_in = (Ipp8u*) inBot_sal->get(0).asBlob();
      ippiConvert_8u32f_C1R(sal_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMul_32f_C1IR(tmp_32f,psb_32f,attn,psb_32f,srcsize); 
      got_sal=true; 
      printf("got sal!\n");     
    }
    //get tsb image:
    if (!got_tsb && inBot_tsb!=NULL && use_tsb){
      tsb_in = (Ipp8u*) inBot_tsb->get(0).asBlob();
      ippiConvert_8u32f_C1R(tsb_in,psb_in,tmp_32f,psb_32f,srcsize); 
      ippiMul_32f_C1IR(tmp_32f,psb_32f,attn,psb_32f,srcsize); 
      got_tsb=true; 
      //printf("got tsb!\n");
    }
    //get ior image:
    if (!got_ior && inBot_ior!=NULL && use_ior){
      ior_in = (Ipp8u*) inBot_ior->get(0).asBlob();
      ippiConvert_8u32f_C1R(ior_in,psb_in,tmp_32f,psb_32f,srcsize);
      ippiMul_32f_C1IR(tmp_32f,psb_32f,attn,psb_32f,srcsize); 
      got_ior=true; 
      //printf("got ior!\n");
    }




    //
    // POSTPROCESSING: saccade filtering and coordinates.
    //


    //Do we have all maps?:
    if (got_tsb==use_tsb && 
	got_ior==use_ior &&
	got_sal ){
      
      //FIND FIXATION PEEK FOR CONSIDERATION:
      //max Fixation:
      ippiMaxIndx_32f_C1R(attn,psb_32f,srcsize,&fix_val,&fix_x,&fix_y);
#if INFO
      printf("FIXVAL: %f  @(x,y): (%d,%d)\n",fix_val,fix_x,fix_y);
#endif
      
      if (fix_val!=0.0){
	non_z_fv = fix_val;
	non_z_x  = fix_x;
	non_z_y  = fix_y;
      }
      
      
      //keep last num_v locations in ring buffer:
      past_fix_x[pf_ind]   = fix_x;
      past_fix_y[pf_ind]   = fix_y;
      past_fix_val[pf_ind] = fix_val;
      pf_ind++;
      if (pf_ind >=num_v){ pf_ind=0; }
      
      
      //check spread of last num_v conseccutive most salient locations:
      max_x = -999;
      min_x = 999;
      max_y = -999;
      min_y = 999;
      for (int i=0;i<num_v;i++){
	if (past_fix_x[i]>max_x ){ max_x = past_fix_x[i];}
	if (past_fix_x[i]<min_x ){ min_x = past_fix_x[i];}
	if (past_fix_y[i]>max_y ){ max_y = past_fix_y[i];}
	if (past_fix_y[i]<min_y ){ min_y = past_fix_y[i];}
      }
      spread_x = max_x - min_x;
      spread_y = max_y - min_y;
      in_vicinity = false;
      if (spread_x<=spread && 
	  spread_y<=spread   ){
	in_vicinity = true;
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
	
	//get time:
	gettimeofday(&t1,&tz);
	timeval_subtract(&result, &t1, &t0);
	
	
	//VERY SALIENT DISTRACTIONS:
	//if a point is N TIMES as salient as the last thing that 
	//caused saccade (don't forget, we decay old_fix_val!):
	if ( fix_val > sup_n*old_fix_val && old_fix_val!=0.0){
#if INFO
	  printf("**SUPERSALIENT -  new:%f old:%f\n",fix_val,old_fix_val);
#endif
	  sac=true;
	  old_fix_val = fix_val;
	}
	
	
	//CLUSTERED DISTRACTIONS:
	//move after num_v conseccutive salient distractions 
	//in the same vicinity:
	else if (in_vicinity && fix_val!=0.0){
#if INFO
	  printf("**CLUSTER - %d consec peaks within %d pix. fix:%f\n",num_v,spread,fix_val);
#endif
	  //set old_fix_val to highest in cluster:
	  tmp=0.0;
	  for (int i=0;i<num_v;i++){
	    if (past_fix_val[i]>tmp){tmp=past_fix_val[i];}
	  }
	  sac=true;
	  old_fix_val = tmp;
	}
	
	
	//OTHERWISE, IF NO SHIFTS FOR A WHILE
	//DO A TIMED GAZE SHIFT:
	else if ( result.tv_sec >= wait_time){
	  //move to the last non-0.0 fix peak:
#if INFO
	  printf("**TIMEDSHIFT - %ds %dus > %ds fix:%f\n",
		 result.tv_sec,result.tv_usec,wait_time,non_z_fv);
#endif
	  fix_x = non_z_x;
	  fix_y = non_z_y;
	  old_fix_val = non_z_fv;	  
	  sac=true;
	}
	
      }//END: filter saccades
      
      
      
      else{//Don't Filter Saccades:
	//no saccade filtering, just look at the most salient
	//point if it's bigger than that which caused previous saccade:
	if (fix_val!=0.0 && fix_val>=old_fix_val){
	  old_fix_val = fix_val;
	  sac=true;
	  //sleep(1); //sleep a little so we don't go saccade-crazy!
	}
      }//END: no saccade filtering
      



 




 
      //prepare output:
      conv_32f_to_8u(attn,psb_32f,attn_out,psb,srcsize);     
      //send it on its way:
      Bottle& tmpBot = outPort_attn.prepare();
      tmpBot.clear();
      tmpBot.add(Value::makeBlob( attn_out, psb*srcsize.height));
      tmpBot.addInt(fix_x);
      tmpBot.addInt(fix_y);
      outPort_attn.write();
       







       



      //Motion:
      if (sac && allow_mot){
	if (input==0){
	  des_xl = fix_x-srcsize.width/2;
	  des_xr = 0;
	}
	else{
	  des_xr = fix_x-srcsize.width/2;
	  des_xl = 0;
	}
	des_y  = fix_y-srcsize.height/2;
	
#if INFO
	printf("DESIRED RELATIVE SACCADE: xl:%d xr:%d y:%d\n", des_xl,des_xr,des_y);
#endif      		
	
	
	motion_request.content().pix_xl = des_xl;
	motion_request.content().pix_xr = des_xr;
	motion_request.content().pix_y  = des_y;
	outPort_mot.write(motion_request);
	
		  
	//because we did a saccade, we need to adjust 
	//filter cache entries to new origin: 	
	if (filter_saccades){	  
	  //adjust cluster stuff:
	  for (int i=0;i<num_v;i++){
	    past_fix_x[i] += fix_x-srcsize.width/2;
	    past_fix_y[i] += fix_y-srcsize.height/2;
	  }
	  //adjust last non-zero to previous point:
	  non_z_x += fix_x-srcsize.width/2;
	  non_z_y += fix_y-srcsize.height/2;
	}
	//reset 'time-since-saccade' timer
	gettimeofday(&t0,&tz);
      }
      
      
      
      //decay old_fix_val so gaze doesn't get 
      //stuck if no fix change for a while:
      old_fix_val*=fixvaldec;
      
      //reset:
      ippiSet_32f_C1R(1.0,attn,psb_32f,srcsize); 
      got_tsb = false;
      got_ior = false;
      got_sal = false;
      sac=false;

 

    }
    else if (inBot_sal == NULL && 
	     inBot_ior == NULL && 
	     inBot_tsb == NULL ){
      printf("No Input\n");
      usleep(10000);//don't blow out port
    }
    
    
    TSTOP
    
  }

  //never here...
  TEND

}
