/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <iostream>
#include <math.h> 

//MY INCLUDES
#include <timing.h>
#include <dog.h>
#include <coord.h>
#include <norm.h>
#include <multiclass.h>
//THIS APP IS A RECCLIENT!!!
#include <recio.h>
#include "zdf.h"


//use NDT or RANK comparision?
#define RANK0_NDT1 1 

//NDT:
#define NDTX     1
#define NDTY     1
#define NDTSIZE  8 //4 or 8
#define NDTEQ    3 //0

//RANK:
#define RANKX    1 //1 or 2
#define RANKY    1
#define RANKSIZE 9 //9 or 15
//RANKSISE = (RANKX*2+1)*(RANKY*2+1) //e.g., 15 for (2,1) :)


#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

#define INFO 1



iCub::contrib::primateVision::ZDFServer::ZDFServer(string*c_)
{
  
  cfg = c_;
  
  start();
  
}

iCub::contrib::primateVision::ZDFServer::~ZDFServer()
{
  
}


void iCub::contrib::primateVision::ZDFServer::run(){
  
  
  Property prop; 
  prop.fromConfigFile(cfg->c_str());
  
  struct MultiClass::Parameters params;
  params.iter_max                  = prop.findGroup("ZDF").find("MAX_ITERATIONS").asInt();
  params.randomize_every_iteration = prop.findGroup("ZDF").find("RANDOMIZE_EVERY_ITER").asInt();
  params.smoothness_penalty_base   = prop.findGroup("ZDF").find("SMOOTHNESS_PENALTY_BASE").asInt();
  params.smoothness_penalty        = prop.findGroup("ZDF").find("SMOOTHNESS_PENALTY").asInt();
  params.data_penalty              = prop.findGroup("ZDF").find("DATA_PENALTY").asInt();
  params.smoothness_3sigmaon2      = prop.findGroup("ZDF").find("SMOOTHNESS_3SIGMAON2").asInt();
  int radial_penalty   = prop.findGroup("ZDF").find("RADIAL_PENALTY").asInt();
  int m_size           = prop.findGroup("ZDF").find("M_SIZE").asInt(); //(square)
  int bland_dog_thresh = prop.findGroup("ZDF").find("BLAND_DOG_THRESH").asInt();
  double bland_prob    = prop.findGroup("ZDF").find("BLAND_PROB").asDouble();

  int t_size           = prop.findGroup("ZDFTRACK").find("T_SIZE").asInt();
  int t_lock           = prop.findGroup("ZDFTRACK").find("T_LOCK").asInt();
  int v_lock_lr        = prop.findGroup("ZDFTRACK").find("V_LOCK_LR").asInt();
  int v_lock_ud        = prop.findGroup("ZDFTRACK").find("V_LOCK_UD").asInt();
  Ipp32f shift_sim_t   = prop.findGroup("ZDFTRACK").find("TRACK_SIM").asDouble();
  Ipp32f shift_sim_v   = prop.findGroup("ZDFTRACK").find("VERGE_SIM").asDouble();
  double verg_gain     = prop.findGroup("ZDFTRACK").find("VERGE_GAIN").asDouble();
  double track_gain    = prop.findGroup("ZDFTRACK").find("TRACK_GAIN").asDouble();
  int min_area         = prop.findGroup("ZDFTRACK").find("MIN_AREA").asInt();
  int max_area         = prop.findGroup("ZDFTRACK").find("MAX_AREA").asInt();
  int max_spread       = prop.findGroup("ZDFTRACK").find("MAX_SPREAD").asInt();
  int max_wait         = prop.findGroup("ZDFTRACK").find("RETURN_HOME").asInt();
  bool motion          = (bool) prop.findGroup("ZDFTRACK").find("MOTION").asInt();
  bool track_lock      = (bool) prop.findGroup("ZDFTRACK").find("TRACK_LOCK").asInt();

  int nclasses         = 2; //zd, not_zd
  bool return_home = false;
  if (max_wait!=0){return_home = true;}


  //PROBE RECSERVER:
  Port inPort_s;
  inPort_s.open("/zdfserver/input/serv_params");     
  Network::connect("/zdfserver/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/zdfserver/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 

  //RECSERVER INPUT:
  BufferedPort<Bottle> inPort_ly; 
  inPort_ly.open("/zdfserver/input/rec_ly");     
  Network::connect("/recserver/output/left_ye" , "/zdfserver/input/rec_ly");
  Bottle *inBot_ly;
  
  BufferedPort<Bottle> inPort_ry; 
  inPort_ry.open("/zdfserver/input/rec_ry");     
  Network::connect("/recserver/output/right_ye" , "/zdfserver/input/rec_ry");
  Bottle *inBot_ry;
  
  RecResultParams* rec_res;
  int dpix_y=0;
  
  



  //Sizes:
  IppiSize srcsize;
  srcsize.width = rsp.width;
  srcsize.height = rsp.height;
  int psb_in = rsp.psb;
  IppiSize msize;
  msize.width  = m_size;
  msize.height = m_size;
  IppiSize tsize;
  tsize.width  = t_size;
  tsize.height = t_size;
  IppiSize tisize;
  tisize.width  = tsize.width  + 2*t_lock;
  tisize.height = tsize.height + 2*t_lock;
  IppiSize trsize;
  trsize.width  = tisize.width  - tsize.width  + 1;
  trsize.height = tisize.height - tsize.height + 1;
  IppiSize visize;
  visize.width  = tsize.width  + 2*v_lock_lr;
  visize.height = tsize.height + 2*v_lock_ud;
  IppiSize vrsize;
  vrsize.width  = visize.width  - tsize.width  + 1;
  vrsize.height = visize.height - tsize.height + 1;

  //Vars:
  int sx,sy;
  Ipp32f max_v;
  Ipp32f max_t;
  int mid_x = (srcsize.width  - msize.width)/2;
  int mid_y = (srcsize.height - msize.height)/2;
  int mid_x_m = (msize.width  - tsize.width)/2;
  int mid_y_m = (msize.height - tsize.height)/2;
  int area;
  int cog_x = 0;
  int cog_y = 0;
  int spread;
 

  //RANK/NDT TRANSFORM:
  Coord c;
  int ndt1[NDTSIZE];
  int ndt2[NDTSIZE];
  int rank1[RANKSIZE];
  int rank2[RANKSIZE];
  double cmp_res;
  int koffsetx;
  int koffsety;
  if (RANK0_NDT1==0){
    koffsetx=RANKX;
    koffsety=RANKY;
  }
  else{
    koffsetx=NDTX;
    koffsety=NDTY;
  }


  //MALLOCS:
  int psb_m,psb_t,psb_rest,psb_resv;
  Ipp32f *res_t     = ippiMalloc_32f_C1(trsize.width,trsize.height,&psb_rest);
  Ipp32f *res_v     = ippiMalloc_32f_C1(vrsize.width,vrsize.height,&psb_resv);
  Ipp8u *out        = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *seg_im     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *seg_dog    = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);

  Ipp8u *fov_l      = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_r      = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *zd_prob_8u = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *o_prob_8u  = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u **p_prob    = (Ipp8u**) malloc(sizeof(Ipp8u*)*nclasses);

  ippiSet_8u_C1R(0,zd_prob_8u,psb_m,msize);
  ippiSet_8u_C1R(0,o_prob_8u,psb_m,msize);

  p_prob[0] = o_prob_8u;
  p_prob[1] = zd_prob_8u;

  //template:
  Ipp8u *temp       = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
  //input:
  Ipp8u *rec_im_ly;
  Ipp8u *rec_im_ry;

  //Processing Classes:
  DoG*dl = new DoG(msize);
  DoG*dr = new DoG(msize);
  MultiClass *m = new MultiClass(msize,psb_m,nclasses,&params);





  //SERVER OUTPUTS:

  //Server params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/zdfserver/output/serv_params");
  
  //Motion output
  Port outPort_mot;
  outPort_mot.open("/zdfserver/output/mot");     // Give it a name on the network.
  Network::connect("/zdfserver/output/mot", "/recserver/input/motion");
  Network::connect("/recserver/input/motion", "/zdfserver/output/mot");
  BinPortable<RecMotionRequest> motion_request;


  if (motion){
    //initalise:
    motion_request.content().pix_y  = 0;
    motion_request.content().pix_xl = 25;
    motion_request.content().pix_xr = -25;
    motion_request.content().deg_r  = 0.0;
    motion_request.content().deg_p  = 0.0;
    motion_request.content().deg_y  = 0.0;
    motion_request.content().relative = false; //absolute initial pos.
    motion_request.content().suspend  = 0;
    motion_request.content().lockto  = NO_LOCK;
    motion_request.content().unlock = true;
    //send:
    //outPort_mot.write(motion_request); //don't send :)
  }

  // Make port for res_mask output:
  BufferedPort<Bottle> outPort_res_mask;
  outPort_res_mask.open("/zdfserver/output/res_mask");
  // Make port for res_prob output:
  BufferedPort<Bottle> outPort_res_prob;
  outPort_res_prob.open("/zdfserver/output/res_prob");
  // Make a port for left im fovea
  BufferedPort<Bottle> outPort_fov_l;
  outPort_fov_l.open("/zdfserver/output/fov_l");
  // Make a port for right im fovea
  BufferedPort<Bottle> outPort_fov_r;
  outPort_fov_r.open("/zdfserver/output/fov_r");
  BufferedPort<Bottle> outPort_temp;
  outPort_temp.open("/zdfserver/output/template");
  // Make a port for seg im
  BufferedPort<Bottle> outPort_seg_im;
  outPort_seg_im.open("/zdfserver/output/seg_im");
  // Make a port for seg dog
  BufferedPort<Bottle> outPort_seg_dog;
  outPort_seg_dog.open("/zdfserver/output/seg_dog");
  // Make a port for YARPVIEW DISPLAY
  BufferedPort<ImageOf<PixelMono> > outPort_yarpimg;
  outPort_yarpimg.open("/zdfserver/output/yarpimg");
  // Make a port for rec l
  BufferedPort<Bottle> outPort_rec_l;
  outPort_rec_l.open("/zdfserver/output/rec_l");
  // Make a port for rec r
  BufferedPort<Bottle> outPort_rec_r;
  outPort_rec_r.open("/zdfserver/output/rec_r");


  // Make replier for server param probes on params port:
  //Set Replied Params:
  ZDFServerParams zsp;
  zsp.m_size=m_size;
  zsp.m_psb=psb_m;
  zsp.t_size=t_size;
  zsp.t_psb=psb_t;
  //Replier:
  ZDFReplyParamProbe replier;
  replier.reply=zsp;
  outPort_s.setReplier(replier); 



  int tl_x=0,tl_y=0;
  int del_x=0,del_y=0;
  int waiting = 0;
  bool track = false;
  int rad_pen,max_rad_pen;
  double r;
  double rmax = sqrt((msize.width/2.0)*(msize.width/2.0) 
		     +(msize.height/2.0)*(msize.height/2.0));

  bool update = false;

  bool acquire = false;

  //TCREATE


  //Main event loop:
  while (1)
    {   
      
      //TSTART
      
      
      inBot_ly = inPort_ly.read(false);
      inBot_ry = inPort_ry.read();
      
      if (inBot_ly!=NULL &&
	  inBot_ry!=NULL ){
	
	rec_im_ly = (Ipp8u*) inBot_ly->get(0).asBlob();
	rec_im_ry = (Ipp8u*) inBot_ry->get(0).asBlob();
	rec_res = (RecResultParams*) inBot_ry->get(1).asBlob();
   	dpix_y = rec_res->ly-rec_res->ry;
	
	
	if (acquire){
	  ippiCopy_8u_C1R(&rec_im_ly[((srcsize.height-tisize.height)/2)*psb_in +
				     (srcsize.width-tisize.width)/2],
			  psb_in,temp,psb_t,tsize);	  
	}




	//**************************
	//MAKE LEFT (LEAD) FOVEA
	//find template in left image:
	ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ly[((srcsize.height-tisize.height)/2)*psb_in +
							  (srcsize.width-tisize.width)/2],
					       psb_in,tisize,
					       temp,
					       psb_t,tsize,
					       res_t, psb_rest);
	ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
	if (max_t < shift_sim_t){
	  //this also prevents tracking motion:
	  tl_x = 0;
	  tl_y = 0;
	  track = false;
	}
	else{ 
	  //this initiates tracking motion if non-zero:
	  tl_x = sx - trsize.width/2;
	  tl_y = sy - trsize.height/2;
	  track = true;
	}
	ippiCopy_8u_C1R(&rec_im_ly[(mid_y+tl_y)*psb_in + mid_x+tl_x],psb_in,fov_l,psb_m,msize);
		
	

	
	//**************************
	//MAKE RIGHT (TRAILING) FOVEA
	//always align right fovea with left fovea:
	ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ry[((srcsize.height-visize.height)/2+dpix_y)*psb_in +
							  (srcsize.width-visize.width)/2],
					       psb_in,visize,
					       &fov_l[mid_y_m*psb_m + mid_x_m],
					       psb_m,tsize,
					       res_v, psb_resv);  
	ippiMaxIndx_32f_C1R(res_v,psb_resv,vrsize,&max_t,&sx,&sy);
	if (max_t < shift_sim_v){
	  //no l-r alignment found..
	  //this prevents verge motion:
	  del_x = 0;
	  del_y = 0;
	}
	else{ 
	  //stereo alignment found!!
	  //this initiates verge motion if non-zero:
	  del_x = sx - vrsize.width/2;
	  del_y = sy - vrsize.height/2;
	}
	//Right (TRAILING) Fovea from centre:
	ippiCopy_8u_C1R(&rec_im_ry[(mid_y+del_y+dpix_y+del_y)*psb_in + mid_x+del_x],psb_in,fov_r,psb_m,msize);





	//we have foveas, aligned when possible.
	//always do segmentation





	//**************************
	//DOG filters out low-confidence un-textured regions:
	dl->proc(fov_l,psb_m);
	dr->proc(fov_r,psb_m);
	
	
	//**************************
	//SPATIAL ZD probability map from fov_l and fov_r:
	//perform RANK or NDT kernel comparison:		
	for (int j=koffsety;j<msize.height-koffsety;j++){
	  c.y=j;
	  for (int i=koffsetx;i<msize.width-koffsetx;i++){
	    c.x=i;
	   
	    //if either l or r textured at this retinal location: 
	    if (dl->get_dog_onoff()[i + j*dl->get_psb()] >= bland_dog_thresh ||
		dr->get_dog_onoff()[i + j*dr->get_psb()] >= bland_dog_thresh ){
	      
	      if (RANK0_NDT1==0){
		//use RANK:
		get_rank(c,fov_l,psb_m,rank1);
		get_rank(c,fov_r,psb_m,rank2);
		//get_rank(c,dl->get_dog_onoff(),dl->get_psb(),rank1);
		//get_rank(c,dr->get_dog_onoff(),dr->get_psb(),rank2);
		cmp_res = cmp_rank(rank1,rank2);
	      }
	      else{ 
		//use NDT:
		get_ndt(c,fov_l,psb_m,ndt1);
		get_ndt(c,fov_r,psb_m,ndt2);
		//get_ndt(c,dl->get_dog_onoff(),dl->get_psb(),ndt1);
		//get_ndt(c,dr->get_dog_onoff(),dr->get_psb(),ndt2);
		cmp_res = cmp_ndt(ndt1,ndt2);

	      }
	      zd_prob_8u[j*psb_m+i] = (int)(cmp_res*255.0);
	      
	    }
	    else{
	      //untextured, so set to bland prob (ZD):
	      zd_prob_8u[j*psb_m+i] = (int)(bland_prob*255.0);
	    } 

	    //RADIAL PENALTY:
	    //The further from the origin, less likely it's ZD, so reduce zd_prob radially:
	    //current radius:
	    r = sqrt((c.x-msize.width/2.0)*(c.x-msize.width/2.0)+(c.y-msize.height/2.0)*(c.y-msize.height/2.0));
	    rad_pen =  (int) ( (r/rmax)*radial_penalty );
	    max_rad_pen = zd_prob_8u[j*psb_m+i];
	    if(max_rad_pen < rad_pen) {
	      rad_pen=max_rad_pen;
	    }
	    //apply radial penalty
	    zd_prob_8u[j*psb_m+i]-= rad_pen;
	    
	    //OTHER:
	    //manufacture NZD prob (other):
	    o_prob_8u[psb_m*j+i] = 255 - zd_prob_8u[psb_m*j+i];
	  }
	}
	
	

	
	//DO MRF OPTIMISATION!!:
	m->proc(fov_l,p_prob); //provide edge map and probability map
	//cache for distribution:
	ippiCopy_8u_C1R(m->get_class(),m->get_psb(),out,psb_m,msize);
	//evaluate result:
	getAreaCoGSpread(out,psb_m,msize, &area,&cog_x,&cog_y,&spread); 	

	


	//we have mask and image  (out)   [0/255].
	//construct masked image  (fov_l) [0..255]:
	for (int j=0;j<msize.height;j++){
	  for (int i=0;i<msize.width;i++){
	    if (out[j*psb_m + i]==0){
	      seg_im[j*psb_m + i]=0;
	      seg_dog[j*psb_m + i]=0;
	    }
	    else{
	      seg_dog[j*psb_m + i] = dl->get_dog_onoff()[j*psb_m + i];
	      seg_im[j*psb_m + i] = fov_l[j*psb_m + i];
	    }
	  }
	}
	
		

	//If nice segmentation:
	if (area>=min_area && area<=max_area && spread<=max_spread){
	  //update template towards segmentation CoG
	  printf("area:%d spread:%d cogx:%d cogy:%d - UPDATING TEMPLATE\n",area,spread,cog_x,cog_y);
	  if (cog_x>1){
	   cog_x = 1;
	  }
	  else if (cog_x<-1){
	    cog_x = -1;
	  }
	  else{ cog_x = 0;}

	  if (cog_y>1){
	   cog_y = 1;
	  }
	  else if (cog_y<-1){
	    cog_y = -1;
	  }
	  else{ cog_y = 0;}

	  ippiCopy_8u_C1R(&fov_l[(mid_x_m+cog_x) + 
				 (mid_y_m+cog_y)*psb_m],
			  psb_m,temp_l,psb_t,tsize);
	  ippiCopy_8u_C1R(&fov_r[(mid_x_m+cog_x) + 
				 (mid_y_m+cog_y)*psb_m],
			  psb_m,temp_r,psb_t,tsize);
	  waiting=0;
	  update = true;
	}
	//Otherwise, just keep previous template.
	else{
	  printf("area:%d spread:%d cogx:%d cogy:%d\n",area,spread,cog_x,cog_y);	
	  waiting++;
	  update = false;
	}
	
	
	

	if (motion){

	  if(waiting>=max_wait && return_home){
	    printf("Returning home! (waiting:%d >= max_wait:%d)\n",waiting,max_wait);
	    //re-initalise:
	    motion_request.content().pix_y  = 0;
	    motion_request.content().pix_xl = 25;
	    motion_request.content().pix_xr = -25;
	    motion_request.content().deg_r  = 0.0;
	    motion_request.content().deg_p  = 0.0;
	    motion_request.content().deg_y  = 0.0;
	    motion_request.content().relative = false; //absolute initial pos.
	    motion_request.content().suspend  = 50;
	    motion_request.content().lockto  = NO_LOCK;
	    motion_request.content().unlock = true;
	    //send:
	    outPort_mot.write(motion_request);
	    waiting = 0;
	  }

	  else{

	    //*********************************
	    //ALWAYS MOVE TO REDUCE VIRTUAL VERGE SHIFT AND TRACK OFFSET TO ZERO:
	    motion_request.content().pix_xl = (int) (((double)tl_x)*track_gain);
	    motion_request.content().pix_xr = (int) (((double)tl_x)*track_gain + ((double)del_x)*verg_gain);
	    motion_request.content().pix_y  = (int) (((double)tl_y)*track_gain);
	    motion_request.content().relative = true; //relative move.
	    motion_request.content().suspend  = 0; 
	    
	    //If target locking requesteg, while tracking success, 
	    //lock motion control to the ZDF server only.  Otherwise, unlock 
	    //so that attentional saccades can occur.
	    if (track_lock && track){
	      //lock to ZDF:
	      motion_request.content().lockto = ZDF_LOCK; //can only call if presently no other lock
	      motion_request.content().unlock = false;
	      printf("ZDFServer: ZDF_LOCK requested.\n");
	    }
	    else{
	      //unlock:
	      motion_request.content().lockto = ZDF_LOCK; //If locked to ZDFServer,  only ZDFServer can unlock
	      motion_request.content().unlock = true;
	    }
	    
	    outPort_mot.write(motion_request);
	    //*********************************	  
	  }
	}
	


	
	
	//SEND RESULT IMS
	Bottle& tmpBot_res_mask = outPort_res_mask.prepare();
	tmpBot_res_mask.clear();
	tmpBot_res_mask.add(Value::makeBlob( out, psb_m*m_size));
	tmpBot_res_mask.addDouble(rec_res->gaze3D_x);
	tmpBot_res_mask.addDouble(rec_res->gaze3D_y);
	tmpBot_res_mask.addDouble(rec_res->gaze3D_z);
	tmpBot_res_mask.addInt((int)update);
	outPort_res_mask.write();
	
	Bottle& tmpBot_res_prob = outPort_res_prob.prepare();
	tmpBot_res_prob.clear();
	tmpBot_res_prob.add(Value::makeBlob( zd_prob_8u, psb_m*m_size));
	outPort_res_prob.write();
	
	Bottle& tmpBot_fov_l = outPort_fov_l.prepare();
	tmpBot_fov_l.clear();
	tmpBot_fov_l.add(Value::makeBlob( fov_l, psb_m*m_size));
	outPort_fov_l.write();
	
	Bottle& tmpBot_rec_l = outPort_rec_l.prepare();
	tmpBot_rec_l.clear();
	tmpBot_rec_l.add(Value::makeBlob( rec_im_ly, psb_in*srcsize.height));
	outPort_rec_l.write();

	Bottle& tmpBot_rec_r = outPort_rec_r.prepare();
	tmpBot_rec_r.clear();
	tmpBot_rec_r.add(Value::makeBlob( rec_im_ry, psb_in*srcsize.height));
	outPort_rec_r.write();

	Bottle& tmpBot_fov_r = outPort_fov_r.prepare();
	tmpBot_fov_r.clear();
	tmpBot_fov_r.add(Value::makeBlob( fov_r, psb_m*m_size));
	outPort_fov_r.write();

	Bottle& tmpBot_temp = outPort_temp.prepare();
	tmpBot_temp.clear();
	tmpBot_temp.add(Value::makeBlob( temp, psb_t*t_size));
	outPort_temp.write();

	Bottle& tmpBot_seg_im = outPort_seg_im.prepare();
	tmpBot_seg_im.clear();
	tmpBot_seg_im.add(Value::makeBlob( seg_im, psb_m*m_size));
	outPort_seg_im.write();

	Bottle& tmpBot_seg_dog = outPort_seg_dog.prepare();
	tmpBot_seg_dog.clear();
	tmpBot_seg_dog.add(Value::makeBlob( seg_dog, psb_m*m_size));
	//target pos.. DOES NOT TAKE INTO ACCOUNT VIRTUAL SHIFT!!
	//use rsp.pix2deg(S)!!!
	tmpBot_seg_dog.addDouble(rec_res->gaze3D_x);
	tmpBot_seg_dog.addDouble(rec_res->gaze3D_y);
	tmpBot_seg_dog.addDouble(rec_res->gaze3D_z);
	tmpBot_seg_dog.addInt((int)update);
	outPort_seg_dog.write();

	//A yarpview compatible output port:
	//norm to 255 for display:
	normValI_8u(255,out,psb_m,msize);
	ImageOf<PixelMono>& tmp_yarpimg = outPort_yarpimg.prepare();
	tmp_yarpimg.resize(m_size,m_size);
	for (int y=0;y<m_size;y++){
	  memcpy(tmp_yarpimg.getRowArray()[y],&seg_im[y*psb_m],m_size);
	}
	outPort_yarpimg.write();
	
	
      }
      else if (inBot_ly==NULL &&
	       inBot_ry==NULL ){
	printf("No Input\n");
	usleep(5000); //don't blow out port
      }
      
      
      
      //TSTOP
    }
  
  //never here..
  //TEND
}







void iCub::contrib::primateVision::ZDFServer::get_ndt(Coord c,Ipp8u * im, int w, int*list)
{

  Coord n;

  int ndt_ind = 0;
  n = c+Coord(1,0);     
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;  

  ndt_ind++;
  n = c+Coord(0,1);
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;

  ndt_ind++;
  n = c+Coord(-1,0);
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;

  ndt_ind++;
  n = c+Coord(0,-1);
  if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
    list[ndt_ind]= 0;
  else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
    list[ndt_ind]= 1;
  else
    list[ndt_ind]= -1;
  

  if (NDTSIZE>4){

    //diagonals:
    ndt_ind++;
    n = c+Coord(1,1);     
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;  

    ndt_ind++;
    n = c+Coord(1,-1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(-1,1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(-1,-1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
      list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
      list[ndt_ind]= 1;
    else
      list[ndt_ind]= -1;
  }
  
}

double iCub::contrib::primateVision::ZDFServer::cmp_ndt(int*ndt_l, int*ndt_r)
{

  int s=0;

  for (int count=0;count<NDTSIZE;count++){
    if(ndt_l[count]==ndt_r[count]){
      s++;
    }
  }

  return ((double)s)/((double)NDTSIZE);

}


void iCub::contrib::primateVision::ZDFServer::get_rank(Coord c,Ipp8u *im, int w, int*list)
{
  Coord n;
  int i = 0;

  for (int x=-RANKX;x<=RANKX;x++){
    for (int y=-RANKY;y<=RANKY;y++){
      
      n = c+Coord(x,y);
      list[i] = im[n.y*w + n.x];
      i++;

    }
  }

}

double iCub::contrib::primateVision::ZDFServer::cmp_rank(int*l1, int*l2)
{ 
  int n1 = 0; //number of non-ties for x
  int n2 = 0; //number of non-ties for y
  int is = 0;
  
  int a1,a2,aa;
  
  double tau;//,svar,z,prob;

  for(int j=0;j<RANKSIZE;j++) {
    for(int k=j+1;k<RANKSIZE;k++) {
      a1 = l1[j] - l1[k];
      a2 = l2[j] - l2[k];
      aa = a1*a2;
      if(aa) {
	++n1;
	++n2;
  	
	aa > 0 ? ++is : --is;
	
      } else {
	if(a1) ++n1;
	if(a2) ++n2;
      }
    }
  }
  
  tau = (is) / (sqrt(n1) * sqrt(n2));
  // svar = (4.0 * n + 10.0) / (9.0 * n * (n - 1.0));
  // z = tau / sqrt(svar);
  // prob = erfcc(abs(z) / 1.4142136);

  if (tau < 0.0){tau=0.0;}

  return tau;
}



void iCub::contrib::primateVision::ZDFServer::getAreaCoGSpread(Ipp8u*im_,int psb_,IppiSize sz_,int*parea,int*pdx,int*pdy,int*spread){

  int naccum = 0, xaccum = 0, yaccum = 0;
  *spread = 0;

  for (int j=0;j<sz_.height;j++){
    for (int i=0;i<sz_.width;i++){
      if (im_[j*psb_+i]!=0){
	xaccum+=i; yaccum+=j; naccum++;
      }
    }
  }
  
  *parea = naccum;
  if (naccum > 0){
    *pdx = -(sz_.width/2 - xaccum/naccum - 1);
    *pdy = -(sz_.height/2 - yaccum/naccum - 1);  

    //get spread:
    for (int j=0;j<sz_.height;j++){
      for (int i=0;i<sz_.width;i++){
	if (im_[j*psb_+i]!=0){  

	  *spread += (int) sqrt( abs((i-sz_.width/2-1) - (*pdx)) *
				 abs((i-sz_.width/2-1) - (*pdx)) 
				 + 
				 abs((j-sz_.height/2-1) -(*pdy)) *
				 abs((j-sz_.height/2-1) -(*pdy)) );
	}
      }
    }
    
    *spread/=naccum;
  }
  else {
    *pdx = 0;
    *pdy = 0;
    *spread = 0;
  }
  
 
}  


