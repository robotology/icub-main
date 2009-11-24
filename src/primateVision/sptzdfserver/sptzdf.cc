/*
 * Copyright (C) 2008-2009 Andrew Dankers. All rights reserved.
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
#include <convert_bitdepth.h>
#include <multiclass.h>
//THIS APP IS A RECCLIENT!!!
#include <recio.h>
#include "sptzdf.h"


#define RANK0_NDT1 1

#define NDTX     1
#define NDTY     1
#define NDTSIZE  8
#define NDTEQ    0

#define RANKX  1 //2
#define RANKY  1
#define RANKSIZE 9 //15
//RANKSISE = (RANKX*2+1)*(RANKY*2+1) //15 for (2,1) :)


#define MAX(a,b) ((a) < (b) ? (b) : (a))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

#define INFO 1



iCub::contrib::primateVision::SpTZDFServer::SpTZDFServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::SpTZDFServer::~SpTZDFServer()
{
  
}


void iCub::contrib::primateVision::SpTZDFServer::run(){
  

  Property prop; 
  prop.fromConfigFile(cfg->c_str());

  struct MultiClass::Parameters params;
  params.iter_max                  = prop.findGroup("SPTZDF").find("MAX_ITERATIONS").asInt();
  params.randomize_every_iteration = prop.findGroup("SPTZDF").find("RANDOMIZE_EVERY_ITER").asInt();
  params.smoothness_penalty        = prop.findGroup("SPTZDF").find("SMOOTHNESS_PENALTY").asInt();
  params.data_penalty              = prop.findGroup("SPTZDF").find("DATA_PENALTY").asInt();
  params.smoothness_3sigmaon2      = prop.findGroup("SPTZDF").find("SMOOTHNESS_3SIGMAON2").asInt();

  int nclasses                     = 2; //zd, not_zd
  int m_size                       = prop.findGroup("SPTZDF").find("M_SIZE").asInt(); //(square)
  int bland_dog_thresh             = prop.findGroup("SPTZDF").find("BLAND_DOG_THRESH").asInt();
  double bland_prob                = prop.findGroup("SPTZDF").find("BLAND_PROB").asDouble();

  int t_size                       = prop.findGroup("SPTZDFTRACK").find("T_SIZE").asInt();
  int t_lock_lr                    = prop.findGroup("SPTZDFTRACK").find("T_LOCK_LR").asInt();
  int t_lock_ud                    = prop.findGroup("SPTZDFTRACK").find("T_LOCK_UD").asInt();
  Ipp32f shift_sim_t               = prop.findGroup("SPTZDFTRACK").find("T_SIM").asDouble();

  int min_area                     = prop.findGroup("SPTZDFTRACK").find("MIN_AREA").asInt();
  int max_area                     = prop.findGroup("SPTZDFTRACK").find("MAX_AREA").asInt();
  int max_spread                   = prop.findGroup("SPTZDFTRACK").find("MAX_SPREAD").asInt();
  int max_wait                     = prop.findGroup("SPTZDFTRACK").find("RETURN_HOME").asInt();
  int mot                          = prop.findGroup("SPTZDFTRACK").find("MOTION").asInt();
  bool return_home = false;
  if (max_wait!=0){return_home = true;}



  //PROBE RECSERVER:
  Port inPort_s;
  inPort_s.open("/sptzdfserver/input/serv_params");     // Give it a name on the network.
  Network::connect("/sptzdfserver/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/sptzdfserver/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 

  //RECSERVER INPUT:
  BufferedPort<Bottle> inPort_ly; 
  inPort_ly.open("/sptzdfserver/input/rec_ly");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ye" , "/sptzdfserver/input/rec_ly");
  Bottle *inBot_ly;

  BufferedPort<Bottle> inPort_ry; 
  inPort_ry.open("/sptzdfserver/input/rec_ry");     // Give it a name on the network.
  Network::connect("/recserver/output/right_ye" , "/sptzdfserver/input/rec_ry");
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
  tisize.width  = tsize.width  + 2*t_lock_lr;
  tisize.height = tsize.height + 2*t_lock_ud;
  IppiSize trsize;
  trsize.width  = tisize.width  - tsize.width  + 1;
  trsize.height = tisize.height - tsize.height + 1;

  //Vars:
  int sx,sy;
  Ipp32f max_t;
  int pos_x = (srcsize.width  - msize.width)/2;
  int pos_y = (srcsize.height - msize.height)/2;
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
  Ipp32f cmp_res;
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
  int psb_m,psb_m_32f,psb,psb_rest,psb_t;
  Ipp32f *res_t        = ippiMalloc_32f_C1(trsize.width,trsize.height, &psb_rest);
  Ipp8u *out           = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *seg           = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_l         = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_r         = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *zd_prob_8u    = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *lzd_prob_8u   = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *rzd_prob_8u   = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *szd_prob_8u   = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *o_prob_8u     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *edge_im       = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp32f *edge_im_32f  = ippiMalloc_32f_C1(msize.width,msize.height, &psb_m_32f);
  Ipp32f *zd_prob_32f  = ippiMalloc_32f_C1(msize.width,msize.height, &psb_m_32f);
  Ipp32f *o_prob_32f   = ippiMalloc_32f_C1(msize.width,msize.height, &psb_m_32f);

  //template:
  Ipp8u *temp          = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
  //input:
  Ipp8u *rec_im_ly;
  Ipp8u *rec_im_ry;
  //cached:
  Ipp8u *rec_im_ly_old = ippiMalloc_8u_C1(srcsize.width,srcsize.height, &psb);
  Ipp8u *rec_im_ry_old = ippiMalloc_8u_C1(srcsize.width,srcsize.height, &psb);
  Ipp8u *fov_l_old     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_r_old     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *dog_l_old     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *dog_r_old     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);


  Ipp8u **p_prob = (Ipp8u**) malloc(sizeof(Ipp8u*)*nclasses);
  p_prob[0] = o_prob_8u;
  p_prob[1] = zd_prob_8u;




  //Processing Classes:
  DoG*dl = new DoG(msize);
  DoG*dr = new DoG(msize);
  MultiClass *m = new MultiClass(msize,psb_m,nclasses,&params);





  //SERVER OUTPUTS:

  //Server params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/sptzdfserver/output/serv_params");

  //Motion output
  Port outPort_mot;
  outPort_mot.open("/sptzdfserver/output/mot"); 
  Network::connect("/sptzdfserver/output/mot", "/recserver/input/motion");
  Network::connect("/recserver/input/motion", "/sptzdfserver/output/mot");
  BinPortable<RecMotionRequest> motion_request;

  if (mot){
    //initalise:
    motion_request.content().pix_y  = 0.0;
    motion_request.content().pix_xl = 20.0;
    motion_request.content().pix_xr = -20.0;
    motion_request.content().deg_r  = 0.0;
    motion_request.content().relative = false; //absolute initial pos.
    motion_request.content().suspend  = 0;
    motion_request.content().lockto  = NO_LOCK;
    motion_request.content().unlock = true;
    //reset:
    outPort_mot.write(motion_request);
  }

  // Make port for res_mask output:
  BufferedPort<Bottle> outPort_res_mask;
  outPort_res_mask.open("/sptzdfserver/output/res_mask");

  // Make port for res_prob output:
  BufferedPort<Bottle> outPort_res_prob;
  outPort_res_prob.open("/sptzdfserver/output/res_prob");

  // Make port for edge_im output:
  BufferedPort<Bottle> outPort_edge_im;
  outPort_edge_im.open("/sptzdfserver/output/edge_im");

  BufferedPort<Bottle> outPort_fov_l;
  outPort_fov_l.open("/sptzdfserver/output/fov_l");

  BufferedPort<Bottle> outPort_fov_r;
  outPort_fov_r.open("/sptzdfserver/output/fov_r");

  BufferedPort<Bottle> outPort_temp;
  outPort_temp.open("/sptzdfserver/output/template");

  // Make a port for seg
  BufferedPort<Bottle> outPort_seg;
  outPort_seg.open("/sptzdfserver/output/seg");
  
  // Make a port for YARPVIEW DISPLAY
  BufferedPort<ImageOf<PixelMono> > outPort_yarpimg;
  outPort_yarpimg.open("/sptzdfserver/output/yarpimg");



  // Make replier for server param probes on params port:
  //Set Replied Params:
  SpTZDFServerParams zsp;
  zsp.m_size=m_size;
  zsp.m_psb=psb_m;
  zsp.t_size=t_size;
  zsp.t_psb=psb_t;
  //Replier:
  SpTZDFReplyParamProbe replier;
  replier.reply=zsp;
  outPort_s.setReplier(replier); 

 


  int olx=0,oly=0,clx=0,cly=0;
  int orx=0,ory=0,crx=0,cry=0;
  bool cl_ok,cr_ok,ol_ok,or_ok;
  bool l_zd,r_zd,s_zd;
  int omxl,omxr,omyl,omyr;
  int cmxl,cmxr,cmyl,cmyr;

  int k=0;
  int since_update = 0;  
  int no_align = 0;
  bool track = false;
  bool get_new_temp = true;
  bool got_new_temp = true;



  //TCREATE



  //Main event loop:
  while(1)
    {   
      

      //TSTART


      inBot_ly = inPort_ly.read(false);
      inBot_ry = inPort_ry.read();



      
      if (inBot_ly!=NULL &&
	  inBot_ry!=NULL ){
	

	rec_im_ly = (Ipp8u*) inBot_ly->get(0).asBlob();
	rec_im_ry = (Ipp8u*) inBot_ry->get(0).asBlob();
	rec_res = (RecResultParams*) inBot_ly->get(1).asBlob();
	dpix_y = rec_res->ly-rec_res->ry;


	omxl = cmxl;
	omxr = cmxr;
	omyl = cmyl;
	omyr = cmyr;
	cmxl = rec_res->lx;
	cmxr = rec_res->rx;
	cmyl = rec_res->ly;
	cmyr = rec_res->ry;

	

	
	//we have a template to search for.
	l_zd = false;
	r_zd = false;
	s_zd = false;	
	
	


	//find temp in current_L
	//**************************
	ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ly[((srcsize.height-tisize.height)/2)*psb_in +
							  (srcsize.width-tisize.width)/2],
					       psb_in,tisize,
					       temp,
					       psb_t,tsize,
					       res_t, psb_rest);
	ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
	if (max_t > shift_sim_t){
	  //printf("CL FOUND.\n");
	  clx = sx - trsize.width/2;
	  cly = sy - trsize.height/2;
	  cl_ok = true;
	}
	else{ 
	  //printf("CL NOT FOUND.\n");
	  //keep old location!
	  //clx = 0;
	  //cly = 0;
	  cl_ok = false;
	}
	//construct fov_l:
	ippiCopy_8u_C1R(&rec_im_ly[(pos_y+cly)*psb_in + pos_x+clx],psb_in,fov_l,psb_m,msize);
	
	
	//find temp in old_L, 
	//**************************
	ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ly_old[((srcsize.height-tisize.height)/2)*psb +
							      (srcsize.width-tisize.width)/2],
					       psb,tisize,
					       temp,
					       psb_t,tsize,
					       res_t, psb_rest);
	ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
	if (max_t > shift_sim_t){
	  //printf("OL FOUND.\n");
	  olx = sx - trsize.width/2;
	  oly = sy - trsize.height/2;
	  ol_ok = true;
	}
	else{ 
	  //printf("OL NOT FOUND.\n");
	  //keep old location!
	  //olx = 0;
	  //oly = 0;
	  ol_ok = false;
	}
	//construct fov_l_old:
	ippiCopy_8u_C1R(&rec_im_ly_old[(pos_y+oly)*psb + pos_x+olx],psb,fov_l_old,psb_m,msize);
	
	
	//find temp in current_R, 
	//**************************
	ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ry[((srcsize.height-tisize.height)/2 + dpix_y)*psb_in +
							  (srcsize.width-tisize.width)/2],
					       psb_in,tisize,
					       temp,
					       psb_t,tsize,
					       res_t, psb_rest);
	ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
	if (max_t > shift_sim_t){
	  //printf("CR FOUND.\n");
	  crx = sx - trsize.width/2;
	  cry = sy - trsize.height/2 + dpix_y;
	  cr_ok = true;
	}
	else{ 
	  //printf("CR NOT FOUND.\n");
	  //keep old location!
	  //crx = 0;
	  //cry = 0;
	  cr_ok = false;
	}
	//construct fov_r:
	ippiCopy_8u_C1R(&rec_im_ry[(pos_y+cry)*psb_in + pos_x+crx],psb_in,fov_r,psb_m,msize);

	
	//find temp in old_R, 
	//**************************
	ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ry_old[((srcsize.height-tisize.height)/2 + dpix_y)*psb +
							      (srcsize.width-tisize.width)/2],
					       psb,tisize,
					       temp,
					       psb_t,tsize,
					       res_t, psb_rest);
	ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
	if (max_t > shift_sim_t){
	  //printf("OR FOUND.\n");
	  orx = sx - trsize.width/2;
	  ory = sy - trsize.height/2 + dpix_y;
	  or_ok = true;
	}
	else{ 
	  //printf("OR NOT FOUND.\n");
	  //keep old location!
	  //orx = 0;
	  //ory = 0;
	  or_ok = false;
	}
	//construct fov_r_old:
	ippiCopy_8u_C1R(&rec_im_ry_old[(pos_y+ory)*psb + pos_x+orx],psb,fov_r_old,psb_m,msize);


	//if templates found in L and template moved in world (mosaic coords): 
	if (cl_ok && ol_ok  && ((clx+cmxl)!=(olx+omxl) || (cly+cmyl)!=(oly+omyl)) )
	  {l_zd = true;}// printf("L\n");} 
	//if templates found in R and  template moved in world (mosaic coords): 
	if (cr_ok && or_ok  && ((crx+cmxr)!=(orx+omxr) || (cry+cmyr)!=(ory+omyr)) )
	  {r_zd = true;}// printf("R\n");} 
	//stereo:
	if (cl_ok && cr_ok)
	  {s_zd = true;}// printf("S\n");} 





	//INDIVIDUAL ZD MAP TESTING
	//s_zd = false;
	//l_zd = false;
	//r_zd = false;







	//now we have all fovea images aligned to template where possible.

	//DOG texture filtering:
	dl->proc(fov_l,psb_m);
	dr->proc(fov_r,psb_m);
	
	//If any alignment, let's build the probability maps:
	if (s_zd || l_zd || r_zd){

	  for (int j=koffsety;j<=msize.height-koffsety-1;j++){
	    c.y=j;
	    for (int i=koffsetx;i<msize.width-koffsetx-1;i++){
	      c.x=i;
	      
	      
	      //STEREO:
	      if (s_zd){
		//if textured: 
		if (dl->get_dog_onoff()[i + j*dl->get_psb()] >= bland_dog_thresh ||
		    dr->get_dog_onoff()[i + j*dr->get_psb()] >= bland_dog_thresh ){
		  
		  if (RANK0_NDT1==0){
		    get_rank(c,fov_l,psb_m,rank1);
		    get_rank(c,fov_r,psb_m,rank2);
		    cmp_res = cmp_rank(rank1,rank2);
		  }
		  else{
		    get_ndt(c,fov_l,psb_m,ndt1);
		    get_ndt(c,fov_r,psb_m,ndt2);
		    cmp_res = cmp_ndt(ndt1,ndt2);
		  }
		  szd_prob_8u[j*psb_m+i] = (int)(cmp_res*255.0);
		
		}
		else{
		  //untextured, so set to bland prob:
		  szd_prob_8u[j*psb_m+i] = (int)(bland_prob*255.0);
		} 
	      }
	      
	      
	      //LEFT TEMPORAL:
	      if (l_zd){
		if (dl->get_dog_onoff()[i + j*dl->get_psb()] >= bland_dog_thresh ||
		    dog_l_old[i + j*psb_m] >= bland_dog_thresh                   ){
		  
		  if (RANK0_NDT1==0){
		    get_rank(c,fov_l,psb_m,rank1);
		    get_rank(c,fov_l_old,psb_m,rank2);
		    cmp_res = cmp_rank(rank1,rank2);
		  }
		  else{
		    get_ndt(c,fov_l,psb_m,ndt1);
		    get_ndt(c,fov_l_old,psb_m,ndt2);
		    cmp_res = cmp_ndt(ndt1,ndt2);
		  }
		  lzd_prob_8u[j*psb_m+i] = (int)(cmp_res*255.0);

		}
		else{
		  //untextured so set to bland prob:
		  lzd_prob_8u[j*psb_m+i] = (int)(bland_prob*255.0);
		} 
	      }
	      
	      //RIGHT TEMPORAL: 
	      if (r_zd){
		if (dr->get_dog_onoff()[i + j*dr->get_psb()] >= bland_dog_thresh ||
		    dog_r_old[i + j*psb_m] >= bland_dog_thresh                   ){
		  
		  if (RANK0_NDT1==0){
		    get_rank(c,fov_r,psb_m,rank1);
		    get_rank(c,fov_r_old,psb_m,rank2);
		    //build szd_prob from current L and R locations.
		    cmp_res = cmp_rank(rank1,rank2);
		  }
		  else{
		    get_ndt(c,fov_r,psb_m,ndt1);
		    get_ndt(c,fov_r_old,psb_m,ndt2);
		    //build szd_prob from current L and R locations.
		    cmp_res = cmp_ndt(ndt1,ndt2);
		  }
		  rzd_prob_8u[j*psb_m+i] = (int)(cmp_res*255.0);

		}
		else{
		  //untextured, so set to bland prob:
		  rzd_prob_8u[j*psb_m+i] = (int)(bland_prob*255.0);
		} 
	      }
	      
	    }
	  }


	  //make single zd_prob map:
	  ippiSet_32f_C1R(0.0,zd_prob_32f,psb_m_32f,msize);
	  double div = 0.0;
	  if (s_zd){
	    ippiAdd_8u32f_C1IR(szd_prob_8u,psb_m,zd_prob_32f,psb_m_32f,msize);
	    printf("S ");
	    div+=1.0;
	  }
	  if (l_zd){
	    ippiAdd_8u32f_C1IR(lzd_prob_8u,psb_m,zd_prob_32f,psb_m_32f,msize);
	    printf("L ");
	    div+=1.0;
	  }
	  if (r_zd){
	    ippiAdd_8u32f_C1IR(rzd_prob_8u,psb_m,zd_prob_32f,psb_m_32f,msize);
	    printf("R ");
	    div+=1.0;
	  }
	  printf("\n---\n");
	  //average:
	  if(div>1.0){
	    ippiDivC_32f_C1IR(div,zd_prob_32f,psb_m_32f,msize);
	  }
	  //convert to 8u without normalisation:
	  ippiConvert_32f8u_C1R(zd_prob_32f,psb_m_32f,zd_prob_8u,psb_m,msize,ippRndNear);
      


	  //manufacture NZD prob:
	  for (int j=RANKY;j<msize.height-RANKY;j++){
	    for (int i=RANKX;i<msize.width-RANKX;i++){
	      o_prob_8u[psb_m*j+i] = 255 - zd_prob_8u[psb_m*j+i];
	    }
	  }


	  //edge_im is a single fovea image:
	  if (l_zd || s_zd){
	    ippiCopy_8u_C1R(fov_l,psb_m,edge_im,psb_m,msize);
	  }
	  else{// if (r_zd){
	    ippiCopy_8u_C1R(fov_r,psb_m,edge_im,psb_m,msize);
	  }



	  //DO MRF SEGMENTATION!!:
	  m->proc(edge_im,p_prob); //provide edge map and probability map
	  //cache for distribution:
	  ippiCopy_8u_C1R(m->get_class(),m->get_psb(),out,psb_m,msize);



	  //evaluate segmentation result:
	  getAreaCoGSpread(out,psb_m,msize, &area,&cog_x,&cog_y,&spread);
	  //if good segmentation:
	  if (area>=min_area && area<=max_area && spread<=max_spread){
	    

	    //********UPDATE FROM IM, NOT EDGE_IM!!! (incase over border)
	    //update template!
	    printf("GOOD SEG. UPDATING TEMPLATE TO COG. area:%d spread:%d\n",area,spread);
	    ippiCopy_8u_C1R(&edge_im[cog_x + (msize.width-tsize.width)/2 + 
				     ((msize.height-tsize.height)/2 + cog_y)*psb_m],
			    psb_m,temp,psb_t,tsize);
	    since_update=0;
	  }
	  
	  //else if bad segmentation, don't update template
	  else {	
	    printf("POOR SEG. area:%d spread:%d\n",area,spread);
	  }
	  
	  track = true;
	  no_align = 0;

	  //alignment worked, so no need for 
	  //a new template and we're no longer
	  //using a new fovea centre template.
	  get_new_temp = false;
	  got_new_temp = false;

	}
	else{
	  
	  //if no L,R,S alignment on current template, MRF won't initiate,
	  //so clear output for display:
	  ippiSet_8u_C1R(0,zd_prob_8u,psb_m,msize); 
	  ippiSet_8u_C1R(0,edge_im,psb_m,msize);
	  ippiSet_8u_C1R(0,out,psb_m,msize);
	  no_align++;
	}



	//see if we need a new template yet:
	
	//if we recently got a new template, don't try aligning on it for too 
	//many times before getting another:
	if (got_new_temp && no_align>=1){
	  //get another template from fovea centre:
	  printf("NO ALIGNMENT ON NEW TEMP FOR 1 FRAMES. NEW TEMPLATE ");
	  get_new_temp = true;
	  track = false;
	}	  
	
	//but if it's not new template and it's been a while,
	else if (no_align>=3){

	  //get another template from fovea centre:
	  printf("NO ALIGNMENT FOR 3 FRAMES. NEW TEMPLATE ");
	  get_new_temp = true;
	  track = false;
	}
	


	
	//if we need a new template:
	if(get_new_temp){    
	  
	  //get another template from fovea centre:
	  //alternate l/r;
	  if (++k%2){
	    //try left this time:
	    printf("(L).\n");
	    ippiCopy_8u_C1R(&fov_l[(msize.width-tsize.width)/2 + 
				   ((msize.height-tsize.height)/2)*psb_m],
			    psb_m,temp,psb_t,tsize);
	  }
	  else{
	    //try right this time:
	    printf("(R).\n");
	    ippiCopy_8u_C1R(&fov_r[(msize.width-tsize.width)/2 + 
				   ((msize.height-tsize.height)/2)*psb_m],
			    psb_m,temp,psb_t,tsize);
	  } 
	  
	  got_new_temp = true;
	  get_new_temp = false;
	}
	 









	//*********************************
	//MOTION
	if (mot){
	  //always move cameras to reduce any current virtual shifts to zero:
	  motion_request.content().lockto   = NO_LOCK;
	  motion_request.content().unlock   = true;
	  if (s_zd){
	    motion_request.content().pix_xl = clx;
	    motion_request.content().pix_xr = crx;
	    motion_request.content().pix_y  = (cly+cry)/2;
	  }
	  else if (l_zd){
	    motion_request.content().pix_xl = clx;
	    motion_request.content().pix_xr = clx;
	    motion_request.content().pix_y  = cly;
	  }
	  else if (r_zd){
	    motion_request.content().pix_xl = crx;
	    motion_request.content().pix_xr = crx;
	    motion_request.content().pix_y  = cry;
	  }
	  motion_request.content().relative = true; //relative move.
	  motion_request.content().suspend  = 0; 
	  outPort_mot.write(motion_request);
	  //*********************************
	  
	  	  
	  
	  //if "return_home" flag activated and no target for a while,
	  //return to start position:
	  since_update++;
	  if (return_home && since_update>=max_wait){
	    //return to home:
	    printf("Request to return home.\n");
	    if (mot){
	      motion_request.content().pix_xl = 20.0;
	      motion_request.content().pix_xr = -20.0;
	      motion_request.content().pix_y  = 0.0;
	      motion_request.content().relative = false; //absolute move.
	      motion_request.content().suspend = 100; //wait
	      outPort_mot.write(motion_request);
	    }
	    since_update = 0;
	  }

	}	  






	//we have mask and image  (out)   [0/255].
	//construct masked image  (fov_l) [0..255]:
	for (int j=0;j<msize.height;j++){
	  for (int i=0;i<msize.width;i++){
	    if (out[j*psb_m + i]==0){
	      seg[j*psb_m + i ] = 0;
	    }
	    else{
	      seg[j*psb_m + i] = edge_im[j*psb_m + i];
	    }
	  }
	}
	



	
	//SEND RESULT IMS
	Bottle& tmpBot_res_mask = outPort_res_mask.prepare();
	tmpBot_res_mask.clear();
	tmpBot_res_mask.add(Value::makeBlob( out, psb_m*m_size));
	tmpBot_res_mask.addInt(area);
	tmpBot_res_mask.addInt(track);
	outPort_res_mask.write();
	
	Bottle& tmpBot_res_prob = outPort_res_prob.prepare();
	tmpBot_res_prob.clear();
	tmpBot_res_prob.add(Value::makeBlob( zd_prob_8u, psb_m*m_size));
	outPort_res_prob.write();
	
	Bottle& tmpBot_edge_im = outPort_edge_im.prepare();
	tmpBot_edge_im.clear();
	tmpBot_edge_im.add(Value::makeBlob( edge_im, psb_m*m_size));
	outPort_edge_im.write();

	Bottle& tmpBot_fov_l = outPort_fov_l.prepare();
	tmpBot_fov_l.clear();
	tmpBot_fov_l.add(Value::makeBlob( fov_l, psb_m*m_size));
	outPort_fov_l.write();

	Bottle& tmpBot_fov_r = outPort_fov_r.prepare();
	tmpBot_fov_r.clear();
	tmpBot_fov_r.add(Value::makeBlob( fov_r, psb_m*m_size));
	outPort_fov_r.write();
		
	Bottle& tmpBot_temp = outPort_temp.prepare();
	tmpBot_temp.clear();
	tmpBot_temp.add(Value::makeBlob( temp, psb_t*t_size));
	outPort_temp.write();

	Bottle& tmpBot_seg = outPort_seg.prepare();
	tmpBot_seg.clear();
	tmpBot_seg.add(Value::makeBlob( seg, psb_m*m_size));
	outPort_seg.write();

	//A yarpview compatible output port:
	//norm to 255 for display:
	normValI_8u(255,out,psb_m,msize);
	ImageOf<PixelMono>& tmp_yarpimg = outPort_yarpimg.prepare();
	tmp_yarpimg.resize(m_size,m_size);
	for (int y=0;y<m_size;y++){
	  memcpy(tmp_yarpimg.getRowArray()[y],&seg[y*psb_m],m_size);
	}
	outPort_yarpimg.write();
	


	//cache for next iteration:
	ippiCopy_8u_C1R(rec_im_ly,psb_in,rec_im_ly_old,psb,srcsize);
	ippiCopy_8u_C1R(rec_im_ry,psb_in,rec_im_ry_old,psb,srcsize);
	ippiCopy_8u_C1R(dl->get_dog_onoff(),dl->get_psb(),dog_l_old,psb_m,msize);
	ippiCopy_8u_C1R(dr->get_dog_onoff(),dr->get_psb(),dog_r_old,psb_m,msize);

      
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





void iCub::contrib::primateVision::SpTZDFServer::get_ndt(Coord c,Ipp8u * im, int w, int*list)
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

double iCub::contrib::primateVision::SpTZDFServer::cmp_ndt(int*ndt_l, int*ndt_r)
{

  int s=0;

  for (int count=0;count<NDTSIZE;count++){
    if(ndt_l[count]==ndt_r[count]){
      s++;
    }
  }

  return ((double)s)/((double)NDTSIZE);

}


void iCub::contrib::primateVision::SpTZDFServer::get_rank(Coord c,Ipp8u *im, int w, int*list)
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

double iCub::contrib::primateVision::SpTZDFServer::cmp_rank(int*l1, int*l2)
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



void iCub::contrib::primateVision::SpTZDFServer::getAreaCoGSpread(Ipp8u*im_,int psb_,IppiSize sz_,int*parea,int*pdx,int*pdy,int*spread){

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


