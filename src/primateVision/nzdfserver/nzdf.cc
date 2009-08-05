/*
 * Copyright (C) 2008-2009 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <iostream>
#include <math.h> 

//MY INCLUDES
#include <timing.h>
#include <norm.h>
#include <depth.h>
#include "nzdfopt.h"
//THIS APP IS A RECCLIENT!!!
#include <recio.h>
#include <dog.h>
#include "nzdf.h"




iCub::contrib::primateVision::NZDFServer::NZDFServer(string*c_)
{
  
  cfg = c_;
  
  start();

}

iCub::contrib::primateVision::NZDFServer::~NZDFServer()
{
  
}


void iCub::contrib::primateVision::NZDFServer::run(){
  

  Property prop; 
  prop.fromConfigFile(cfg->c_str());

  struct NZDFOpt::Parameters params;
  params.iter_max                      = prop.findGroup("NZDF").find("MAX_ITERATIONS").asInt();
  params.randomize_every_iteration     = prop.findGroup("NZDF").find("RANDOMIZE_EVERY_ITER").asInt();
  params.intensity_smoothness_3sigmaon2= prop.findGroup("NZDF").find("INTENSITY_SMOOTHNESS_3SIGMAON2").asInt();
  params.smoothness_penalty            = prop.findGroup("NZDF").find("SMOOTHNESS_PENALTY").asInt();
  params.disparity_penalty             = prop.findGroup("NZDF").find("DISPARITY_PENALTY").asInt();
  params.zd_penalty                    = prop.findGroup("NZDF").find("ZD_PENALTY").asInt();
  params.bland_dog_thresh              = prop.findGroup("NZDF").find("BLAND_DOG_THRESH").asInt();
 

  int nclasses                     = 2; //nzd, not_nzd
  int m_size                       = prop.findGroup("NZDF").find("M_SIZE").asInt(); //(square)

  int t_size                       = prop.findGroup("NZDFTRACK").find("T_SIZE").asInt();
  int t_lock_lr                    = prop.findGroup("NZDFTRACK").find("T_LOCK_LR").asInt();
  int t_lock_ud                    = prop.findGroup("NZDFTRACK").find("T_LOCK_UD").asInt();
  Ipp32f shift_sim_t               = prop.findGroup("NZDFTRACK").find("T_SIM").asDouble();
  int min_area                     = prop.findGroup("NZDFTRACK").find("MIN_AREA").asInt();
  int max_area                     = prop.findGroup("NZDFTRACK").find("MAX_AREA").asInt();
  int max_spread                   = prop.findGroup("NZDFTRACK").find("MAX_SPREAD").asInt();



  //PROBE RECSERVER:
  Port inPort_s;
  inPort_s.open("/nzdfserver/input/serv_params");     // Give it a name on the network.
  Network::connect("/nzdfserver/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/nzdfserver/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 



  //RECSERVER INPUT:
  BufferedPort<Bottle> inPort_ly; 
  inPort_ly.open("/nzdfserver/input/rec_ly");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ye" , "/nzdfserver/input/rec_ly");
  Bottle *inBot_ly;

  BufferedPort<Bottle> inPort_ry; 
  inPort_ry.open("/nzdfserver/input/rec_ry");     // Give it a name on the network.
  Network::connect("/recserver/output/right_ye" , "/nzdfserver/input/rec_ry");
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
 

  //MALLOCS:
  int psb_m,psb,psb_rest,psb_t;
  Ipp32f *res_t    = ippiMalloc_32f_C1(trsize.width,trsize.height, &psb_rest);
  Ipp8u *out       = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_l     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_r     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_d     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *dog_l     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *dog_r     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *temp      = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
  Ipp8u *rec_im_ly = ippiMalloc_8u_C1(srcsize.width,srcsize.height, &psb);
  Ipp8u *rec_im_ry = ippiMalloc_8u_C1(srcsize.width,srcsize.height, &psb);




  //Depth Processing Class:
  Depth*d = new Depth(msize,
                      17,20,9,-16,32,25,30);
//        int filterSize_,
//        int filterCap_,
//        int windowSize_,
//        int minDisparity_,
//        int numDisparities_,
//        int threshold_,
//        int uniqueness_


  DoG*dl = new DoG(msize);
  DoG*dr = new DoG(msize);

  //NZDF Optimisation Class:
  NZDFOpt *m = new NZDFOpt(msize,psb_m,&params);





  //SERVER OUTPUTS:

  //Server params:
  BufferedPort<Bottle> outPort_s;
  outPort_s.open("/nzdfserver/output/serv_params");

  // Make port for res_mask output:
  BufferedPort<Bottle> outPort_res_mask;
  outPort_res_mask.open("/nzdfserver/output/res_mask");

  BufferedPort<Bottle> outPort_fov_l;
  outPort_fov_l.open("/nzdfserver/output/fov_l");

  BufferedPort<Bottle> outPort_fov_r;
  outPort_fov_r.open("/nzdfserver/output/fov_r");

  BufferedPort<Bottle> outPort_fov_d;
  outPort_fov_d.open("/nzdfserver/output/fov_d");

  BufferedPort<Bottle> outPort_temp;
  outPort_temp.open("/nzdfserver/output/template");


  // Make a port for YARPVIEW DISPLAY
  BufferedPort<ImageOf<PixelMono> > outPort_yarpimg;
  outPort_yarpimg.open("/nzdfserver/output/yarpimg");



  //Make replier for server param probes on params port:
  //Set Replied Params:
  NZDFServerParams zsp;
  zsp.m_size=m_size;
  zsp.m_psb=psb_m;
  zsp.t_size=t_size;
  zsp.t_psb=psb_t;
  //Replier:
  NZDFReplyParamProbe replier;
  replier.reply=zsp;
  outPort_s.setReplier(replier); 

 


  int clx,cly;
  int crx,cry;
  bool cl_ok,cr_ok;

  int k=0;
  bool track = false;




  TCREATE




  //Main event loop:
  while(1)
    {   
      

      TSTART


      inBot_ly = inPort_ly.read(false);
      inBot_ry = inPort_ry.read();
      
      if (inBot_ly!=NULL &&
	  inBot_ry!=NULL ){
	
	

	rec_im_ly = (Ipp8u*) inBot_ly->get(0).asBlob();
	rec_im_ry = (Ipp8u*) inBot_ry->get(0).asBlob();
	rec_res = (RecResultParams*) inBot_ly->get(1).asBlob();
	dpix_y = rec_res->ly-rec_res->ry;
	
	
		
	//we always have a template to search for.
	
	
	



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
	  clx = 0;
	  cly = 0;
	  cl_ok = false;
	}
	//construct fov_l:
	ippiCopy_8u_C1R(&rec_im_ly[(pos_y+cly)*psb_in + pos_x+clx],psb_in,fov_l,psb_m,msize);
	
	
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
	  crx = 0;
	  cry = 0;
	  cr_ok = false;
	}
	//construct fov_r:
	ippiCopy_8u_C1R(&rec_im_ry[(pos_y+cry)*psb_in + pos_x+crx],psb_in,fov_r,psb_m,msize);






	//if we have both fovea images aligned to template.	  
	if (cl_ok && cr_ok){
	  

	  //get Disp:
	  d->proc(fov_l,fov_r,psb_m);
	  ippiCopy_8u_C1R(d->get_disp(),d->get_psb(),fov_d,psb_m,msize);

	  //get DOG: filters out low-confidence un-textured regions:
	  dl->proc(fov_l,psb_m);
	  dr->proc(fov_r,psb_m);
	  ippiCopy_8u_C1R(dl->get_dog_onoff(),d->get_psb(),dog_l,psb_m,msize);
	  ippiCopy_8u_C1R(dr->get_dog_onoff(),d->get_psb(),dog_r,psb_m,msize);



	  //DO MRF SEGMENTATION!!:
	  m->proc(fov_l,fov_r,fov_d,dog_l,dog_r);


	  
	  //evaluate segmentation result:
	  ippiCopy_8u_C1R(m->get_nzdf(),m->get_psb(),out,psb_m,msize);
	  getAreaCoGSpread(out,psb_m,msize, &area,&cog_x,&cog_y,&spread);
	  
	  
	  //if good segmentation:
	  if (area>=min_area && area<=max_area && spread<=max_spread){
	    //update template!
	    //printf("GOOD SEG. UPDATING TEMPLATE TO COG. area:%d spread:%d\n",area,spread);
	    ippiCopy_8u_C1R(&fov_l[cog_x + (msize.width-tsize.width)/2 + 
				     ((msize.height-tsize.height)/2 + cog_y)*psb_m],
			    psb_m,temp,psb_t,tsize);
	    track = true;
	  }

	}
      

	//else if there was no stereo alignment on the template,
	//update template to try to align next time on whatever is in fovea centre:
	else {
	  
	  //printf("NO ALIGNMENT. NO SEG. NEW TEMPLATE FROM CENTRE ");
	  
	  ippiSet_8u_C1R(0,out,psb_m,msize);     //clear output for display
	  ippiSet_8u_C1R(0,fov_d,psb_m,msize);   //clear output for display
	  
	  //new template from l:
	  ippiCopy_8u_C1R(&rec_im_ly[(srcsize.width-tsize.width)/2 + 
				     ((srcsize.height-tsize.height)/2)*psb],
			  psb,temp,psb_t,tsize);
	  
	  track = false;
	}
	
	
	
	
	
	
	//SEND RESULT IMS
	Bottle& tmpBot_res_mask = outPort_res_mask.prepare();
	tmpBot_res_mask.clear();
	tmpBot_res_mask.add(Value::makeBlob( out, psb_m*m_size));
	tmpBot_res_mask.addInt(area);
	tmpBot_res_mask.addInt(track);
	outPort_res_mask.write();
		
	Bottle& tmpBot_fov_l = outPort_fov_l.prepare();
	tmpBot_fov_l.clear();
	tmpBot_fov_l.add(Value::makeBlob( fov_l, psb_m*m_size));
	outPort_fov_l.write();
	
	Bottle& tmpBot_fov_r = outPort_fov_r.prepare();
	tmpBot_fov_r.clear();
	tmpBot_fov_r.add(Value::makeBlob( fov_r, psb_m*m_size));
	outPort_fov_r.write();
	
	Bottle& tmpBot_fov_d = outPort_fov_d.prepare();
	tmpBot_fov_d.clear();
	tmpBot_fov_d.add(Value::makeBlob( fov_d, psb_m*m_size));
	outPort_fov_d.write();
	
	Bottle& tmpBot_temp = outPort_temp.prepare();
	tmpBot_temp.clear();
	tmpBot_temp.add(Value::makeBlob( temp, psb_t*t_size));
	outPort_temp.write();
	
	//A yarpview compatible output port:
	//norm to 255 for display:
	normValI_8u(255,out,psb_m,msize);
	ImageOf<PixelMono>& tmp_yarpimg = outPort_yarpimg.prepare();
	tmp_yarpimg.resize(m_size,m_size);
	for (int y=0;y<m_size;y++){
	  memcpy(tmp_yarpimg.getRowArray()[y],&out[y*psb_m],m_size);
	}
	outPort_yarpimg.write();	

	
      }//got inputs
  
  
  
  
      else if (inBot_ly==NULL &&
	       inBot_ry==NULL ){
	printf("No Input\n");
	usleep(10000); //don't blow out port
      }
      
      
  
      TSTOP
    }

  //never here..
  TEND
    }




void iCub::contrib::primateVision::NZDFServer::getAreaCoGSpread(Ipp8u*im_,int psb_,IppiSize sz_,int*parea,int*pdx,int*pdy,int*spread){

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
