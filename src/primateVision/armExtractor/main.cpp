/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <string>
#include <stdio.h>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>
#include <math.h> 

#include <qimage.h>
#include <convert_rgb.h>
#include <display.h>
#include <dog.h>
#include <timing.h>
#include <norm.h>
#include <depth.h>
#include "armXopt.h"

#include <yarp/os/Property.h>


using namespace yarp::os;
using namespace std;
using namespace iCub::contrib::primateVision;






int main(int argc, char *argv[])
{

  QApplication *qa = new QApplication( argc, argv );

  //get config file parameters:
  string fname = string(getenv("DEVEL_ROOT")+string("/src/primateVision/nzdf/nzdf.cfg"));

  for (int i=1;i<argc;i++) {
    if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
      fname = argv[++i];
    }
    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
      cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
      exit(0);
    }
  }  


  Property prop;
  prop.fromConfigFile(fname.c_str());

  struct ArmXOpt::Parameters params;
  params.iter_max                  = prop.findGroup("NZDF").find("MAX_ITERATIONS").asInt();
  params.randomize_every_iteration = prop.findGroup("NZDF").find("RANDOMIZE_EVERY_ITER").asInt();
  params.int_smoothness_3sigmaon2  = prop.findGroup("NZDF").find("INT_SMOOTHNESS_3SIGMAON2").asInt();
  params.bland_dog_thresh          = prop.findGroup("NZDF").find("BLAND_DOG_THRESH").asInt();

  params.neighbour_smoothness_penalty = prop.findGroup("NZDF").find("NEIGHBOUR_SMOOTHNESS_PENALTY").asInt();
  params.int_smoothness_penalty    = prop.findGroup("NZDF").find("INT_SMOOTHNESS_PENALTY").asInt();
  params.disp_smoothness_penalty   = prop.findGroup("NZDF").find("DISP_SMOOTHNESS_PENALTY").asInt();
  params.dmap_bg_smoothness_penalty= prop.findGroup("NZDF").find("DMAP_BG_SMOOTHNESS_PENALTY").asInt();

  params.nzd_dmap_bg_data_penalty  = prop.findGroup("NZDF").find("NZD_DMAP_BG_DATA_PENALTY").asInt();
  params.nzd_dmap_fg_data_penalty  = prop.findGroup("NZDF").find("NZD_DMAP_FG_DATA_PENALTY").asInt();
  params.nzd_dmap_data_penalty     = prop.findGroup("NZDF").find("NZD_DMAP_DATA_PENALTY").asInt();
  params.nzd_fb_data_penalty       = prop.findGroup("NZDF").find("NZD_FB_DATA_PENALTY").asInt();

  params.nnzd_zd_data_penalty      = prop.findGroup("NZDF").find("NNZD_ZD_DATA_PENALTY").asInt();
  params.nnzd_dmap_bg_data_penalty = prop.findGroup("NZDF").find("NNZD_DMAP_BG_DATA_PENALTY").asInt();
  params.nnzd_dmap_fg_data_penalty = prop.findGroup("NZDF").find("NNZD_DMAP_FG_DATA_PENALTY").asInt();
  params.nnzd_dmap_data_penalty    = prop.findGroup("NZDF").find("NNZD_DMAP_DATA_PENALTY").asInt();
  params.nnzd_fb_data_penalty      = prop.findGroup("NZDF").find("NNZD_FB_DATA_PENALTY").asInt();

  params.rad_data_penalty          = prop.findGroup("NZDF").find("RAD_DATA_PENALTY").asInt();
  params.sim_data_penalty          = prop.findGroup("NZDF").find("SIM_DATA_PENALTY").asInt();


  int m_size                       = prop.findGroup("NZDF").find("M_SIZE").asInt(); //(square)
  int t_size                       = prop.findGroup("NZDFTRACK").find("T_SIZE").asInt();
  int t_lock_lr                    = prop.findGroup("NZDFTRACK").find("T_LOCK_LR").asInt();
  int t_lock_ud                    = prop.findGroup("NZDFTRACK").find("T_LOCK_UD").asInt();
  Ipp32f shift_sim_t               = prop.findGroup("NZDFTRACK").find("T_SIM").asDouble();
  int nclasses                     = 2; //nzd, not_nzd



  //load input:
 QImage *qcaml = new QImage((string(getenv("ICUB_DIR"))+string("/../devel/src/primateVision/nzdf/caml.jpg")).c_str(),"JPEG");
 QImage *qcamr = new QImage((string(getenv("ICUB_DIR"))+string("/../devel/src/primateVision/nzdf/camr.jpg")).c_str(),"JPEG");
 QImage *qsim = new QImage((string(getenv("ICUB_DIR"))+string("/../devel/src/primateVision/nzdf/siml.jpg")).c_str(),"JPEG");



  //Sizes:
  IppiSize srcsize;
  srcsize.width  = qcaml->width();
  srcsize.height = qcaml->height();
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
 

  //MALLOCS:
  int psb_m,psb,psb_rest,psb_t;
  Ipp32f *res_t    = ippiMalloc_32f_C1(trsize.width,trsize.height, &psb_rest);
  Ipp8u *feedback  = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_l     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_r     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_sim   = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *sim       = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *out       = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *fov_d     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *dog_l     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *dog_r     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
  Ipp8u *temp      = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);

 

  ippiSet_8u_C1R(0,feedback,psb_m,msize);
  ippiSet_8u_C1R(0,sim,psb_m,msize);



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
  

  Convert_RGB *cil = new Convert_RGB(srcsize);
  Convert_RGB *cir = new Convert_RGB(srcsize);
  Convert_RGB *cis = new Convert_RGB(srcsize);
  
  DoG*dl = new DoG(msize);
  DoG*dr = new DoG(msize);
  

  //NZDF Optimisation Class:
  ArmXOpt *m = new ArmXOpt(msize,psb_m,&params);
  
  
  
  int clx,cly;
  int crx,cry;
  int csx,csy;
  bool cl_ok,cr_ok,cs_ok;
  
   
  
  
  TCREATE   
    
  //Main event loop:
  TSTART
    
    
    
    
  int dpix_y = 22;
  int spix_x = -10;
  int spix_y = 5;
 
	
  cil->proc(qcaml->bits(),qcaml->width()*4);
  cir->proc(qcamr->bits(),qcamr->width()*4);
  cis->proc(qsim->bits(),qsim->width()*4); 
  
  
  
  
  //make template from l:
  ippiCopy_8u_C1R(&cil->get_y()[(srcsize.width-tsize.width)/2 + 
				((srcsize.height-tsize.height)/2)*cil->get_psb()],
		  cil->get_psb(),temp,psb_t,tsize);
  
  
  
  
  
  //redundant but oh well.. leave it for converssion to live later:
  //find temp in current_L
  //**************************
  ippiCrossCorrValid_NormLevel_8u32f_C1R(&cil->get_y()[((srcsize.height-tisize.height)/2)*cil->get_psb() +
						       (srcsize.width-tisize.width)/2],
					 cil->get_psb(),tisize,
					 temp,
					 psb_t,tsize,
					 res_t, psb_rest);
  ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
  if (max_t > shift_sim_t){
    clx = sx - trsize.width/2;
    cly = sy - trsize.height/2;
    cl_ok = true;
    printf("CL FOUND %d %d.\n",clx,cly);
  }
  else{ 
    printf("CL NOT FOUND.\n");
    clx = 0;
    cly = 0;
    cl_ok = false;
  }
  //construct fov_l:
  ippiCopy_8u_C1R(&cil->get_y()[(pos_y+cly)*cil->get_psb() + pos_x+clx],cil->get_psb(),fov_l,psb_m,msize);
  
  
  
  
  
  //find temp in current_R, 
  //**************************
  ippiCrossCorrValid_NormLevel_8u32f_C1R(&cir->get_y()[((srcsize.height-tisize.height)/2 + dpix_y)*cir->get_psb() +
						       (srcsize.width-tisize.width)/2],
					 cir->get_psb(),tisize,
					 temp,
					 psb_t,tsize,
					 res_t, psb_rest);
  ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
  if (max_t > shift_sim_t){
    crx = sx - trsize.width/2;
    cry = sy - trsize.height/2 + dpix_y;
    cr_ok = true;
    printf("CR FOUND %d %d.\n",crx,cry);
  }
  else{ 
    printf("CR NOT FOUND.\n");
    crx = 0;
    cry = 0;
    cr_ok = false;
  }
  //construct fov_r:
  ippiCopy_8u_C1R(&cir->get_y()[(pos_y+cry)*cir->get_psb() + pos_x+crx],cir->get_psb(),fov_r,psb_m,msize);
  
  
  


  //find temp in current_SIM, 
  //**************************
  ippiCrossCorrValid_NormLevel_8u32f_C1R(&cis->get_y()[((srcsize.height-tisize.height)/2 + spix_y)*cis->get_psb() +
						       (srcsize.width-tisize.width)/2 + spix_x],
					 cis->get_psb(),tisize,
					 temp,
					 psb_t,tsize,
					 res_t, psb_rest);
  ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
  if (max_t > shift_sim_t){
    csx = sx - trsize.width/2 + spix_x;
    csy = sy - trsize.height/2 + spix_y;
    cs_ok = true;
    printf("CSIM FOUND %d %d.\n",csx,csy);

  }
  else{ 
    csx = -15;//sx - trsize.width/2 + spix_x;
    csy = 25;//sy - trsize.height/2 + spix_y;
    cs_ok = false;
    printf("CSIM NOT CONFIDENT %d %d.\n",csx,csy);
  }
  //construct fov_r:
  ippiCopy_8u_C1R(&cis->get_y()[(pos_y+csy)*cis->get_psb() + pos_x+csx],cis->get_psb(),fov_sim,psb_m,msize);
  
  


  
  
  
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
    
    
    
    //blur sim image:
#define SBMSK 5    
    IppiSize smsk = {SBMSK*2+1,SBMSK*2+1};
    IppiPoint smsk_o = {SBMSK,SBMSK};
    IppiSize sbsize;
    sbsize.width  = m_size-SBMSK*2;
    sbsize.height = m_size-SBMSK*2;


    //blur feedback image:
#define FBMSK 1    
    IppiSize fmsk = {FBMSK*2+1,FBMSK*2+1};
    IppiPoint fmsk_o = {FBMSK,FBMSK};
    IppiSize fbsize;
    fbsize.width  = m_size-FBMSK*2;
    fbsize.height = m_size-FBMSK*2;



    ippiSet_8u_C1R(0,sim,psb_m,msize);
    ippiFilterBox_8u_C1R(&fov_sim[SBMSK+psb_m*SBMSK],psb_m,&sim[SBMSK+psb_m*SBMSK],psb_m,sbsize,smsk,smsk_o);
    //prep for penalty:
    normValI_8u(255,sim,psb_m,msize);
    
    
    //DISPLAY:
    iCub::contrib::primateVision::Display*disp_0 = new iCub::contrib::primateVision::Display(msize,psb_m,D_8U_NN,"FOV SIM");
    iCub::contrib::primateVision::Display*disp_1 = new iCub::contrib::primateVision::Display(msize,psb_m,D_8U_NN,"FOV L");
    iCub::contrib::primateVision::Display*disp_2 = new iCub::contrib::primateVision::Display(msize,psb_m,D_8U_NN,"FOV R");
    iCub::contrib::primateVision::Display*disp_3 = new iCub::contrib::primateVision::Display(msize,psb_m,D_8U,"DEPTH");
    iCub::contrib::primateVision::Display*disp_4 = new iCub::contrib::primateVision::Display(msize,psb_m,D_8U_NN,"SIM");
    iCub::contrib::primateVision::Display*disp_5 = new iCub::contrib::primateVision::Display(msize,m->get_psb(),D_8U,"OUT");
    iCub::contrib::primateVision::Display*disp_6 = new iCub::contrib::primateVision::Display(msize,psb_m,D_8U_NN,"FEEDBACK"); 
       
    

 




    //feedback & display loop: 
    while (1){ 
      
      printf(".\n");
      
      //DO MRF SEGMENTATION!!:
      m->proc(fov_l,fov_r,fov_d,dog_l,dog_r,sim,feedback);

      //use full bit depth:
      normVal_8u(255,m->get_out(),m->get_psb(),out,psb_m,msize);
      //blur seg for feedback:
      ippiFilterBox_8u_C1R(&out[FBMSK+psb_m*FBMSK],psb_m,&feedback[FBMSK+psb_m*FBMSK],psb_m,fbsize,fmsk,fmsk_o);
      //prep for penalty:
      normValI_8u(255,feedback,psb_m,msize);
       
         
      //DISPLAY:
      disp_0->display(fov_sim);
      disp_0->save(fov_sim,"fov_sim.jpg");
      disp_1->display(fov_l);
      disp_2->display(fov_r);
      disp_3->display(fov_d);
      disp_4->display(sim);
      disp_4->save(sim,"sim_align.jpg");
      disp_5->display(m->get_out());
      disp_5->save(m->get_out(),"out.jpg");
      disp_6->display(feedback);
    
    }//while(1)

  } 
  
}

