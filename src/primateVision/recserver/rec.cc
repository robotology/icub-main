 /*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "rec.h"
#include "recio.h"
#include "rectify.h"

#include <stdio.h>
#include <iostream>
#include <qimage.h>
#include <qstring.h>
#include <ipp.h>

//MY INCLUDES
#include <convert_rgb.h>
#include <timing.h>

#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>

using namespace yarp::os;
using namespace std; 





iCub::contrib::primateVision::RecServer::RecServer(string *cfg_)
{
  cfg=cfg_;
  start();
}


iCub::contrib::primateVision::RecServer::~RecServer()
{
  
}


void iCub::contrib::primateVision::RecServer::exit(){
  motion_handler->stop(); 
}



void iCub::contrib::primateVision::RecServer::run()
{
  
  Property prop;
  prop.fromConfigFile(cfg->c_str());
  string crfl = prop.findGroup("REC").find("CALIB_RESULTS_FILE_L").asString().c_str();
  string crfr = prop.findGroup("REC").find("CALIB_RESULTS_FILE_R").asString().c_str();
  double toff_r = prop.findGroup("REC").find("CAM_R_TILT_OFFSET").asDouble();
  int width = prop.findGroup("REC").find("WIDTH").asInt();
  int height = prop.findGroup("REC").find("HEIGHT").asInt();
  bool motion = (bool) prop.findGroup("REC").find("MOTION").asInt();
  bool realOrSim = (bool) prop.findGroup("REC").find("REAL0_SIM1").asInt();

  bool do_bar_l = true;
  bool do_bar_r = true;
  if (crfl == "none"){
    do_bar_l = false;
    printf("RecServer: Not barrel rectifying LEFT\n");
  }
  if (crfr == "none"){
    do_bar_r = false;
    printf("RecServer: Not barrel rectifying RIGHT\n");
  }

  std::string sim;
  if (realOrSim){
    printf("RecServer: Connecting to iCub Simulator.\n");
    sim = "Sim";
  }
  else{
    sim = "";
  }
  
  Property options;
  IEncoders *enc;
  IVelocityControl *vel;
  PolyDriver *recDevice;
  options.put("device", "remote_controlboard");
  if (realOrSim){
    options.put("local",  "/recserverSim/rcb_client"); 
    options.put("remote", "/icubSim/head");
  }
  else{
    options.put("local",  "/recserver/rcb_client"); 
    options.put("remote", "/icub/head");
  }
  
  recDevice = new PolyDriver(options);
  if (!recDevice->isValid()) {
    printf("RecServer: PolyDriver device not available.\n");
  }
  recDevice->view(vel);
  recDevice->view(enc);
  if (vel==NULL || enc==NULL){     
    recDevice->close();
    printf("RecServer: No robot found.\n");
  }
    
  //for initial vel,acc,pos:
  double vec_enc[6];
  double vec_enc_tmp[6];
  enc->getEncoders(vec_enc);

  double ref_accs[6];
  ref_accs[0] = 1000000000.0;
  ref_accs[1] = 1000000000.0;
  ref_accs[2] = 1000000000.0;
  ref_accs[3] = 1000000000.0;
  ref_accs[4] = 1000000000.0;
  ref_accs[5] = 1000000000.0;
  double desired_angles[6];
  desired_angles[0] = 0.0;
  desired_angles[1] = 0.0;
  desired_angles[2] = 0.0;
  desired_angles[3] = 0.0;
  desired_angles[4] = 0.1; //vergence, initalise non-zero to pass update check.
  desired_angles[5] = 0.0; //version
  double t_deg,l_deg,r_deg,vergence_deg,version_deg;
  double old_t_deg=0.0,old_l_deg=0.0,old_r_deg=0.0;
  bool do_l,do_r;

  //to read inertial port:
  BufferedPort<yarp::sig::Vector> inPort_inertial;
  inPort_inertial.open(("/recserver"+sim+"/input/inertial").c_str());
  Network::connect(("/icub"+sim+"/inertial").c_str() , ("/recserver"+sim+"/input/inertial").c_str());

  //incoming motion commands:
  BufferedPort<BinPortable<RecMotionRequest> > inPort_mot; 
  inPort_mot.open( ("/recserver"+sim+"/input/motion").c_str());



  //SERVER I/O:
  //server param request & reply:
  BufferedPort<BinPortable<RecServerParams> > outPort_s;

  //IN:
  //Image Input:
  BufferedPort<ImageOf<PixelBgr> > pcl,pcr;
 
  //OUT:
  //Param result:
  RecResultParams rec_res_params;
  BufferedPort<BinPortable<RecResultParams> > outPort_p; 
  //BARREL RECT
  BufferedPort<Bottle> outPort_lyb;
  BufferedPort<Bottle> outPort_lub;
  BufferedPort<Bottle> outPort_lvb;
  BufferedPort<Bottle> outPort_ryb;
  BufferedPort<Bottle> outPort_rub;
  BufferedPort<Bottle> outPort_rvb;
  //EPRECT
  BufferedPort<Bottle> outPort_lye;
  BufferedPort<Bottle> outPort_lue;
  BufferedPort<Bottle> outPort_lve;
  BufferedPort<Bottle> outPort_rye;
  BufferedPort<Bottle> outPort_rue;
  BufferedPort<Bottle> outPort_rve;

  //Open the ports:
  outPort_s.open(  ("/recserver"+sim+"/output/serv_params").c_str());
  outPort_p.open(  ("/recserver"+sim+"/output/rec_params").c_str());
  outPort_lyb.open(("/recserver"+sim+"/output/left_yb").c_str() );
  outPort_lub.open(("/recserver"+sim+"/output/left_ub").c_str() );
  outPort_lvb.open(("/recserver"+sim+"/output/left_vb").c_str() );
  outPort_ryb.open(("/recserver"+sim+"/output/right_yb").c_str());
  outPort_rub.open(("/recserver"+sim+"/output/right_ub").c_str());
  outPort_rvb.open(("/recserver"+sim+"/output/right_vb").c_str());
  outPort_lye.open(("/recserver"+sim+"/output/left_ye").c_str() );
  outPort_lue.open(("/recserver"+sim+"/output/left_ue").c_str() );
  outPort_lve.open(("/recserver"+sim+"/output/left_ve").c_str() );
  outPort_rye.open(("/recserver"+sim+"/output/right_ye").c_str());
  outPort_rue.open(("/recserver"+sim+"/output/right_ue").c_str());
  outPort_rve.open(("/recserver"+sim+"/output/right_ve").c_str());

  pcl.open(("/recserver"+sim+"/input/image/left").c_str());
  pcr.open(("/recserver"+sim+"/input/image/right").c_str());
  Network::connect(("/icub"+sim+"/cam/left").c_str() , ("/recserver"+sim+"/input/image/left").c_str());
  Network::connect(("/icub"+sim+"/cam/right").c_str() ,("/recserver"+sim+"/input/image/right").c_str());
  




  //Get first RGB image to establish width, height:
  double scale = 1.0;
  QImage *qim_l,*qim_r;
  ImageOf<PixelBgr> *imgl,*imgr;
  IppiSize insize;
  IppiRect inroi;
  imgl = pcl.read(); //blocking buffered
  insize.width = imgl->width();
  insize.height = imgl->height();
  inroi.x=0;
  inroi.y=0;
  inroi.width=imgl->width();
  inroi.height=imgl->height();
  //get scale factor to reduce to widthxheight:
  scale  = ((double)width)/imgl->width();
  printf("RecServer: Received input image dimensions: (%d,%d)\n",imgl->width(), imgl->height());
  printf("RecServer: Scaling to image dimensions: (%d,%d). Scale factor %f\n",width, height,scale);

  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;


  int mos_width = 3*width;
  int mos_height = 3*height;
  printf("RecServer: Mosaic dimensions: (%d,%d)\n",mos_width,mos_height);

  
  //Image allocs:
  int psb,psb4,psb3_in;
  Ipp8u *colourl_in = ippiMalloc_8u_C3(width,height,&psb3_in);
  Ipp8u *colourr_in = ippiMalloc_8u_C3(width,height,&psb3_in);
  Ipp8u *colourl  = ippiMalloc_8u_C4(width,height,&psb4);
  Ipp8u *colourr  = ippiMalloc_8u_C4(width,height,&psb4);
  Ipp8u *yl_brect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *yr_brect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *ul_brect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *ur_brect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *vl_brect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *vr_brect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *yl_eprect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *yr_eprect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *ul_eprect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *ur_eprect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *vl_eprect = ippiMalloc_8u_C1(width,height,&psb);
  Ipp8u *vr_eprect = ippiMalloc_8u_C1(width,height,&psb);


  //PROCESSING CLASSES:
  Rectify *rl = new Rectify((char*)crfl.c_str(),srcsize, 0.0); 
  Rectify *rr = new Rectify((char*)crfr.c_str(),srcsize, toff_r); 
  int focus = (rl->get_focus() + rr->get_focus())/2;
  //Image conversion classes:
  Convert_RGB *c_rgb2yuv_l = new Convert_RGB(srcsize);
  Convert_RGB *c_rgb2yuv_r = new Convert_RGB(srcsize);


  // Make Handler for motion requests:
  motion_handler = new RecHandleMotionRequest(5);
  motion_handler->pix2degx    = prop.findGroup("REC").find("PIX2DEGX").asDouble();;
  motion_handler->pix2degy    = prop.findGroup("REC").find("PIX2DEGY").asDouble();;
  motion_handler->k_vel_verg  = prop.findGroup("REC").find("KVEL_VERG").asDouble();
  motion_handler->k_vel_vers  = prop.findGroup("REC").find("KVEL_VERS").asDouble();
  motion_handler->k_vel_tilt  = prop.findGroup("REC").find("KVEL_TILT").asDouble();
  motion_handler->k_vel_roll  = prop.findGroup("REC").find("KVEL_ROLL").asDouble();
  motion_handler->k_vel_pitch = prop.findGroup("REC").find("KVEL_PITCH").asDouble();
  motion_handler->k_vel_yaw   = prop.findGroup("REC").find("KVEL_YAW").asDouble();
  motion_handler->vor_on      = (bool) prop.findGroup("VOR").find("VOR_ON").asInt();
  motion_handler->vor_k_rol   = prop.findGroup("VOR").find("K_ROL").asDouble();
  motion_handler->vor_k_pan   = prop.findGroup("VOR").find("K_PAN").asDouble();
  motion_handler->vor_k_tlt   = prop.findGroup("VOR").find("K_TLT").asDouble();
  motion_handler->vor_d_rol   = prop.findGroup("VOR").find("D_ROL").asDouble();
  motion_handler->vor_d_pan   = prop.findGroup("VOR").find("D_PAN").asDouble();
  motion_handler->vor_d_tlt   = prop.findGroup("VOR").find("D_TLT").asDouble();
  motion_handler->motion      = motion;
  motion_handler->enc         = vec_enc;
  motion_handler->vel         = vel;
  motion_handler->encs        = enc;
  motion_handler->inPort_mot  = &inPort_mot;
  motion_handler->inPort_inertial  = &inPort_inertial;

  motion_handler->start();


  //Make replier for server param probes on server port:
  //Set Params:
  RecServerParams rsp;
  rsp.width=width;
  rsp.height=height;
  rsp.mos_width=mos_width;
  rsp.mos_height=mos_height;
  rsp.psb=psb;
  rsp.focus=focus;
  //Replier:
  RecReplyParamProbe server_replier;
  server_replier.reply=rsp;
  outPort_s.setReplier(server_replier); 


    
  if (motion){         
    //SET REF ACC:
    vel->setRefAccelerations(ref_accs);
    //set initial desired positions to current pos:
    enc->getEncoders(vec_enc);
    for (int i=0;i<6;i++){
      desired_angles[i] = vec_enc[i];
    }
  }
  





  //create a timer:
  TCREATE




  //main event loop:
  printf("RecServer: Begin..\n");
  while(1){
    


    TSTART
      
      

    

    
   
    //REQUEST IMAGES:
    imgr = pcr.read(false); //non-blocking
    imgl = pcl.read(false); //non-blocking
    
    //get encoder data:
    if (enc->getEncoders(vec_enc_tmp)){
      for (int i=0;i<6;i++){
	vec_enc[i] = vec_enc_tmp[i];
      }
    }    


 
    vergence_deg = vec_enc[4];
    version_deg  = vec_enc[5];
    t_deg = -vec_enc[3];
    l_deg = -(version_deg + vergence_deg);
    r_deg = (version_deg - vergence_deg);

    
    //send rec params (leave for TSBServer!):
    rec_res_params.lx = rl->get_ix();
    rec_res_params.rx = rr->get_ix();
    rec_res_params.ly = rl->get_iy();
    rec_res_params.ry = rr->get_iy();
    rec_res_params.deg_lx = l_deg;
    rec_res_params.deg_rx = r_deg;
    rec_res_params.deg_ly = t_deg;
    rec_res_params.deg_ry = t_deg + toff_r;
    rec_res_params.head_r = -vec_enc[1];
    rec_res_params.head_p = -vec_enc[0];
    rec_res_params.head_y = -vec_enc[2];
    outPort_p.prepare().content() = rec_res_params;
    outPort_p.write();

    
   
    //RECTIFY IMAGES IF READY:
 
    //LEFT:
    if (imgl!=NULL){

      //see if geometry has changed:
      do_l = true;
      if (t_deg == old_t_deg && l_deg == old_l_deg){do_l = false;}
      //calc rec params if necessary:
      if (do_l){
	rl->proc(t_deg, l_deg); 
      }
      old_l_deg = l_deg;
      old_t_deg = t_deg;
      
      if (scale==1.0){
	//L: convert directly to RGBA:
	ippiCopy_8u_C3AC4R(imgl->getPixelAddress(0,0),imgl->width()*3,colourl,psb4,srcsize);
      }
      else{
	//L: scale to width,height:
	ippiResize_8u_C3R(imgl->getPixelAddress(0,0),insize,imgl->width()*3,
			  inroi,
			  colourl_in,psb3_in,srcsize,
			  scale,
			  scale,
			  IPPI_INTER_CUBIC);
	
	//L: convert to RGBA:
	ippiCopy_8u_C3AC4R(colourl_in,psb3_in,colourl,psb4,srcsize);
      }
      
      //convert to Y,U,V image channels:
      c_rgb2yuv_l->proc(colourl,psb4);
      
      
      
      //CALC AND SEND RECTIFIED IMAGES IMMEDIATELY!
      
      //update output params to attach to ims:
      rec_res_params.lx = rl->get_ix();
      rec_res_params.rx = rr->get_ix();
      rec_res_params.ly = rl->get_iy();
      rec_res_params.ry = rr->get_iy();
      rec_res_params.deg_lx = l_deg;
      rec_res_params.deg_rx = r_deg;
      rec_res_params.deg_ly = t_deg;
      rec_res_params.deg_ry = t_deg + toff_r;
      rec_res_params.head_r = -vec_enc[1];
      rec_res_params.head_p = -vec_enc[0];
      rec_res_params.head_y = -vec_enc[2];
      
      
      //BARREL RECTIFIED IMS + PARAMS:
      if (do_bar_l){
	rl->barrel_rect(c_rgb2yuv_l->get_y(),c_rgb2yuv_l->get_psb(),yl_brect,psb);
	rl->barrel_rect(c_rgb2yuv_l->get_u(),c_rgb2yuv_l->get_psb(),ul_brect,psb);
	rl->barrel_rect(c_rgb2yuv_l->get_v(),c_rgb2yuv_l->get_psb(),vl_brect,psb);
      }
      else{
	ippiCopy_8u_C1R(c_rgb2yuv_l->get_y(),c_rgb2yuv_l->get_psb(),yl_brect,psb,srcsize);
	ippiCopy_8u_C1R(c_rgb2yuv_l->get_u(),c_rgb2yuv_l->get_psb(),ul_brect,psb,srcsize);
	ippiCopy_8u_C1R(c_rgb2yuv_l->get_v(),c_rgb2yuv_l->get_psb(),vl_brect,psb,srcsize);
      }
      
      Bottle& outBot_lyb = outPort_lyb.prepare();
      outBot_lyb.clear();
      outBot_lyb.add(Value::makeBlob(yl_brect, psb*height));
      outBot_lyb.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_lyb.write();
      
      Bottle& outBot_lub = outPort_lub.prepare();
      outBot_lub.clear();
      outBot_lub.add(Value::makeBlob(ul_brect, psb*height));
      outBot_lub.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_lub.write();
      
      Bottle& outBot_lvb = outPort_lvb.prepare();
      outBot_lvb.clear();
      outBot_lvb.add(Value::makeBlob(vl_brect, psb*height));
      outBot_lvb.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_lvb.write();

      //EPIPOLAR RECTIFIED IMS + PARAMS
      rl->epipolar_rect(yl_brect,psb,yl_eprect,psb);
      Bottle& outBot_lye = outPort_lye.prepare();
      outBot_lye.clear();
      outBot_lye.add(Value::makeBlob(yl_eprect, psb*height));
      outBot_lye.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_lye.write();
      
      rl->epipolar_rect(ul_brect,psb,ul_eprect,psb);
      Bottle& outBot_lue = outPort_lue.prepare();
      outBot_lue.clear();
      outBot_lue.add(Value::makeBlob(ul_eprect, psb*height));
      outBot_lue.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_lue.write();
      
      rl->epipolar_rect(vl_brect,psb,vl_eprect,psb);
      Bottle& outBot_lve = outPort_lve.prepare();
      outBot_lve.clear();
      outBot_lve.add(Value::makeBlob(vl_eprect, psb*height));
      outBot_lve.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_lve.write();
      
    }
    
    
    
   
    
    //RIGHT:
    if (imgr!=NULL){

      //see if geometry has changed:
      do_r = true;
      if (t_deg == old_t_deg && r_deg == old_r_deg){do_r = false;}      
      //calc rec params if necessary:
      if (do_r){
	rr->proc(t_deg, r_deg); 
      }
      old_r_deg = r_deg;
      old_t_deg = t_deg;
      
      if (scale==1.0){
	//R: convert directly to RGBA:
	ippiCopy_8u_C3AC4R(imgr->getPixelAddress(0,0),imgr->width()*3,colourr,psb4,srcsize);
      }
      else{
	//R: scale to width,height:
	ippiResize_8u_C3R(imgr->getPixelAddress(0,0),insize,imgr->width()*3,
			  inroi,
			  colourr_in,psb3_in,srcsize,
			  scale,
			  scale,
			  IPPI_INTER_CUBIC);
	
	//R: convert to RGBA:
	ippiCopy_8u_C3AC4R(colourr_in,psb3_in,colourr,psb4,srcsize);
      }
      
      //convert to Y,U,V image channels:
      c_rgb2yuv_r->proc(colourr,psb4);
      
      
      
      //CALC AND SEND RECTIFIED IMAGES IMMEDIATELY!

      //update output params to attach to ims:
      rec_res_params.lx = rl->get_ix();
      rec_res_params.rx = rr->get_ix();
      rec_res_params.ly = rl->get_iy();
      rec_res_params.ry = rr->get_iy();
      rec_res_params.deg_lx = l_deg;
      rec_res_params.deg_rx = r_deg;
      rec_res_params.deg_ly = t_deg;
      rec_res_params.deg_ry = t_deg + toff_r;
      rec_res_params.head_r = -vec_enc[1];
      rec_res_params.head_p = -vec_enc[0];
      rec_res_params.head_y = -vec_enc[2];


      //BARREL RECTIFIED IMS + PARAMS:
      if (do_bar_r){
	rr->barrel_rect(c_rgb2yuv_r->get_y(),c_rgb2yuv_r->get_psb(),yr_brect,psb);
	rr->barrel_rect(c_rgb2yuv_r->get_u(),c_rgb2yuv_r->get_psb(),ur_brect,psb);
	rr->barrel_rect(c_rgb2yuv_r->get_v(),c_rgb2yuv_r->get_psb(),vr_brect,psb);
      }
      else{
	ippiCopy_8u_C1R(c_rgb2yuv_r->get_y(),c_rgb2yuv_r->get_psb(),yr_brect,psb,srcsize);
	ippiCopy_8u_C1R(c_rgb2yuv_r->get_u(),c_rgb2yuv_r->get_psb(),ur_brect,psb,srcsize);
	ippiCopy_8u_C1R(c_rgb2yuv_r->get_v(),c_rgb2yuv_r->get_psb(),vr_brect,psb,srcsize);
      }
      
      Bottle& outBot_ryb = outPort_ryb.prepare();
      outBot_ryb.clear();
      outBot_ryb.add(Value::makeBlob(yr_brect, psb*height));
      outBot_ryb.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_ryb.write();
 
      Bottle& outBot_rub = outPort_rub.prepare();
      outBot_rub.clear();
      outBot_rub.add(Value::makeBlob(ur_brect, psb*height));
      outBot_rub.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_rub.write();
      
      Bottle& outBot_rvb = outPort_rvb.prepare();
      outBot_rvb.clear();
      outBot_rvb.add(Value::makeBlob(vr_brect, psb*height));
      outBot_rvb.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_rvb.write();
      

      //EPIPOLAR RECTIFIED IMS + PARAMS
      rr->epipolar_rect(yr_brect,psb,yr_eprect,psb);
      Bottle& outBot_rye = outPort_rye.prepare();
      outBot_rye.clear();
      outBot_rye.add(Value::makeBlob(yr_eprect, psb*height));
      outBot_rye.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_rye.write();
      
      rr->epipolar_rect(ur_brect,psb,ur_eprect,psb);
      Bottle& outBot_rue = outPort_rue.prepare();
      outBot_rue.clear();
      outBot_rue.add(Value::makeBlob(ur_eprect, psb*height));
      outBot_rue.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_rue.write();
      
      rr->epipolar_rect(vr_brect,psb,vr_eprect,psb);
      Bottle& outBot_rve = outPort_rve.prepare();
      outBot_rve.clear();
      outBot_rve.add(Value::makeBlob(vr_eprect, psb*height));
      outBot_rve.add(Value::makeBlob(&rec_res_params,sizeof(RecResultParams)));
      outPort_rve.write();
      
    }
    


    




    //prevent port saturation:
    if (imgr==NULL && imgl==NULL){
      printf("No Input\n");
      usleep(5000); //dont blow out port
    }
    
    
    TSTOP    
    
  }
  


  TEND

}


