/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_recclient RecClient
 *
 *This endclient module RecClient displays output from the RecServer, as well as providing sample code showing how to make a useful RecServer client module. N.B, most other processing modules in the primateVision framework are clients of the RecServer, based on this code, but with additional processing and differing outputs. Notice also that this client first probes the RecServer to obtain image size and other parameters!
 *
 * This module should be called in the following way:\n \b recclient \b\n
 *\n
 * Ports accessed:\n
 * <ul>
 * <li> /recserver/output/left_ye
 * <li> /recserver/output/right_ye
 * <li> /recserver/output/rec_params
 * <li> /recserver/input/motion
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /recclient/input/left_ye
 * <li> /recclient/input/right_ye
 * <li> /recclient/input/rec_params
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /recclient/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/recserver/recclient/main.cc 
 * 
 */


/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include <display.h>
#include <mosaic.h>
#include <recio.h>
 

#define SHOW_MOSAICS 1
#define SHOW_Y_IMS 0
#define SHOW_Y_IMS_BRECT 0
#define SHOW_UV_IMS 0

 
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  QString arg1 = argv[1];
  QString arg2 = argv[2];
  QString arg3 = argv[3];

  QString sim;
  if (arg1=="sim" || arg2=="sim" || arg3=="sim"){
    sim = "Sim";
    printf("RecClient: Connecting to iCubSim\n");
  }

  bool mot=false;
  if (arg1=="motion" || arg2=="motion" || arg3=="motion"){
    mot = true;
    printf("RecClient: Motion test enabled.\n");
  }

  bool save=false;
  if (arg1=="save" || arg2=="save" || arg3=="save"){
    save = true;
    printf("RecClient: Saving images.\n");
  }


  //get server params:
  Port inPort_s;
  inPort_s.open("/recclient"+sim+"/input/serv_params");
  Network::connect("/recclient"+sim+"/input/serv_params","/recserver"+sim+"/output/serv_params");
  Network::connect("/recserver"+sim+"/output/serv_params","/recclient"+sim+"/input/serv_params");
  BinPortable<RecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;
  //check both l and r rec results as may arrive asynchronously:
  RecResultParams* rec_res_l;
  RecResultParams* rec_res_r;


  //ACCESS MOTION LIKE THIS!
  Port outPort_mot;
  outPort_mot.open("/recclient"+sim+"/output/mot");
  Network::connect("/recclient"+sim+"/output/mot", "/recserver"+sim+"/input/motion");
  Network::connect("/recserver"+sim+"/input/motion", "/recclient"+sim+"/output/mot");
  BinPortable<RecMotionRequest> motion_request;

  //initalise:
  motion_request.content().pix_y  = 0;
  motion_request.content().pix_xl = 0;
  motion_request.content().pix_xr = 0;
  motion_request.content().deg_r  = 0.0;
  motion_request.content().deg_p  = 0.0;
  motion_request.content().deg_y  = 0.0;
  motion_request.content().relative = false; //gonna send absolute moves!
  motion_request.content().suspend = 0; 
  motion_request.content().lockto = NO_LOCK; 
  motion_request.content().unlock = true; 



  int width = rsp.width;
  int height = rsp.height;
  int psb = rsp.psb;
  IppiSize srcsize;
  srcsize.width = width;
  srcsize.height = height;

  int save_num = 0;


#if SHOW_Y_IMS || SHOW_MOSAICS
  BufferedPort<Bottle> inPort_ly;      // Create a ports
  inPort_ly.open("/recclient"+sim+"/input/rec_ly");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/left_ye" , "/recclient"+sim+"/input/rec_ly");
  Bottle *inBot_ly;
  Ipp8u* rec_im_ly;

  BufferedPort<Bottle> inPort_ry;      // Create a ports
  inPort_ry.open("/recclient"+sim+"/input/rec_ry");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/right_ye" , "/recclient"+sim+"/input/rec_ry");
  Bottle *inBot_ry;
  Ipp8u* rec_im_ry;
#endif


#if SHOW_Y_IMS_BRECT
  BufferedPort<Bottle> inPort_lyb;      // Create a ports
  inPort_lyb.open("/recclient"+sim+"/input/rec_lyb");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/left_yb" , "/recclient"+sim+"/input/rec_lyb");
  Bottle *inBot_lyb;
  Ipp8u* rec_im_lyb;

  BufferedPort<Bottle> inPort_ryb;      // Create a ports
  inPort_ryb.open("/recclient"+sim+"/input/rec_ryb");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/right_yb" , "/recclient"+sim+"/input/rec_ryb");
  Bottle *inBot_ryb;
  Ipp8u* rec_im_ryb;
#endif


#if SHOW_UV_IMS
  BufferedPort<Bottle> inPort_lu;      // Create a ports
  inPort_lu.open("/recclient"+sim+"/input/rec_lu");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/left_ue" , "/recclient"+sim+"/input/rec_lu");
  Bottle *inBot_lu;
  Ipp8u* rec_im_lu;

  BufferedPort<Bottle> inPort_lv;      // Create a ports
  inPort_lv.open("/recclient"+sim+"/input/rec_lv");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/left_ve" , "/recclient"+sim+"/input/rec_lv");
  Bottle *inBot_lv;
  Ipp8u* rec_im_lv;

  BufferedPort<Bottle> inPort_ru;      // Create a ports
  inPort_ru.open("/recclient"+sim+"/input/rec_ru");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/right_ue" , "/recclient"+sim+"/input/rec_ru");
  Bottle *inBot_ru;
  Ipp8u* rec_im_ru;

  BufferedPort<Bottle> inPort_rv;      // Create a ports
  inPort_rv.open("/recclient"+sim+"/input/rec_rv");     // Give it a name on the network.
  Network::connect("/recserver"+sim+"/output/right_ve" , "/recclient"+sim+"/input/rec_rv");
  Bottle *inBot_rv;
  Ipp8u* rec_im_rv;
#endif



  //LAUNCH GUI WINDOWS:
#if SHOW_Y_IMS
  iCub::contrib::primateVision::Display *d_ly = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"YL_REC");
  iCub::contrib::primateVision::Display *d_ry = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"YR_REC");
#endif
#if SHOW_Y_IMS_BRECT
  iCub::contrib::primateVision::Display *d_lyb = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"YL_REC_BRECT");
  iCub::contrib::primateVision::Display *d_ryb = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"YR_REC_BRECT");
#endif
#if SHOW_UV_IMS
  iCub::contrib::primateVision::Display *d_lu = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"UL_REC");
  iCub::contrib::primateVision::Display *d_lv = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"VL_REC");
  iCub::contrib::primateVision::Display *d_ru = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"UR_REC");
  iCub::contrib::primateVision::Display *d_rv = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"VR_REC");
#endif
#if SHOW_MOSAICS
  IppiSize mossize = {rsp.mos_width,rsp.mos_height};
  Mosaic *ml = new Mosaic(mossize,srcsize,psb,D_8U_NN,"REC CLIENT TEST L");
  Mosaic *mr = new Mosaic(mossize,srcsize,psb,D_8U_NN,"REC CLIENT TEST R");
#endif



  int k=0;




  while (1){

    if (mot){
      //SEND POSITIONS LIKE THIS!
      motion_request.content().pix_xl = (int) 200.0*sin(((double)k)/100.0);
      motion_request.content().pix_xr = (int) 200.0*sin(((double)k)/100.0);
      motion_request.content().pix_y  = (int) 150.0*sin(((double)k)/80.0);
      printf("RecClient: Moving (%d,%d,%d)\n",
	     motion_request.content().pix_xl,
	     motion_request.content().pix_xr,
	     motion_request.content().pix_y);
      outPort_mot.write(motion_request);
    }
    k++;

   //LEFT:
#if SHOW_Y_IMS_BRECT
    inBot_lyb = inPort_lyb.read(false);
    inBot_ryb = inPort_ryb.read(false);
#endif
#if SHOW_UV_IMS
    inBot_lu = inPort_lu.read(false);
    inBot_lv = inPort_lv.read(false);
    inBot_ru = inPort_ru.read(false);
    inBot_rv = inPort_rv.read(false);
#endif
#if SHOW_Y_IMS || SHOW_MOSAICS
    inBot_ly = inPort_ly.read(false);
    inBot_ry = inPort_ry.read();//blocking
#endif




    //LEFT:
#if SHOW_Y_IMS || SHOW_MOSAICS
    if (inBot_ly!=NULL){
      rec_im_ly = (Ipp8u*) inBot_ly->get(0).asBlob();
      rec_res_l = (RecResultParams*) inBot_ly->get(1).asBlob();
#endif
#if SHOW_Y_IMS
      d_ly->display(rec_im_ly);
#endif
#if SHOW_MOSAICS
      ml->display(rec_im_ly,rec_res_l->lx,rec_res_l->ly);
#endif
#if SHOW_Y_IMS || SHOW_MOSAICS
    }
#endif

    //RIGHT:
#if SHOW_Y_IMS || SHOW_MOSAICS
    if (inBot_ry!=NULL){
      rec_im_ry = (Ipp8u*) inBot_ry->get(0).asBlob();
      rec_res_r = (RecResultParams*) inBot_ry->get(1).asBlob();
#endif
#if SHOW_Y_IMS
      d_ry->display(rec_im_ry);
#endif
#if SHOW_MOSAICS
      mr->display(rec_im_ry,rec_res_r->rx,rec_res_r->ry);
#endif
#if SHOW_Y_IMS || SHOW_MOSAICS
    }
#endif
    


#if SHOW_Y_IMS_BRECT
    if (inBot_lyb!=NULL){
      rec_im_lyb = (Ipp8u*) inBot_lyb->get(0).asBlob();
      d_lyb->display(rec_im_lyb);
    }
    if (inBot_ryb!=NULL){
      rec_im_ryb = (Ipp8u*) inBot_ryb->get(0).asBlob();
      d_ryb->display(rec_im_ryb);
    }
#endif

#if SHOW_UV_IMS
    if (inBot_lu!=NULL){
      rec_im_lu = (Ipp8u*) inBot_lu->get(0).asBlob();
      d_lu->display(rec_im_lu);
    }
    if (inBot_lv!=NULL){
      rec_im_lv = (Ipp8u*) inBot_lv->get(0).asBlob();
      d_lv->display(rec_im_lv);
    }
    if (inBot_ru!=NULL){
      rec_im_ru = (Ipp8u*) inBot_ru->get(0).asBlob();
      d_ru->display(rec_im_ru);
    }
    if (inBot_rv!=NULL){
      rec_im_rv = (Ipp8u*) inBot_rv->get(0).asBlob();
      d_rv->display(rec_im_rv);
    }
#endif



   //SAVE:

    if (save) {

#if SHOW_Y_IMS
      d_ly->save(rec_im_ly,"rec_ly_"+QString::number(save_num)+".jpg");
      d_ry->save(rec_im_ry,"rec_ry_"+QString::number(save_num)+".jpg");
#endif

#if SHOW_MOSAICS
      ml->save(rec_im_ly,"mos_ly_"+QString::number(save_num)+".jpg");
      mr->save(rec_im_ry,"mos_ry_"+QString::number(save_num)+".jpg");
#endif

#if SHOW_Y_IMS_BRECT
      d_lyb->save(rec_im_lyb,"rec_lyb_"+QString::number(save_num)+".jpg");
      d_ryb->save(rec_im_ryb,"rec_ryb_"+QString::number(save_num)+".jpg");
#endif

#if SHOW_UV_IMS
      d_lu->save(rec_im_lu,"rec_lu_"+QString::number(save_num)+".jpg");
      d_lv->save(rec_im_lv,"rec_lv_"+QString::number(save_num)+".jpg");
      d_ru->save(rec_im_ru,"rec_ru_"+QString::number(save_num)+".jpg");
      d_rv->save(rec_im_rv,"rec_rv_"+QString::number(save_num)+".jpg");
#endif
      save_num++;
    }


    if (inBot_ly==NULL && inBot_ry==NULL){
      usleep(1000);
    }

  }

  //never here!
  
}
