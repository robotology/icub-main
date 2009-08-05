/**
 * @ingroup icub_primatevision_recserver
 * \defgroup icub_primatevision_recserver_recclient_motion RecClient_Motion
 *
 *This endclient module RecClient_Motion displays output from the RecServer and demonstrates motion using the RecServer API, as well as providing sample code showing how to make a useful RecServer client module. Moves eyes with sinusoidal motion
 *
 * This module should be called in the following way:\n \b recclient_motion \b\n
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
 * This file can be edited at src/primateVision/recserver/recclient/main_mot.cc 
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
 

#define SHOW_Y_IMS 0
#define SHOW_Y_IMS_BRECT 0
#define SHOW_UV_IMS 0
#define SHOW_MOSAICS 1

#define SAVE 0


using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  Port inPort_s;
  inPort_s.open("/recclient_mot/input/serv_params");
  Network::connect("/recclient_mot/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/recclient_mot/input/serv_params");
  BinPortable<RecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;
  //check both l and r images as may arrive asynchronously!
  RecResultParams* rec_res_l;
  RecResultParams* rec_res_r;


  //ACCESS MOTION LIKE THIS!
  Port outPort_mot;
  outPort_mot.open("/recclient_mot/output/mot");
  Network::connect("/recclient_mot/output/mot", "/recserver/input/motion");
  Network::connect("/recserver/input/motion", "/recclient_mot/output/mot");
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
  inPort_ly.open("/recclient/input/rec_ly");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ye" , "/recclient/input/rec_ly");
  Bottle *inBot_ly;
  Ipp8u* rec_im_ly;

  BufferedPort<Bottle> inPort_ry;      // Create a ports
  inPort_ry.open("/recclient/input/rec_ry");     // Give it a name on the network.
  Network::connect("/recserver/output/right_ye" , "/recclient/input/rec_ry");
  Bottle *inBot_ry;
  Ipp8u* rec_im_ry;
#endif


#if SHOW_Y_IMS_BRECT
  BufferedPort<Bottle> inPort_lyb;      // Create a ports
  inPort_lyb.open("/recclient/input/rec_lyb");     // Give it a name on the network.
  Network::connect("/recserver/output/left_yb" , "/recclient/input/rec_lyb");
  Bottle *inBot_lyb;
  Ipp8u* rec_im_lyb;

  BufferedPort<Bottle> inPort_ryb;      // Create a ports
  inPort_ryb.open("/recclient/input/rec_ryb");     // Give it a name on the network.
  Network::connect("/recserver/output/right_yb" , "/recclient/input/rec_ryb");
  Bottle *inBot_ryb;
  Ipp8u* rec_im_ryb;
#endif


#if SHOW_UV_IMS
  BufferedPort<Bottle> inPort_lu;      // Create a ports
  inPort_lu.open("/recclient/input/rec_lu");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ue" , "/recclient/input/rec_lu");
  Bottle *inBot_lu;
  Ipp8u* rec_im_lu;

  BufferedPort<Bottle> inPort_lv;      // Create a ports
  inPort_lv.open("/recclient/input/rec_lv");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ve" , "/recclient/input/rec_lv");
  Bottle *inBot_lv;
  Ipp8u* rec_im_lv;

  BufferedPort<Bottle> inPort_ru;      // Create a ports
  inPort_ru.open("/recclient/input/rec_ru");     // Give it a name on the network.
  Network::connect("/recserver/output/right_ue" , "/recclient/input/rec_ru");
  Bottle *inBot_ru;
  Ipp8u* rec_im_ru;

  BufferedPort<Bottle> inPort_rv;      // Create a ports
  inPort_rv.open("/recclient/input/rec_rv");     // Give it a name on the network.
  Network::connect("/recserver/output/right_ve" , "/recclient/input/rec_rv");
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


    //SET POSITION LIKE THIS!
    motion_request.content().pix_xl = (int) 200.0*sin(((double)k)/100.0);
    motion_request.content().pix_xr = (int) 200.0*sin(((double)k)/100.0);
    motion_request.content().pix_y  = (int) 150.0*sin(((double)k)/80.0);
    outPort_mot.write(motion_request);
    



#if SHOW_Y_IMS || SHOW_MOSAICS
#endif
#if SHOW_Y_IMS_BRECT
#endif
#if SHOW_UV_IMS
#endif


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
      k++;
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

    if (SAVE) {

#if SHOW_Y_IMS
      // d_ly->save(rec_im_ly,"rec_im_ly_"+QString::number(save_num)+".jpg");
      d_ry->save(rec_im_ry,"r"+QString::number(save_num)+".jpg");
#endif
#if SHOW_Y_IMS_BRECT
      // d_lyb->save(rec_im_lyb,"rec_im_lyb_"+QString::number(save_num)+".jpg");
      d_ryb->save(rec_im_ryb,"b"+QString::number(save_num)+".jpg");
#endif

#if SHOW_UV_IMS
      d_lu->save(rec_im_lu,"rec_im_lu_"+QString::number(save_num)+".jpg");
      d_lv->save(rec_im_lv,"rec_im_lv_"+QString::number(save_num)+".jpg");
      d_ru->save(rec_im_ru,"rec_im_ru_"+QString::number(save_num)+".jpg");
      d_rv->save(rec_im_rv,"rec_im_rv_"+QString::number(save_num)+".jpg");
#endif
      save_num++;
    }


    if (inBot_ly==NULL && inBot_ry==NULL){
      usleep(1000);
      printf("No Input\n");
    }

  }

  //never here!
  
}
