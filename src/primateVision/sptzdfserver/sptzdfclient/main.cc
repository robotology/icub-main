/**
 * @ingroup icub_primatevision_sptzdfserver
 * \defgroup icub_primatevision_sptzdfserver_sptzdfclient SpTZDFClient
 *
 *This endclient module SpTZDFClient displays segmentation output from the SpTZDFServer, and initates motion commands to maintain track in the segmented target via the RecServer motion port. It also provides sample code showing how to make a useful SpTZDFServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b sptzdfclient \b\n
 *\n 
 * Ports accessed:\n
 * <ul>
 * <li> /sptzdfserver/output/seg
 * <li> /sptzdfserver/output/pzd
 * <li> /sptzdfserver/output/fovl
 * <li> /sptzdfserver/output/fovr
 * <li> /recserver/input/motion

 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /sptzdfclient/input/seg
 * <li> /sptzdfclient/input/pzd
 * <li> /sptzdfclient/input/fovl
 * <li> /sptzdfclient/input/fovr
 * <li> /recserver/input/motion
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /sptzdfclient/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/sptzdfserver/sptzdfclient/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>

//MY INCLUDES
#include <display.h>
#include <sptzdfio.h>


using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  Port inPort_p;
  inPort_p.open("/sptzdfclient/input/serv_params");   
  Network::connect("/sptzdfclient/input/serv_params" , "/sptzdfserver/output/serv_params");
  Network::connect("/sptzdfserver/output/serv_params", "/sptzdfclient/input/serv_params");
  BinPortable<SpTZDFServerParams> response; 
  Bottle empty;
  inPort_p.write(empty,response);
  SpTZDFServerParams zsp = response.content();
  std::cout << "SpTZDFServer Probe Response: " << zsp.toString() << std::endl; 

  int m_size = zsp.m_size;
  int m_psb  = zsp.m_psb;
  int t_size = zsp.t_size;
  int t_psb  = zsp.t_psb;

  IppiSize tsize={t_size,t_size};
  IppiSize msize={m_size,m_size};


  BufferedPort<Bottle> inPort_res_mask;
  inPort_res_mask.open("/sptzdfclient/input/res_mask");
  Network::connect("/sptzdfserver/output/res_mask" , "/sptzdfclient/input/res_mask");
  Bottle *inBot_res_mask;
  Ipp8u  *zdf_im_res_mask;

  BufferedPort<Bottle> inPort_res_prob;
  inPort_res_prob.open("/sptzdfclient/input/res_prob");
  Network::connect("/sptzdfserver/output/res_prob" , "/sptzdfclient/input/res_prob");
  Bottle *inBot_res_prob;
  Ipp8u  *zdf_im_res_prob;

  BufferedPort<Bottle> inPort_fov_l;
  inPort_fov_l.open("/sptzdfclient/input/fov_l");
  Network::connect("/sptzdfserver/output/fov_l" , "/sptzdfclient/input/fov_l");
  Bottle *inBot_fov_l;
  Ipp8u  *zdf_im_fov_l;

  BufferedPort<Bottle> inPort_fov_r;
  inPort_fov_r.open("/sptzdfclient/input/fov_r");
  Network::connect("/sptzdfserver/output/fov_r" , "/sptzdfclient/input/fov_r");
  Bottle *inBot_fov_r;
  Ipp8u  *zdf_im_fov_r;

  BufferedPort<Bottle> inPort_edge_im;
  inPort_edge_im.open("/sptzdfclient/input/edge_im");
  Network::connect("/sptzdfserver/output/edge_im" , "/sptzdfclient/input/edge_im");
  Bottle *inBot_edge_im;
  Ipp8u  *zdf_edge_im;

  BufferedPort<Bottle> inPort_temp;
  inPort_temp.open("/sptzdfclient/input/template");
  Network::connect("/sptzdfserver/output/template" , "/sptzdfclient/input/template");
  Bottle *inBot_temp;
  Ipp8u  *zdf_im_temp;

  BufferedPort<Bottle> inPort_seg;
  inPort_seg.open("/sptzdfclient/input/seg");
  Network::connect("/sptzdfserver/output/seg" , "/sptzdfclient/input/seg");
  Bottle *inBot_seg;
  Ipp8u  *zdf_im_seg;



  //LAUNCH GUI WINDOWS:
  iCub::contrib::primateVision::Display *d_res_mask = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"SPTZDF_RES_MASK");
  iCub::contrib::primateVision::Display *d_res_prob = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U_NN,"SPTZDF_RES_PROB");
  //iCub::contrib::primateVision::Display *d_fov_l    = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U_NN,"SPTZDF_FOV_L");
  //iCub::contrib::primateVision::Display *d_fov_r    = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U_NN,"SPTZDF_FOV_R");
  iCub::contrib::primateVision::Display *d_edge_im  = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U_NN,"SPTZDF_EDGE_IM");
  //iCub::contrib::primateVision::Display *d_temp     = new iCub::contrib::primateVision::Display(tsize,t_psb,D_8U_NN,"SPTZDF_TEMPLATE");
  iCub::contrib::primateVision::Display *d_seg      = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"SPTZDF_SEG");

  printf("Begin..\n");
  
  while (1){

    //IMS:
    inBot_res_prob = inPort_res_prob.read(false);
    //inBot_fov_l    = inPort_fov_l.read(false);
    //inBot_fov_r    = inPort_fov_r.read(false);
    inBot_edge_im  = inPort_edge_im.read(false);
    //inBot_temp     = inPort_temp.read(false);
    inBot_seg      = inPort_seg.read(false);
    inBot_res_mask = inPort_res_mask.read();//false


    if (inBot_res_mask!=NULL){
      zdf_im_res_mask = (Ipp8u*) inBot_res_mask->get(0).asBlob();
      d_res_mask->display(zdf_im_res_mask);
    }
    if (inBot_res_prob!=NULL){
      zdf_im_res_prob = (Ipp8u*) inBot_res_prob->get(0).asBlob();
      d_res_prob->display(zdf_im_res_prob);
    }
    //if (inBot_fov_l!=NULL){
    //  zdf_im_fov_l = (Ipp8u*) inBot_fov_l->get(0).asBlob();
    //  d_fov_l->display(zdf_im_fov_l);
    //}
    //if (inBot_fov_r!=NULL){
    //  zdf_im_fov_r = (Ipp8u*) inBot_fov_r->get(0).asBlob();
    //  d_fov_r->display(zdf_im_fov_r);
    //}
    if (inBot_edge_im!=NULL){
      zdf_edge_im = (Ipp8u*) inBot_edge_im->get(0).asBlob();
      d_edge_im->display(zdf_edge_im);
    }
    //if (inBot_temp!=NULL){
    //  zdf_im_temp = (Ipp8u*) inBot_temp->get(0).asBlob();
    //  d_temp->display(zdf_im_temp);
    //}
    if (inBot_seg!=NULL){
      zdf_im_seg = (Ipp8u*) inBot_seg->get(0).asBlob();
      d_seg->display(zdf_im_seg);
    }
    
    if (inBot_res_mask==NULL &&
	inBot_res_prob==NULL &&
	//inBot_fov_l==NULL &&
	inBot_edge_im==NULL &&
	//inBot_temp==NULL &&
	inBot_seg==NULL //&&
	//inBot_fov_r==NULL 
	){
      printf("No Input\n");
      usleep(5000);// don't blow out port
    }
    
  }

  //never here!
  
}
