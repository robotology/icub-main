/**
 * @ingroup icub_primatevision_nzdfserver
 * \defgroup icub_primatevision_nzdfserver_nzdfclient NZDFClient
 *
 *This endclient module NZDFClient displays segmentation output from the NZDFServer, and initates motion commands to maintain track in the segmented target via the RecServer motion port. It also provides sample code showing how to make a useful NZDFServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b nzdfclient \b\n
 *\n 
 * Ports accessed:\n
 * <ul>
 * <li> /nzdfserver/output/seg
 * <li> /nzdfserver/output/pnzd
 * <li> /nzdfserver/output/fovl
 * <li> /nzdfserver/output/fovr
 * <li> /recserver/input/motion

 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /nzdfclient/input/seg
 * <li> /nzdfclient/input/pnzd
 * <li> /nzdfclient/input/fovl
 * <li> /nzdfclient/input/fovr
 * <li> /recserver/input/motion
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /nzdfclient/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/nzdfserver/nzdfclient/main.cc 
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
#include <nzdfio.h>


using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  Port inPort_p;
  inPort_p.open("/nzdfclient/input/serv_params");   
  Network::connect("/nzdfclient/input/serv_params" , "/nzdfserver/output/serv_params");
  Network::connect("/nzdfserver/output/serv_params", "/nzdfclient/input/serv_params");
  BinPortable<NZDFServerParams> response; 
  Bottle empty;
  inPort_p.write(empty,response);
  NZDFServerParams zsp = response.content();
  std::cout << "NZDFServer Probe Response: " << zsp.toString() << std::endl; 

  int m_size = zsp.m_size;
  int m_psb  = zsp.m_psb;
  int t_size = zsp.t_size;
  int t_psb  = zsp.t_psb;

  IppiSize tsize={t_size,t_size};
  IppiSize msize={m_size,m_size};


  BufferedPort<Bottle> inPort_res_mask;      // Create a port
  inPort_res_mask.open("/nzdfclient/input/res_mask");     // Give it a name on the network.
  Network::connect("/nzdfserver/output/res_mask" , "/nzdfclient/input/res_mask");
  Bottle *inBot_res_mask;
  Ipp8u  *zdf_im_res_mask;

  BufferedPort<Bottle> inPort_fov_l;
  inPort_fov_l.open("/nzdfclient/input/fov_l");
  Network::connect("/nzdfserver/output/fov_l" , "/nzdfclient/input/fov_l");
  Bottle *inBot_fov_l;
  Ipp8u  *zdf_im_fov_l;

  BufferedPort<Bottle> inPort_fov_r;
  inPort_fov_r.open("/nzdfclient/input/fov_r");
  Network::connect("/nzdfserver/output/fov_r" , "/nzdfclient/input/fov_r");
  Bottle *inBot_fov_r;
  Ipp8u  *zdf_im_fov_r;

  BufferedPort<Bottle> inPort_fov_d;
  inPort_fov_d.open("/nzdfclient/input/fov_d");
  Network::connect("/nzdfserver/output/fov_d" , "/nzdfclient/input/fov_d");
  Bottle *inBot_fov_d;
  Ipp8u  *zdf_im_fov_d;

  BufferedPort<Bottle> inPort_temp;
  inPort_temp.open("/nzdfclient/input/template");
  Network::connect("/nzdfserver/output/template" , "/nzdfclient/input/template");
  Bottle *inBot_temp;
  Ipp8u  *zdf_im_temp;



  //LAUNCH GUI WINDOWS:
  iCub::contrib::primateVision::Display *d_res_mask = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"NZDF_RES_MASK");
  iCub::contrib::primateVision::Display *d_fov_l    = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U_NN,"NZDF_FOV_L");
  iCub::contrib::primateVision::Display *d_fov_r    = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U_NN,"NZDF_FOV_R");
  iCub::contrib::primateVision::Display *d_fov_d    = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"NZDF_FOV_D");
  iCub::contrib::primateVision::Display *d_temp     = new iCub::contrib::primateVision::Display(tsize,t_psb,D_8U_NN,"NZDF_TEMPLATE");

  printf("Begin..\n");
  

  int k=0;
  while (1){

    //IMS:
    inBot_fov_l    = inPort_fov_l.read(false);
    inBot_fov_r    = inPort_fov_r.read(false);
    inBot_fov_d    = inPort_fov_d.read(false);
    inBot_temp     = inPort_temp.read(false);
    inBot_res_mask = inPort_res_mask.read();//false




    k++;

    if (inBot_res_mask!=NULL){
      zdf_im_res_mask = (Ipp8u*) inBot_res_mask->get(0).asBlob();
      d_res_mask->display(zdf_im_res_mask);
    }
    if (inBot_fov_l!=NULL){
      zdf_im_fov_l = (Ipp8u*) inBot_fov_l->get(0).asBlob();
      d_fov_l->display(zdf_im_fov_l);
      //d_fov_l->save(zdf_im_fov_l,"iml"+QString::number(k)+".jpg");
    }
    if (inBot_fov_r!=NULL){
      zdf_im_fov_r = (Ipp8u*) inBot_fov_r->get(0).asBlob();
      d_fov_r->display(zdf_im_fov_r);
      //d_fov_r->save(zdf_im_fov_r,"imr"+QString::number(k)+".jpg");
    }
    if (inBot_fov_d!=NULL){
      zdf_im_fov_d = (Ipp8u*) inBot_fov_d->get(0).asBlob();
      d_fov_d->display(zdf_im_fov_d);
    }
    if (inBot_temp!=NULL){
      zdf_im_temp = (Ipp8u*) inBot_temp->get(0).asBlob();
      d_temp->display(zdf_im_temp);
    }
    
    if (inBot_res_mask==NULL &&
	inBot_fov_l==NULL &&
	inBot_fov_d==NULL &&
	inBot_temp==NULL &&
	inBot_fov_r==NULL ){
      printf("No Input\n");
      usleep(5000);// don't blow out port
    }
    
  }

  //never here!
  
}
