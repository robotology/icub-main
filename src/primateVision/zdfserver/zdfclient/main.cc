/**
 * @ingroup icub_primatevision_zdfserver
 * \defgroup icub_primatevision_zdfserver_zdfclient ZDFClient
 *
 *This endclient module ZDFClient displays segmentation output from the ZDFServer, and initates motion commands to maintain track in the segmented target via the RecServer motion port. It also provides sample code showing how to make a useful ZDFServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b zdfclient \b\n
 *\n 
 * Ports accessed:\n
 * <ul>
 * <li> /zdfserver/output/seg
 * <li> /zdfserver/output/pzd
 * <li> /zdfserver/output/fovl
 * <li> /zdfserver/output/fovr
 * <li> /recserver/input/motion

 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /zdfclient/input/seg
 * <li> /zdfclient/input/pzd
 * <li> /zdfclient/input/fovl
 * <li> /zdfclient/input/fovr
 * <li> /recserver/input/motion
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /zdfclient/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/zdfserver/zdfclient/main.cc 
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
#include <zdfio.h>


using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  Port inPort_p;
  inPort_p.open("/zdfclient/input/serv_params");   
  Network::connect("/zdfclient/input/serv_params" , "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/zdfclient/input/serv_params");
  BinPortable<ZDFServerParams> response; 
  Bottle empty;
  inPort_p.write(empty,response);
  ZDFServerParams zsp = response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl; 

  int m_size = zsp.m_size;
  int m_psb  = zsp.m_psb;
  int t_size = zsp.t_size;
  int t_psb  = zsp.t_psb;

  IppiSize tsize={t_size,t_size};
  IppiSize msize={m_size,m_size};
  IppiSize osize={320,240};

  //BufferedPort<Bottle> inPort_res_mask;      // Create a port
  //inPort_res_mask.open("/zdfclient/input/res_mask");     // Give it a name on the network.
  //Network::connect("/zdfserver/output/res_mask" , "/zdfclient/input/res_mask");
  //Bottle *inBot_res_mask;
  //Ipp8u  *zdf_im_res_mask;

  BufferedPort<Bottle> inPort_res_prob;      // Create a port
  inPort_res_prob.open("/zdfclient/input/res_prob");     // Give it a name on the network.
  Network::connect("/zdfserver/output/res_prob" , "/zdfclient/input/res_prob");
  Bottle *inBot_res_prob;
  Ipp8u  *zdf_im_res_prob;

  BufferedPort<Bottle> inPort_fov_l;      // Create a port
  inPort_fov_l.open("/zdfclient/input/fov_l");     // Give it a name on the network.
  Network::connect("/zdfserver/output/fov_l" , "/zdfclient/input/fov_l");
  Bottle *inBot_fov_l;
  Ipp8u  *zdf_im_fov_l;

  //BufferedPort<Bottle> inPort_rec_l;      // Create a port
  //inPort_rec_l.open("/zdfclient/input/rec_l");     // Give it a name on the network.
  //Network::connect("/zdfserver/output/rec_l" , "/zdfclient/input/rec_l");
  //Bottle *inBot_rec_l;
  //Ipp8u  *zdf_im_rec_l;

  //BufferedPort<Bottle> inPort_rec_r;      // Create a port
  //inPort_rec_r.open("/zdfclient/input/rec_r");     // Give it a name on the network.
  //Network::connect("/zdfserver/output/rec_r" , "/zdfclient/input/rec_r");
  //Bottle *inBot_rec_r;
  //Ipp8u  *zdf_im_rec_r;

  BufferedPort<Bottle> inPort_fov_r;      // Create a port
  inPort_fov_r.open("/zdfclient/input/fov_r");     // Give it a name on the network.
  Network::connect("/zdfserver/output/fov_r" , "/zdfclient/input/fov_r");
  Bottle *inBot_fov_r;
  Ipp8u  *zdf_im_fov_r;

  //BufferedPort<Bottle> inPort_temp;
  //inPort_temp.open("/zdfclient/input/template");
  //Network::connect("/zdfserver/output/template" , "/zdfclient/input/template");
  //Bottle *inBot_temp;
  //Ipp8u  *zdf_im_temp;

  BufferedPort<Bottle> inPort_seg_im;      // Create a port
  inPort_seg_im.open("/zdfclient/input/seg_im");     // Give it a name on the network.
  Network::connect("/zdfserver/output/seg_im" , "/zdfclient/input/seg_im");
  Bottle *inBot_seg_im;
  Ipp8u  *zdf_im_seg_im;

  //BufferedPort<Bottle> inPort_seg_dog;      // Create a port
  //inPort_seg_dog.open("/zdfclient/input/seg_dog");     // Give it a name on the network.
  //Network::connect("/zdfserver/output/seg_dog" , "/zdfclient/input/seg_dog");
  //Bottle *inBot_seg_dog;
  //Ipp8u  *zdf_im_seg_dog;


  //LAUNCH GUI WINDOWS:
  //iCub::contrib::primateVision::Display *d_res_mask = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_RES_MASK");
  iCub::contrib::primateVision::Display *d_fov_l    = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_FOV_L");
  //iCub::contrib::primateVision::Display *d_rec_l    = new iCub::contrib::primateVision::Display(osize,320,D_8U,"ZDF_REC_L");
  //iCub::contrib::primateVision::Display *d_rec_r    = new iCub::contrib::primateVision::Display(osize,320,D_8U,"ZDF_REC_R");
  iCub::contrib::primateVision::Display *d_fov_r    = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_FOV_R");
  iCub::contrib::primateVision::Display *d_res_prob = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_RES_PROB");
  iCub::contrib::primateVision::Display *d_seg_im   = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_SEG_IM");
  //iCub::contrib::primateVision::Display *d_seg_dog  = new iCub::contrib::primateVision::Display(msize,m_psb,D_8U,"ZDF_SEG_DOG");
  //iCub::contrib::primateVision::Display *d_temp     = new iCub::contrib::primateVision::Display(tsize,t_psb,D_8U_NN,"ZDF_TEMPLATE");
 


  printf("begin..\n");
  
  int k=0;

  while (1){

    k++;

    //IMS:
    inBot_res_prob = inPort_res_prob.read(false);
    inBot_fov_l = inPort_fov_l.read(false);
    //inBot_rec_l = inPort_rec_l.read(false);
    //inBot_rec_r = inPort_rec_r.read(false);
    inBot_fov_r = inPort_fov_r.read(false);
    inBot_seg_im = inPort_seg_im.read();
    //inBot_seg_dog = inPort_seg_dog.read(false);
    //inBot_res_mask = inPort_res_mask.read(false);
    //inBot_temp  = inPort_temp.read();


    //if (inBot_res_mask!=NULL){
    //  zdf_im_res_mask = (Ipp8u*) inBot_res_mask->get(0).asBlob();
    //  d_res_mask->display(zdf_im_res_mask);
    // }
    if (inBot_res_prob!=NULL){
      zdf_im_res_prob = (Ipp8u*) inBot_res_prob->get(0).asBlob();
      d_res_prob->display(zdf_im_res_prob);
    }
    if (inBot_fov_l!=NULL){
      zdf_im_fov_l = (Ipp8u*) inBot_fov_l->get(0).asBlob();
      d_fov_l->display(zdf_im_fov_l);
    }
    if (inBot_fov_r!=NULL){
      zdf_im_fov_r = (Ipp8u*) inBot_fov_r->get(0).asBlob();
      d_fov_r->display(zdf_im_fov_r);
    }

    //if (inBot_rec_l!=NULL){
    //  zdf_im_rec_l = (Ipp8u*) inBot_rec_l->get(0).asBlob();
    //  d_rec_l->display(zdf_im_rec_l);
    //  //d_rec_l->save(zdf_im_rec_l,"rec_l"+QString::number(k)+".jpg");
    // }
    //if (inBot_rec_r!=NULL){
    //  zdf_im_rec_r = (Ipp8u*) inBot_rec_r->get(0).asBlob();
    //  d_rec_r->display(zdf_im_rec_r);
    //  //d_rec_r->save(zdf_im_rec_r,"rec_r"+QString::number(k)+".jpg");
    // }

    if (inBot_seg_im!=NULL){
      zdf_im_seg_im = (Ipp8u*) inBot_seg_im->get(0).asBlob();
      d_seg_im->display(zdf_im_seg_im);
      //d_seg_im->save(zdf_im_seg_im,"seg_im"+QString::number(k)+".jpg");
    }
    //if (inBot_seg_dog!=NULL){
    //  zdf_im_seg_dog = (Ipp8u*) inBot_seg_dog->get(0).asBlob();
    //  d_seg_dog->display(zdf_im_seg_dog);
    //  //d_seg_dog->save(zdf_im_seg_dog,"seg_dog"+QString::number(k)+".jpg");
    // }
    //if (inBot_temp!=NULL){
    //  zdf_im_temp = (Ipp8u*) inBot_temp->get(0).asBlob();
    //  d_temp->display(zdf_im_temp);
    //}
    
    if (//inBot_res_mask==NULL &&
	inBot_res_prob==NULL &&
	inBot_fov_l==NULL &&
	//inBot_rec_l==NULL &&
	//inBot_rec_r==NULL &&
	inBot_seg_im==NULL &&
	//inBot_seg_dog==NULL &&
	//inBot_temp==NULL &&
	inBot_fov_r==NULL ){
      printf("No Input\n");
      usleep(5000);// don't blow out port
    }
    
  }

  //never here!
  
}
