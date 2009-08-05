/**
 * @ingroup icub_primatevision_attentionserver_stereo
 * \defgroup icub_primatevision_attentionserver_stereo_attentionclient_stereo AttnClient_Stereo
 *
 *This endclient module AttnClient_Stereo displays output from the AttnServer_Stereo and initiates attentional saccades to the desired peaks via motion requests sent to the RecServer. It also provides sample code showing how to make a useful AttnServer client module. Useful for display and testing.
 *
 * This module should be called in the following way:\n \b attnclient_stereo \b\n
 *\n
 * Ports accessed:\n
 * <ul>
 * <li> /attnserverS/output/params
 * <li> /attnserverS/output/attnL
 * <li> /attnserverS/output/attnR
 * <li> /recserver/input/motion
 * </ul>
 
 * Input ports:\n
 * <ul>
 * <li> /attnclientS/input/params
 * <li> /attnclientS/input/attnL
 * <li> /attnclientS/input/attnR
 * </ul>
 *
 * Output ports:\n
 * <ul>
 * <li> /attnclientS/output/motion
 * </ul>
 * \n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/attentionserver_stereo/attentionclient_stereo/main.cc 
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
#include <mydraw.h>
//client of:
#include <attnio_st.h>

#define BLACK 0.01
#define GREY  0.3
#define WHITE 0.99


using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

 
  Port inPort_s;
  inPort_s.open("/attnclient_st/input/serv_params");     // Give it a name on the network.
  Network::connect("/attnclient_st/input/serv_params", "/attnserver_st/output/serv_params");
  Network::connect("/attnserver_st/output/serv_params", "/attnclient_st/input/serv_params");
  BinPortable<AttnStServerParams> server_response_s; 
  Bottle empty;
  inPort_s.write(empty,server_response_s);
  AttnStServerParams ssp_s = server_response_s.content();
  std::cout << "AttnServer ST Probe Response: " << ssp_s.toString() << std::endl;

  IppiSize srcsize;
  srcsize.width = ssp_s.width;
  srcsize.height = ssp_s.height;
  int psb = ssp_s.psb;


  BufferedPort<Bottle> inPort_l;      // Create a ports
  inPort_l.open("/attnclient_st/input/attn_l");     // Give it a name on the network.
  Network::connect("/attnserver_st/output/attn_l" , "/attnclient_st/input/attn_l");
  Bottle *inBot_l;
  Ipp8u* al;

  BufferedPort<Bottle> inPort_r;      // Create a ports
  inPort_r.open("/attnclient_st/input/attn_r");     // Give it a name on the network.
  Network::connect("/attnserver_st/output/attn_r" , "/attnclient_st/input/attn_r");
  Bottle *inBot_r;
  Ipp8u* ar;





  iCub::contrib::primateVision::Display *d_l = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"ATTN_ST L");
  iCub::contrib::primateVision::Display *d_r = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"ATTN_ST R");




  int fx_l,fx_r,fy_l,fy_r;
  int tx_l,tx_r,ty_l,ty_r;
  int win_l,win_r;

  while (1){
    

    inBot_r = inPort_r.read(false);
    inBot_l = inPort_l.read();


    if (inBot_l!=NULL){ 
      al    = (Ipp8u*) inBot_l->get(0).asBlob();
      fx_l  = (int) inBot_l->get(1).asInt();
      fy_l  = (int) inBot_l->get(2).asInt();
      tx_l  = (int) inBot_l->get(3).asInt();
      ty_l  = (int) inBot_l->get(4).asInt();
      win_l = (int) inBot_l->get(5).asInt();

      MyDrawLine(al,psb,srcsize,srcsize.width/2,srcsize.height/2,BLACK);
      
      if (win_l == ALEFT){
	MyDrawLine(al,psb,srcsize,tx_l,ty_l,WHITE);
      }
      else if (win_l == ARIGHT){
	MyDrawLine(al,psb,srcsize,tx_l,ty_l,WHITE);
	MyDrawLine(al,psb,srcsize,fx_l,fy_l,GREY);
      }
      else{
	MyDrawLine(al,psb,srcsize,fx_l,fy_l,GREY);
      }
      

      d_l->display(al);


    }
    

    if (inBot_r!=NULL){      
      ar    = (Ipp8u*) inBot_r->get(0).asBlob();
      fx_r  = (int) inBot_r->get(1).asInt();
      fy_r  = (int) inBot_r->get(2).asInt();
      tx_r  = (int) inBot_r->get(3).asInt();
      ty_r  = (int) inBot_r->get(4).asInt();
      win_r = (int) inBot_r->get(5).asInt();      

      MyDrawLine(ar,psb,srcsize,srcsize.width/2,srcsize.height/2,BLACK);

      if (win_r == ALEFT){
	MyDrawLine(ar,psb,srcsize,tx_r,ty_r,WHITE);
	MyDrawLine(ar,psb,srcsize,fx_r,fy_r,GREY);
      }	
      else if (win_r == ARIGHT){
	MyDrawLine(ar,psb,srcsize,tx_r,ty_r,WHITE);
      }
      else {
	MyDrawLine(ar,psb,srcsize,fx_r,fy_r,WHITE);
      }
      

      d_r->display(ar);

      
    }
    
  }
  
  //never here! 
  
}
