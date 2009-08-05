/**
 * @ingroup icub_primatevision_utils_flow
 * \defgroup icub_primatevision_utils_flow_testflowlive TestFlowLive
 *
 * A test application for the Flow class that operates in live images obtained from a RecServer.
 *
 * This module should be called in the following way:\n \b testflow \b 
 *\n
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/utils/flow/testflowlive/main.cc 
 * 
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <iostream>
#include <qapplication.h>
#include <ipp.h>

//my includes
#include <display.h>
#include <mosaic.h>
#include <flow.h>  
#include <dog.h>  
// THIS APP IS A RECCLIENT!!!
#include <recio.h>

using namespace std;
using namespace iCub::contrib::primateVision;

int main(int argc, char *argv[]) 
{

  QApplication *qa = new QApplication( argc, argv );
 


  int flow_scale = 2; //1,2,3,4
  int bland_dog_thresh = 1; //0..255


  Port inPort_s;
  inPort_s.open("/testflowlive/input/serv_params");     // Give it a name on the network.
  Network::connect("/testflowlive/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/testflowlive/input/serv_params");
  BinPortable<RecServerParams> response; 
  Bottle empty;
  inPort_s.write(empty,response);
  RecServerParams rsp = response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl; 

  Port inPort_p;
  inPort_p.open("/testflowlive/input/rec_params");     // Give it a name on the network.
  Network::connect("/recserver/output/rec_params", "/testflowlive/input/rec_params");
  BinPortable<RecResultParams> rec_res; 
  int psb;
  IppiSize srcsize;
  srcsize.width = rsp.width;
  srcsize.height = rsp.height;
  psb = rsp.psb;

  BufferedPort<Bottle> inPort_ly;      // Create a port
  inPort_ly.open("/testflowlive/input/rec_ly");     // Give it a name on the network.
  Network::connect("/recserver/output/left_ye" , "/testflowlive/input/rec_ly");
  Bottle *inBot_ly;
  Ipp8u* rec_im_ly;


  Port outPort_mot;
  outPort_mot.open("/testflowlive/output/mot");     // Give it a name on the network.
  Network::connect("/testflowlive/output/mot", "/recserver/input/motion");
  Network::connect("/recserver/input/motion", "/testflowlive/output/mot");
  BinPortable<RecMotionRequest> motion_request;



  //PROCESSING CLASS SETUP:
  Flow*fl  = new Flow(srcsize,flow_scale);
  DoG*dl = new DoG(srcsize);


  iCub::contrib::primateVision::Display *disp_flx   = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"FLX");
  iCub::contrib::primateVision::Display *disp_fly   = new iCub::contrib::primateVision::Display(srcsize,psb,D_8U_NN,"FLY");


  IppiSize mossize = {rsp.mos_width,rsp.mos_height};
  Mosaic *ml = new Mosaic(mossize,srcsize,psb,D_8U_NN,"FLOW TEST L");


  //SET START POSITION:
  motion_request.content().pix_xl = 0;
  motion_request.content().pix_xr = 0;
  motion_request.content().pix_y  = 0;
  motion_request.content().deg_r  = 0.0;
  motion_request.content().deg_p  = 0.0;
  motion_request.content().deg_y  = 0.0;
  motion_request.content().relative  = false;
  outPort_mot.write(motion_request);






  //MAIN EVENT LOOP:
  int k=0;
  while (1){


    k++;

    //SET POSITION LIKE THIS!
    motion_request.content().pix_xl = (int) 200.0*sin(((double)k)/30.0);
    motion_request.content().pix_xr = (int) 200.0*sin(((double)k)/30.0);
    motion_request.content().pix_y = (int) 150.0*sin(((double)k)/50.0);
    outPort_mot.write(motion_request);
 

    //get geometry:
    inPort_p.read(rec_res); //blocking buffered
    //std::cout << "Rec Result: " << rec_res.content().toString() << std::endl;     

    //get image:
    inBot_ly = inPort_ly.read();
    Value& val_ly = inBot_ly->get(0);
    rec_im_ly = (Ipp8u*) val_ly.asBlob();


 
    //flow is done on dog:
    dl->proc(rec_im_ly,psb);
    //lx,ly hold mosaic coords of window.
    //get flow:
    fl->proc(dl->get_dog_onoff(),psb,rec_res.content().lx,rec_res.content().ly);
    //fx,fy appear:
    //value:   0  1  2  3  4  5  6  7 
    //meaning: N -3 -2 -1  0  1  2  3 pixels


    //remove flow estimation in bland, noisy regions:
    for (int x=0;x<srcsize.width;x++){
      for (int y=0;y<srcsize.height;y++){ 
	if (dl->get_dog_onoff()[x + y*dl->get_psb()] < bland_dog_thresh ){ //dst
	  fl->get_fx()[x + y*fl->get_psb()] = 0;
	  fl->get_fy()[x + y*fl->get_psb()] = 0;
	}
      }
    }
	  
	  



    //prepare results for nice non-normed display:

    //threshold out any 0s! (set N to 4)
    ippiThreshold_LTVal_8u_C1IR(fl->get_fx(),psb,srcsize,(Ipp8u)1,(Ipp8u)4);
    ippiThreshold_LTVal_8u_C1IR(fl->get_fy(),psb,srcsize,(Ipp8u)1,(Ipp8u)4);
    
    //expand to 0-240:
    //sub 1:
    ippiSubC_8u_C1IRSfs(1,fl->get_fx(),psb,srcsize,0);
    ippiSubC_8u_C1IRSfs(1,fl->get_fy(),psb,srcsize,0);
    //   //NOW FLOW IS:
    //   //max left:  0 (-3 pix)
    //   //no motion: 3
    //   //max right: 6 (+3 pix)
    
    //mul by 40:
    ippiMulC_8u_C1IRSfs(40,fl->get_fx(),psb,srcsize,0);
    ippiMulC_8u_C1IRSfs(40,fl->get_fy(),psb,srcsize,0);
    //   //NOW FLOW IS:
    //   //max left:  0
    //   //no motion: 120
    //   //max right: 240
    

    //DISPLAY:
    disp_flx->display(fl->get_fx());
    disp_fly->display(fl->get_fy());
    ml->display(rec_im_ly,rec_res.content().lx,rec_res.content().ly);

  }

}

