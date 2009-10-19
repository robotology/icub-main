/**
 * @ingroup icub_applications_primatevision
 *
 * \defgroup icub_applications_primatevision_pointing PrimateVision_Pointing
 *
 * \brief AttnServer provides attentional saccade shifts. ZDFServer provides coordinated fixation. iKinHand provides pointing arm motion.  This demo module connects all these running servers together appropriately and begins the demo.
 *
 * For running the application:
 *  
 *  edit...
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at src/primateVision/demos/pointing/main.cc
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
#include <recio.h>


using namespace std;
using namespace iCub::contrib::primateVision;

int main( int argc, char **argv )
{

  QApplication a(argc, argv);

  Port inPort_p;
  inPort_p.open("/pointing/input/serv_params");   
  Network::connect("/pointing/input/serv_params" , "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/pointing/input/serv_params");
  BinPortable<ZDFServerParams> response; 
  Bottle empty;
  inPort_p.write(empty,response);
  ZDFServerParams zsp = response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl; 


  int match_width,match_height,match_psb;
  match_width  = zsp.width;
  match_height = zsp.height;
  match_psb    = zsp.psb;

  IppiSize msize;
  msize.width  = match_width;
  msize.height = match_height;


  BufferedPort<Bottle> inPort_res_mask; 
  inPort_res_mask.open("/pointing/input/res_mask");
  Network::connect("/zdfserver/output/res_mask" , "/pointing/input/res_mask");
  Bottle *inBot_res_mask;
  Ipp8u  *zdf_im_res_mask;



  BufferedPort<Vector> inPort_fp;
  inPort_fp.open("/pointing/input/arm_fpi"); 
  Network::connect("/eyeTriangulation/X:o","/pointing/input/arm_fpi");
  Vector* fpi;

  BufferedPort<Vector> outPort_fp;
  outPort_fp.open("/pointing/input/arm_fpo"); 
  Network::connect("/pointing/input/arm_fpo","iKinArmCtrl/right_arm/xd:i");






  //LAUNCH GUI WINDOW:
  iCub::contrib::primateVision::Display *d_res_mask = new iCub::contrib::primateVision::Display(msize,match_psb,D_8U,"ZDF_RES_MASK");


  int zdf_area = 0;
  int zdf_track = 0;
  bool sent_rest;



  printf("begin..\n");
  
  while (1){

       


    //get zdf state:
    inBot_res_mask = inPort_res_mask.read();


    if (inBot_res_mask!=NULL){
      zdf_im_res_mask = (Ipp8u*) inBot_res_mask->get(0).asBlob();
      zdf_area  = inBot_res_mask->get(1).asInt();
      zdf_track = inBot_res_mask->get(2).asInt();
 


      if (zdf_track==1){
	printf("Track:%d\n",zdf_track);

	//SEND HAND MOTION COMMAND:
	fpi  = inPort_fp.read();
        Vector &fpo=outPort_fp.prepare();
	fpo.resize(7);
	fpo[0]=(*fpi)[0];
	fpo[1]=(*fpi)[1];
	fpo[2]=(*fpi)[2];
	fpo[3]= 0.123;
	fpo[4]=-0.535;
	fpo[5]= 0.835;
	fpo[6]= 2.917;
	sent_rest = false;
	outPort_fp.write();

      }
      else{
	printf("Suspend:%d\n",zdf_track);
	//RETURN TO REST POSITION:
	if (!sent_rest){
	  Vector &fpo=outPort_fp.prepare();
	  fpo.resize(7);
	  fpo[0]=-0.236;
	  fpo[1]= 0.121;
	  fpo[2]=-0.015;
	  fpo[3]= 0.115;
	  fpo[4]= 0.417;
	  fpo[5]= -0.90;
	  fpo[6]= 2.845;

	  outPort_fp.write();
	  sent_rest = true;
	}	
      }
    


      //display mask
      d_res_mask->display(zdf_im_res_mask); 
      

    }


    if (inBot_res_mask==NULL ){
      printf("No Input\n");
      usleep(10000);// don't blow out port
    }
    
  }

  //never here!
  
}
