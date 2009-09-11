/**
 * Client of attention_server and yarp_images and rec_motion.  
 * wait a second after attentional saccade
 * for zdf to settle. Then Lock motion, grab full res image and run Ajay's code...  
 */ 


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
//MY INCLUDES
#include <display.h>
#include <convert_rgb.h>
//client of:
#include <attnio_st.h>
#include <recio.h>

#define BLACK 0.01
#define GREY  0.3
#define WHITE 0.99


using namespace iCub::contrib::primateVision;


int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

 

  Bottle empty;


  Port inPort_as;
  inPort_as.open("/mary/input/serv_params_a"); 
  Network::connect("/mary/input/serv_params_a", "/attnserver_st/output/serv_params");
  Network::connect("/attnserver_st/output/serv_params", "/mary/input/serv_params_a");
  BinPortable<AttnStServerParams> server_response_a; 
  inPort_as.write(empty,server_response_a);
  AttnStServerParams ssp_a = server_response_a.content();
  std::cout << "AttnServer ST Probe Response: " << ssp_a.toString() << std::endl;
  IppiSize srcsize;
  srcsize.width = ssp_a.width;
  srcsize.height = ssp_a.height;
  int psb = ssp_a.psb;

  BufferedPort<Bottle> inPort_l; 
  inPort_l.open("/mary/input/attn_l");
  Network::connect("/attnserver_st/output/attn_l" , "/mary/input/attn_l");
  Bottle *inBot_l;
  Ipp8u* al;
  BufferedPort<Bottle> inPort_r; 
  inPort_r.open("/mary/input/attn_r");
  Network::connect("/attnserver_st/output/attn_r" , "/mary/input/attn_r");
  Bottle *inBot_r;
  Ipp8u* ar;




  //Direct Original Image Input:
  BufferedPort<ImageOf<PixelBgr> > pcl,pcr;
  pcl.open("/mary/input/image/left");
  pcr.open("/mary/input/image/right");
  Network::connect("/icub/cam/left" , "/mary/input/image/left");
  Network::connect("/icub/cam/right" ,"/mary/input/image/right");
  ImageOf<PixelBgr> *imgl;


  //Recserver access for motion lock:
  Port inPort_rs;
  inPort_rs.open("/mary/input/serv_params_r"); 
  Network::connect("/mary/input/serv_params_r","/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params","/mary/input/serv_params_r");
  BinPortable<RecServerParams> server_response_r; 
  inPort_rs.write(empty,server_response_r);
  RecServerParams rsp = server_response_r.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;


  //ACCESS MOTION
  Port outPort_mot;
  outPort_mot.open("/mary/output/mot");
  Network::connect("/mary/output/mot", "/recserver/input/motion");
  Network::connect("/recserver/input/motion", "/mary/output/mot");
  BinPortable<RecMotionRequest> motion_request;
  //initalise static motion request params:
  motion_request.content().pix_y  = 0;
  motion_request.content().pix_xl = 0;
  motion_request.content().pix_xr = 0;
  motion_request.content().deg_r  = 0.0;
  motion_request.content().deg_p  = 0.0;
  motion_request.content().deg_y  = 0.0;
  motion_request.content().relative = true; 
  motion_request.content().suspend = 0; 
  motion_request.content().lockto = MARY_LOCK; 
  motion_request.content().unlock = true; 



  int sac_num=0;
  int old_sac_num=0;
  bool first = true;






  //Get first RGB image to establish width, height:
  IppiSize insize;
  imgl = pcl.read(); //blocking buffered
  insize.width = imgl->width();
  insize.height = imgl->height();
  printf("Mary: Received input image dimensions: (%d,%d)\n",insize.width, insize.height);

  int psb4;
  Ipp8u *colour = ippiMalloc_8u_C4(insize.width,insize.height,&psb4);
  

  Convert_RGB *c_rgb2yuv = new Convert_RGB(insize);
  iCub::contrib::primateVision::Display *d_im = new iCub::contrib::primateVision::Display(insize,c_rgb2yuv->get_psb(),D_8U,"ORIG GREY");






  int num=0;

  while (1){
    

    inBot_r = inPort_r.read(false);
    inBot_l = inPort_l.read();


    if (inBot_l!=NULL){ 
      sac_num = (int) inBot_l->get(6).asInt();
      if (first){old_sac_num=sac_num;first=false;} //prevent proc on run.
    }
    

    if (inBot_r!=NULL){      
      sac_num = (int) inBot_r->get(6).asInt();    
      if (first){old_sac_num=sac_num;first=false;} //prevent proc on run.
    }






    //PROC AFTER SACCADE:
    if (sac_num != old_sac_num){
      //we've just done a saccade.
      old_sac_num = sac_num;

      //sleep to allow saccade susspend to expire and then zdf to settle on target:
      sleep(4);


      //prevent motion do Ajay's processing:
      printf("SACCADE %d. LOCKING MOTION.\n",sac_num);
      //lock motion:
      motion_request.content().lockto = MARY_LOCK; 
      motion_request.content().unlock = false; 
      outPort_mot.write(motion_request);


      //Get full res left im:
      imgl = pcl.read(); //blocking buffered
      //convert to RGBA:


      //save to disk:
      num++;
      FILE* f =fopen("image"+QString::number(num)+".bin","wb");
      //FILE* f =fopen("image.bin","wb");
      fwrite(imgl->getPixelAddress(0,0),sizeof(unsigned char),imgl->width()*3*imgl->height(),f);
      fclose(f);

      //just for display: 
      ippiCopy_8u_C3AC4R(imgl->getPixelAddress(0,0),imgl->width()*3,colour,psb4,insize);
      //convert to yuv for display (problems displaying colour):
      c_rgb2yuv->proc(colour,psb4);
      //display grey:
      d_im->display(c_rgb2yuv->get_y());


      //replace with 'THE' call:
      printf("PROCESSING...\n");

      //run matlab on image.bin:
      //execvp("/home/andrew/src/iCub/src/primateVision/demos/mary/edgeDetectordemo",NULL);
      printf(" Edge Detected... \n");

      //run segmentation:
      //execvp("/home/andrew/src/iCub/src/primateVision/demos/mary/segFixReg image.bin gradient.bin orient.bin",NULL);
      printf(" Segmentation Done ... \n");

      //Move the images...
      //execvp("mv image.png image"+QString::number(num)+".png",NULL);
      //execvp("mv edgeMap.png edgeMap"+QString::number(num)+".png",NULL);
      //execvp("mv segWithoutColor.png segWithoutColor"+QString::number(num)+".png",NULL);
      //execvp("mv segWithColor.png segWithColor"+QString::number(num)+".png",NULL);

      printf("DONE. UNLOCKING MOTION\n");
      //unlock motion:
      motion_request.content().lockto = MARY_LOCK; 
      motion_request.content().unlock = true; 
      outPort_mot.write(motion_request);  
    }



    
  }
  
  //never here! 
  
}

