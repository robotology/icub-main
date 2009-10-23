
/*esPointsImg project*/


#include <stdio.h>
#include <string>
#include <iostream>
#include <qapplication.h>
#include <q3vbox.h>
#include <qimage.h>
#include <ipp.h>
#include <ippi.h>
#include <math.h>

#include "proc.h"
//#include <recio.h>

#define  PI 3.14159265
#define alpha PI/4


PROC::PROC(QApplication*a_,world3d*w_,string*imfile_)
{

  a=a_;
  world=w_;
  imfile=imfile_;

  start();

}


PROC::~PROC()
{

}


void PROC::run()
{

  //NEW CODE. IMAGE LOADED FROM A FILE -> Ipp8u *dest_im is not used in this case, only with cameras

  //Creating image with black transparent border
  QImage *start_im = new QImage(imfile->c_str(),"JPEG");

  IppiSize srcsize, destsize;
  srcsize.width = start_im->width();
  srcsize.height = start_im->height();
  int ww = start_im->width();
  int hh = start_im->height();
  int diag = (int)sqrt(pow(ww,2)+pow(hh,2));
  int top_bord = (diag-hh)/2 + 1;
  int left_bord = (diag-ww)/2 + 1;
  destsize.width = ww + 2*left_bord;
  destsize.height = hh + 2*top_bord;

  QImage *qim = new QImage(destsize.width, destsize.height, 8*4);
  qim->setAlphaBuffer(true);
  Ipp8u v[4] = {0, 0, 0, 0};
  ippiCopyConstBorder_8u_C4R(start_im->bits(), start_im->width()*4, srcsize, qim->bits(), destsize.width*4, destsize, top_bord, left_bord, v);
  
  //Constant image. Set once
  world->setim(qim,0.0,0.0,0.0,0.0,0.0,alpha,ww,hh);
  
  //main event loop:
  while(1){
    
    //remote callbacks:
    a->lock();
    world->callback();
    a->unlock();

    usleep(500000);


  }


  //CODE FOR IMAGES TAKEN FROM CAMERAS

  /*  Port inPort_s;
  inPort_s.open("/egosphere2/input/serv_params");     // Give it a name on the network.
  Network::connect("/egosphere2/input/serv_params", "/recserver/output/serv_params");
  Network::connect("/recserver/output/serv_params", "/egosphere2/input/serv_params");
  BinPortable<RecServerParams> server_response; 
  Bottle empty;
  inPort_s.write(empty,server_response);
  RecServerParams rsp = server_response.content();
  std::cout << "RecServer Probe Response: " << rsp.toString() << std::endl;


  //RIGHT IMS PORT:
  BufferedPort<Bottle> inPort_ry;      // Create a ports
  inPort_ry.open("/egosphere2/input/rec_ry");     // Give it a name on the network.
  Network::connect("/recserver/output/right_yb" , "/egosphere2/input/rec_ry");
  Bottle *inBot_ry;
  Ipp8u* rec_im_ry;


  RecResultParams* rec_res; 


  int width,height,psb;
  width = rsp.width;
  height = rsp.height;
  psb = rsp.psb;

  IppiSize srcsize, destsize;
  srcsize.width = width;
  srcsize.height = height;


  //QImage *qim = new QImage(width,height,8,256);
  //for(unsigned int ui=0;ui<256;ui++) //set to B&W
  //  {
  //    qim->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
  //    }

  int diag = (int)sqrt(pow(width,2)+pow(height,2));
  int top_bord = (diag-height)/2 + 1;
  int left_bord = (diag-width)/2 + 1;
  destsize.width = width + 2*left_bord;
  destsize.height = height + 2*top_bord;

  QImage *qim = new QImage(destsize.width, destsize.height, 8*4);
  qim->setAlphaBuffer(true);
  Ipp8u v[4] = {0, 0, 0, 0};
  

  //main event loop:
  while(1){

    //GET BARREL RECT RIGHT IM:
    inBot_ry = inPort_ry.read();
    rec_im_ry = (Ipp8u*) inBot_ry->get(0).asBlob();
    rec_res = (RecResultParams*) inBot_ry->get(1).asBlob();
  

    //set new image & pos:
    ippiCopyConstBorder_8u_C4R(rec_im_ry, width*psb, srcsize, qim->bits(), destsize.width*4, destsize, top_bord, left_bord, v);
    
    world->setim(qim,
		 rec_res->deg_rx,
		 rec_res->deg_ry,
		 rec_res->head_r,
		 rec_res->head_p,
		 rec_res->head_y,
		 alpha,
		 width,
		 height
		 );
    
    //remote callbacks:
    a->lock();
    world->callback();
    a->unlock();

  }
*/

    
}

