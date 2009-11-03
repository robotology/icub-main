
/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <qapplication.h>
#include <ippi.h>

//MY INCLUDES
#include <zdfio.h>
#include <ocsio.h>
#include <display.h>




#define ORIENT 1
#define DOG_THRESH 10

using namespace iCub::contrib::primateVision;



void polar2cart_8u(Ipp8u*pol,int psb_pol,IppiSize pol_size, Ipp8u*cart,int psb_cart,IppiSize cart_size){

  ippiSet_8u_C1R(0,cart,psb_cart,cart_size);

  int r,t;
  double at;
  
  for(int y=0;y<cart_size.height;y++){
    for(int x=0;x<cart_size.width;x++){
 
      //radius = dist of this pixel from cog pixel (cart_size.width/2,cart_size.height/2):
      r = (int) sqrt((x-cart_size.width/2.0)*(x-cart_size.width/2.0) + (y-cart_size.height/2.0)*(y-cart_size.height/2.0));
      //theta = 180 + atan(y/x) * rows/360:
      
      at = atan2(x-cart_size.width/2.0,y-cart_size.height/2.0);
      if (at == IPP_PI){
	t = 0;
      }
      else{
	t = (int) ( ( 180.0 + at*(180.0/IPP_PI))  *  pol_size.height/360.0);
      }

      if (r<pol_size.width && r>=0 && t<pol_size.height && t>=0){
	cart[y*psb_cart + x] = pol[t*psb_pol + r];
      }
      //else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}

    }
  }
  
}


void cart2polar_8u(Ipp8u*cart,int psb_cart,IppiSize cart_size,  Ipp8u* pol,int psb_pol,IppiSize pol_size, int cx,int cy){
  
  ippiSet_8u_C1R(0,pol,psb_pol,pol_size);

  int x,y; 
  
  for(int t=0;t<pol_size.height;t++){
    for(int r=0;r<pol_size.width;r++){
      
      //x = r sin(t):
      x = (int)(r * sin((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.height/2.0 + cx);
      
      //y = r cos(t):
      y = (int)(r * cos((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.width/2.0  + cy);

      if (x>=0 && x<cart_size.width && y>=0 && y<cart_size.height){
        pol[t*psb_pol + r] = cart[y*psb_cart + x];
      }
      //else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}
	
    }
  }
      
}


void cart2polar_32f(Ipp32f*cart,int psb_cart,IppiSize cart_size,  Ipp32f* pol,int psb_pol,IppiSize pol_size, int cx,int cy){

  ippiSet_32f_C1R(0,pol,psb_pol,pol_size);

  int x,y; 
  
  for(int t=0;t<pol_size.height;t++){
    for(int r=0;r<pol_size.width;r++){

      //x = r sin(t):
      x = (int)(r * sin((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.height/2.0 + cx);

      //y = r cos(t):
      y = (int)(r * cos((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.width/2.0  + cy);

      if (x>=0 && x<cart_size.width && y>=0 && y<cart_size.height){
        pol[t*psb_pol/4 + r] = cart[y*psb_cart/4 + x];
      }
      //else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}
	
    }
  }

}


void getMaxRightXY_8u(Ipp8u* in, int psb_in,IppiSize sz,int*X,int*Y){
   
  //find rightmost occupied pixel
  
  //loop over image to find rightmost pixel:
  int max_right = 0;
  for (int y=0;y<sz.height;y++){
    for (int x=max_right;x<sz.width;x++){
      if (in[y*psb_in+x]!=0 && x>max_right){
	max_right = x;
	*X = x;
	*Y = y;
      }
    }
  }
  
}


void rollUp_8u(int n,Ipp8u*in,int psb_in, IppiSize sz,Ipp8u*out,int psb_out){

  //roll image up by n rows:
  
  IppiSize sz_upper = {sz.width,sz.height-n};
  IppiSize sz_lower = {sz.width,n};
  
  //copy nth row downwards of 'in' to 0th row of 'out':
  ippiCopy_8u_C1R(&in[n*psb_in],psb_in,out,psb_out,sz_upper);
  
  //copy 0 to nth row of 'in' to (h-n)th row of 'out':
  ippiCopy_8u_C1R(in,psb_in,&out[(sz.height-n)*psb_out],psb_out,sz_lower);
  
}


void stretchRight_8u(int max_right, int max_width, Ipp8u* in, int psb_in,IppiSize sz_o, Ipp8u* out, int psb_out){

  //stretch image right.
  
  IppiSize sz_1 = {max_right,sz_o.height};
  IppiRect r_1 = {0,0,max_right,sz_o.height};
  IppiSize sz_2 = {max_width,sz_o.height};
  double xf = ((double)max_width) / ((double)max_right);

  //stretch image right:
  ippiResize_8u_C1R(in,sz_1,psb_in,r_1,  out,psb_out,sz_2,  xf,1.0,IPPI_INTER_LINEAR);

}






int main( int argc, char **argv )
{

  QApplication *a = new QApplication(argc, argv);

  QString arg1 = argv[1];
  bool save = false;
  if (arg1=="save"){
    save = true;
  }


  //Probe ZDFServer:
  Port inPort_z;
  inPort_z.open("/objRec2/input/zdf_serv_params");   
  Network::connect("/objRec2/input/zdf_serv_params" , "/zdfserver/output/serv_params");
  Network::connect("/zdfserver/output/serv_params", "/objRec2/input/zdf_serv_params");
  BinPortable<ZDFServerParams> response; 
  Bottle empty;
  inPort_z.write(empty,response);
  ZDFServerParams zsp = response.content();
  std::cout << "ZDFServer Probe Response: " << zsp.toString() << std::endl; 
  int zdf_width = zsp.width;
  int zdf_height = zsp.height;
  int zdf_psb  = zsp.psb;
  IppiSize zdf_size={zdf_width,zdf_height};

#if ORIENT
  //Probe OCSServer:
  Port inPort_o;
  inPort_o.open("/objRec2/input/ocs_serv_params");     // Give it a name on the network.
  Network::connect("/objRec2/input/ocs_serv_params", "/ocsserver_0/output/serv_params");
  Network::connect("/ocsserver_0/output/serv_params", "/objRec2/input/ocs_serv_params");
  BinPortable<OCSServerParams> server_response; 
  inPort_o.write(empty,server_response);
  OCSServerParams osp = server_response.content();
  std::cout << "OCSServer Probe Response: " << osp.toString() << std::endl;
  int ocs_width     = osp.width;
  int ocs_height    = osp.height;
  int ocs_psb       = osp.psb;
  int ocs_psb_32f   = osp.psb_32f;
  IppiSize ocs_size = {ocs_width,ocs_height};
#endif

  //For getting zdf data:
  BufferedPort<ZDFServerData > inPort_zdf;  
  inPort_zdf.open("/objRec2/input/zdfdata");
  Network::connect("/zdfserver/output/data" , "/objRec2/input/zdfdata");
  ZDFServerData *zdfData;

#if ORIENT
  //For getting orientation data:
  BufferedPort<Bottle> inPort_or;  
  inPort_or.open("/objRec2/input/or");  
  Network::connect("/ocsserver_0/output/phaseOr" , "/objRec2/input/or");
  Bottle *inBot_or;
#endif

  int max_width = zdf_width/2;
  int polar_width = 96;
  int polar_height = 128;
  IppiSize polar_size = {polar_width,polar_height};

  printf("%d %d %d\n",polar_width,polar_height,max_width);

  int psb_8u,psb_32f,pol_psb_8u,pol_psb_32f;
#if ORIENT
  Ipp32f *orient                   = ippiMalloc_32f_C1(zdf_width,zdf_height,&psb_32f);
  Ipp32f *pol_orient               = ippiMalloc_32f_C1(polar_width,polar_height,&pol_psb_32f);
#endif
  Ipp8u  *pol_seg                  = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *pol_seg_rolled           = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *pol_seg_rolled_stretched = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *out                      = ippiMalloc_8u_C1(zdf_width,zdf_height,&psb_8u);
  Ipp8u  *seg_in;
  Ipp32f *orient_in;


  iCub::contrib::primateVision::Display *d_in     = new iCub::contrib::primateVision::Display(zdf_size,zdf_psb,D_8U_NN,"IN");
  iCub::contrib::primateVision::Display *d_pol    = new iCub::contrib::primateVision::Display(polar_size,pol_psb_8u,D_8U_NN,"POL");
#if ORIENT
  iCub::contrib::primateVision::Display *d_in_o   = new iCub::contrib::primateVision::Display(zdf_size,psb_32f,D_32F,"IN OR");
  iCub::contrib::primateVision::Display *d_pol_o  = new iCub::contrib::primateVision::Display(polar_size,pol_psb_32f,D_32F,"POL OR");
#endif
  iCub::contrib::primateVision::Display *d_pol_r  = new iCub::contrib::primateVision::Display(polar_size,pol_psb_8u,D_8U_NN,"POL R");
  iCub::contrib::primateVision::Display *d_pol_rs = new iCub::contrib::primateVision::Display(polar_size,pol_psb_8u,D_8U_NN,"POL RS");
  iCub::contrib::primateVision::Display *d_out_i  = new iCub::contrib::primateVision::Display(zdf_size,psb_8u,D_8U_NN,"OUT INV");



  int n,nc;
  double av,av_col;
  int cog_x,cog_y;
  int up_pix;
  int rightmost_x,rightmost_y;


  printf("ObjRec2: begin..\n");
  while (1){


    // get seg and CoG from ZDFSERVER:
    zdfData = inPort_zdf.read(); //blocking
    seg_in = (Ipp8u*) zdfData->dog.getRawImage();
    cog_x = zdfData->cog_x;
    cog_y = zdfData->cog_y;

#if ORIENT
    // get orientation map from OCSSERVER:
    inBot_or = inPort_or.read(); //blocking
    orient_in = (Ipp32f*) inBot_or->get(0).asBlob();

    // make copy of foveal orientation data:
    ippiCopy_32f_C1R(&orient_in[(ocs_psb_32f/4)*(ocs_height-zdf_height)/2 + (ocs_width-zdf_width)/2],
		     ocs_psb_32f,orient,psb_32f,zdf_size);
#endif


    // polar transform seg about CoG:
    cart2polar_8u(seg_in,zdf_psb,zdf_size, 
		  pol_seg,pol_psb_8u,polar_size, 
		  cog_x,cog_y);


#if ORIENT
    // polar transform orient about same CoG as seg:
    cart2polar_32f(orient,psb_32f,zdf_size, 
		   pol_orient,pol_psb_32f,polar_size, 
		   cog_x,cog_y); 



    // av = av. of all non-zero DoG entries in seg.
    av = 0.0;
    nc = 0;
    for (int col=0;col<polar_width;col++){
      for (int row=0;row<polar_height;row++){
	if (pol_seg[row*pol_psb_8u + col] >= DOG_THRESH){ //if occupied/textured.
	  av += pol_orient[row*pol_psb_32f/4 + col]; //0-180
	  nc++;
	}
      }
    }
    av /= nc;
    av += 280.0;
    up_pix = (int)(av*(polar_size.height/360.0));
    printf("av:%f nc:%d up_pix:%d\n",av,nc,up_pix); 

#endif


    //get rightmost pixel:
    getMaxRightXY_8u(pol_seg,pol_psb_8u,polar_size,&rightmost_x,&rightmost_y);

#if !ORIENT
    up_pix = rightmost_y;
#endif

    //roll polar image up (ORIENT INVARIANCE):    
    rollUp_8u(up_pix,pol_seg,pol_psb_8u,polar_size, pol_seg_rolled,pol_psb_8u);
    
    //stretch rightmosy pix in polar map right to max_width (SCALING INVARIANCE):
    stretchRight_8u(rightmost_x,max_width,pol_seg_rolled,pol_psb_8u, polar_size,  pol_seg_rolled_stretched, pol_psb_8u);
 



    //convert result back to cartesian space:
    polar2cart_8u(pol_seg_rolled_stretched,pol_psb_8u,polar_size,  out,psb_8u,zdf_size);




    // display orientation & scale invarient output:
    d_in->display(seg_in);
    d_pol->display(pol_seg);
#if ORIENT
    d_in_o->display(orient);
    d_pol_o->display(pol_orient);
#endif
    d_pol_r->display(pol_seg_rolled);
    d_pol_rs->display(pol_seg_rolled_stretched);
    d_out_i->display(out);



    //NOW SEND THE ORIENTED SEG TO CLASSIFIER!!


  }
  
  //never here!
  return a->exec();
}


