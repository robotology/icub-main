
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
#define THRESH 40
#define NUM_BINS 50


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

  //stretch image right to max_width.

  ippiSet_8u_C1R(0,out,psb_out,sz_o);
  
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
  Network::connect("/ocsserver_0/output/featOr" , "/objRec2/input/or");
  Bottle *inBot_or;
  //For getting edge data:
  BufferedPort<Bottle> inPort_e;  
  inPort_e.open("/objRec2/input/edge");  
  Network::connect("/ocsserver_0/output/M" , "/objRec2/input/edge");
  Bottle *inBot_e;
#endif

  int max_width = zdf_width/2;
  int polar_width = 96;
  int polar_height = 128;
  IppiSize polar_size = {polar_width,polar_height};

  printf("%d %d %d\n",polar_width,polar_height,max_width);

  int psb_8u,pol_psb_8u;
#if ORIENT
  Ipp8u *orient                    = ippiMalloc_8u_C1(zdf_width,zdf_height,&psb_8u);
  Ipp8u *edge                      = ippiMalloc_8u_C1(zdf_width,zdf_height,&psb_8u);
#endif
  Ipp8u  *pol_seg                  = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *pol_seg_rolled           = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *pol_seg_rolled_stretched = ippiMalloc_8u_C1(polar_width,polar_height,&pol_psb_8u);
  Ipp8u  *out                      = ippiMalloc_8u_C1(zdf_width,zdf_height,&psb_8u);
  Ipp8u  *tex_in,*dog_in;
  Ipp8u *orient_in,*edge_in;


  iCub::contrib::primateVision::Display *d_in     = new iCub::contrib::primateVision::Display(zdf_size,zdf_psb,D_8U_NN,"IN");
  iCub::contrib::primateVision::Display *d_pol    = new iCub::contrib::primateVision::Display(polar_size,pol_psb_8u,D_8U_NN,"POL");
#if ORIENT
  iCub::contrib::primateVision::Display *d_in_o   = new iCub::contrib::primateVision::Display(zdf_size,psb_8u,D_8U_NN,"IN O");
  iCub::contrib::primateVision::Display *d_in_e   = new iCub::contrib::primateVision::Display(zdf_size,psb_8u,D_8U_NN,"IN E");
#endif
  iCub::contrib::primateVision::Display *d_pol_r  = new iCub::contrib::primateVision::Display(polar_size,pol_psb_8u,D_8U_NN,"POL R");
  iCub::contrib::primateVision::Display *d_pol_rs = new iCub::contrib::primateVision::Display(polar_size,pol_psb_8u,D_8U_NN,"POL RS");
  iCub::contrib::primateVision::Display *d_out_i  = new iCub::contrib::primateVision::Display(zdf_size,psb_8u,D_8U_NN,"OUT INV");



  int cog_x,cog_y;
  int up_pix;
  int rightmost_x,rightmost_y;

  int av_orient;
  int stretch = max_width;
  int rightmost = max_width;

  Ipp32s histo[NUM_BINS];
  Ipp32s levels[NUM_BINS+1];
  double av, X, Y;
  

  printf("ObjRec2: begin..\n");
  while (1){


    // get seg and CoG from ZDFSERVER:
    zdfData = inPort_zdf.read(); //blocking
    tex_in = (Ipp8u*) zdfData->tex.getRawImage();
    dog_in = (Ipp8u*) zdfData->dog.getRawImage();
    cog_x = zdfData->cog_x;
    cog_y = zdfData->cog_y;

#if ORIENT
    // get edge orientation map from OCSSERVER:
    inBot_or = inPort_or.read(); //blocking
    orient_in = (Ipp8u*) inBot_or->get(0).asBlob();  //0-180 inclusive.

    // get edge map from OCSSERVER:
    inBot_e = inPort_e.read(); //blocking
    edge_in = (Ipp8u*) inBot_e->get(0).asBlob();  

    // make copy of foveal orientation data:
    ippiCopy_8u_C1R(&orient_in[ocs_psb*(ocs_height-zdf_height)/2 + (ocs_width-zdf_width)/2],
		    ocs_psb,orient,psb_8u,zdf_size);
    ippiCopy_8u_C1R(&edge_in[ocs_psb*(ocs_height-zdf_height)/2 + (ocs_width-zdf_width)/2],
		    ocs_psb,edge,psb_8u,zdf_size);

    //mask it:
    for (int y=0;y<zdf_height;y++){
      for (int x=0;x<zdf_width;x++){
	//mask non-edges and bg to 255:
	if (edge[y*psb_8u + x] < THRESH || tex_in[y*zdf_psb + x] == 0 ){ //if not edge within seg.
	  orient[y*psb_8u + x] = 255; //white
	}	  
      }
    }
    //orient[0..179,255].

#endif


    // polar transform tex seg about CoG:
    cart2polar_8u(tex_in,zdf_psb,zdf_size, 
		  pol_seg,pol_psb_8u,polar_size, 
		  cog_x,cog_y);


#if ORIENT


    //erode image a bit to remove boundary effects:

 

    //get histogram:
    //ippiHistogramEven_8u_C1R(const Ipp8u*, int, IppiSize, Ipp32s*, Ipp32s*, int, Ipp32s, Ipp32s)’
    ippiHistogramEven_8u_C1R(orient,psb_8u,zdf_size,histo,levels,NUM_BINS+1,1,180);
    //for (int n=0;n<NUM_BINS;n++){
    //  printf("%d [%d-%d]:%d\n",n*180/NUM_BINS,levels[n],levels[n+1],histo[n]);
    //}

    //get 'average' orientation of egde features within the segmentation, 
    //taking into account 0-180 wraparound using the ringCoG method!!!!!
    X=0.0,Y=0.0;
    for (int n=0;n<NUM_BINS;n++){
      X += (1.0+histo[n])*sin((IPP_PI/180.0)*n*360/NUM_BINS); //r.sin(theta)
      Y += (1.0+histo[n])*cos((IPP_PI/180.0)*n*360/NUM_BINS); //r.cos(theta)
    }
    av = atan2(X,Y)*(180.0/IPP_PI)/2.0;
    //printf("X:%f, Y:%f - av:%f\n",X,Y,av);
    printf("av:%f\n",av);

    //rotate to align with av:
    up_pix = (int) av*polar_size.height/360.0;
#endif


    //get rightmost pixel:
    getMaxRightXY_8u(pol_seg,pol_psb_8u,polar_size,&rightmost_x,&rightmost_y);

#if !ORIENT
    up_pix = rightmost_y;
#endif


    //ORIENT INVARIANCE: 
    //roll polar image up:
    //transition slowly!
    if (up_pix<0){
      //rolling up by -n is equiv to roll down by (height-n)
      up_pix = polar_size.height-abs(up_pix);
    }
    rollUp_8u(up_pix,pol_seg,pol_psb_8u,polar_size, pol_seg_rolled,pol_psb_8u);
    


    //SCALE INVARIANCE:
    //stretch rightmost pix in polar map right to max_width.
    //stretchRight_8u(rightmost_x,max_width,pol_seg_rolled,pol_psb_8u, polar_size,  pol_seg_rolled_stretched, pol_psb_8u);
    //no! just transition slowly! stretch TOWARDS max_width to prevent jumps if segmentation shifts!:
    
    if (rightmost_x == max_width){
      //convert result back to cartesian space immediately:
      polar2cart_8u(pol_seg_rolled,pol_psb_8u,polar_size, out,psb_8u,zdf_size);
    }
    else{
      //shift stretch towards max_width:
      if (stretch<max_width){
	stretch++;
      }
      else if (stretch>max_width){
	stretch--;
      }
      //shift rightmost towards rightmost_x:
      if (rightmost<rightmost_x){
	rightmost++;
      }
      else if (rightmost>rightmost_x){
	rightmost--;
      }
      //stretch:
      stretchRight_8u(rightmost,stretch,pol_seg_rolled,pol_psb_8u, polar_size,  pol_seg_rolled_stretched, pol_psb_8u);
      //convert result back to cartesian space:
      polar2cart_8u(pol_seg_rolled_stretched,pol_psb_8u,polar_size,  out,psb_8u,zdf_size);
    }
    




    // display orientation & scale invarient output:
    d_in->display(tex_in);
    d_pol->display(pol_seg);
#if ORIENT
    d_in_o->display(orient);
    d_in_e->display(edge);
#endif
    d_pol_r->display(pol_seg_rolled);
    d_pol_rs->display(pol_seg_rolled_stretched);
    d_out_i->display(out);



    //NOW SEND THE ORIENTED SEG TO CLASSIFIER!!


  }
  
  //never here!
  return a->exec();
}


