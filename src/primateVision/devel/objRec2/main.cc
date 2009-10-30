
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


using namespace iCub::contrib::primateVision;









void polar2cart_8u(Ipp8u*pol,int psb_pol,int pw,int ph,  Ipp8u*cart,int psb_cart){

  int x,y;

  for(int r = 0;r<pw;r++){
    for(int t = 0;t<ph;t++){
      x = r * cos(t);
      y = r * sin(t);
      cart[x*psb_cart + y] = pol[t*psb_pol + r];
    }
  }
}


void cart2polar_8u(Ipp8u*cart,int psb_cart, int cw,int ch, Ipp8u* pol, int psb_pol,  int cx,int cy){

  int r,t;

  for(int x = 0;x<cw;x++){
    for(int y = 0;y<ch;y++){
      r = sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
      if (x-cx!=0){
	t = atan((y-cy)/(x-cx));
      }
      else {t = 0;}
      pol[t*psb_pol + r] = cart[y*psb_cart + x];
    }
  }
}



void cart2polar_32f(Ipp32f*cart,int psb_cart, int cw,int ch, Ipp32f* pol, int psb_pol,  int cx,int cy){

  int r,t;

  for(int x = 0;x<cw;x++){
    for(int y = 0;y<ch;y++){
      r = sqrt((x-cx)*(x-cx) + (y-cy)*(y-cy));
      if (x-cx!=0){
	t = atan((y-cy)/(x-cx));
      }
      else {t = 0;}
      pol[t*psb_pol + r] = cart[y*psb_cart + x];
    }
  }
}



void rollup_8u(int n,Ipp8u*in,int psb_in, int w, int h,Ipp8u*out,int psb_out){
  //roll image up by n rows:
  
  IppiSize su = {w,h-n};
  IppiSize sl = {w,n};
  
  //copy nth row downwards of 'in' to 0th row of 'out':
  ippiCopy_8u_C1R(&in[n*psb_in],psb_in,out,psb_out,su);

  //copy 0 to nth row of 'in' to (h-n)th row of 'out':
  ippiCopy_8u_C1R(in,psb_in,&out[(h-n)*psb_out],psb_out,sl);

}



void stretchright_8u(Ipp8u* in, int psb_in,int w,int h, Ipp8u* out, int psb_out){

  IppiSize sz_o = {w,h};
  //find rightmost occupied pixel, stretch image right by that many cols.

  //loop over image to find rightmost pixel:
  int max_right = 0;
  for (int y=0;y<h;y++){
    for (int x=0;x<w;x++){
      if (in[y*psb_in+x]!=0 && x>max_right){
	max_right = x;
      }
    }
  }

  IppiSize sz_i = {w-max_right,h};
  IppiRect r_i = {0,0,w-max_right,h};
  double xf = (w-max_right)/w;

  //stretch remaining image right:
  ippiResize_8u_C1R(in,sz_i,psb_in,r_i,  out,psb_out,sz_o,  xf,1.0,IPPI_INTER_LINEAR);

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
  IppiSize zdf_size={zdf_width,zdf_height};



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





  //For getting zdf data:
  BufferedPort<ZDFServerData > inPort_zdf;  
  inPort_zdf.open("/objRec2/input/zdfdata");
  Network::connect("/zdfserver/output/data" , "/objRec2/input/zdfdata");
  ZDFServerData *zdfData;


  //For getting orientation data:
  BufferedPort<Bottle> inPort_or;  
  inPort_or.open("/objRec2/input/or");  
  Network::connect("/ocsserver_0/output/phaseOr" , "/objRec2/input/or");
  Bottle *inBot_or;
  Ipp32f* orient;


  //polar width = seg diag/2.
  int polar_width = sqrt(zdf_width*zdf_width + zdf_height*zdf_height)/2; 
  int polar_height = 360;//one row per deg for now.

  int pol_seg_psb;
  Ipp8u*pol_seg              = ippiMalloc_8u_C1(polar_width,polar_height,&pol_seg_psb);
  int pol_or_psb;
  Ipp32f*pol_or              = ippiMalloc_32f_C1(polar_width,polar_height,&pol_or_psb);
  int orient_fov_psb;
  Ipp32f* orient_fov         = ippiMalloc_32f_C1(zdf_width,zdf_height,&orient_fov_psb);
  int pol_seg_rolled_psb;
  Ipp8u*pol_seg_rolled       = ippiMalloc_8u_C1(polar_width,polar_height,&pol_seg_rolled_psb);
  int pol_seg_rolled_right_psb;
  Ipp8u*pol_seg_rolled_right = ippiMalloc_8u_C1(polar_width,polar_height,&pol_seg_rolled_right_psb);
  int out_psb;
  Ipp8u* out                 = ippiMalloc_8u_C1(zdf_width,zdf_height,&out_psb);

  int seg_psb;
  Ipp8u* seg;


  iCub::contrib::primateVision::Display *d_out = new iCub::contrib::primateVision::Display(zdf_size,out_psb,D_8U_NN,"ORIENT INV");





  int n,nn;
  double av,av_col;
  int cog_x,cog_y;


  printf("ObjRec2: begin..\n");
  while (1){


    // get seg and CoG from ZDFSERVER:
    zdfData = inPort_zdf.read(); //blocking
    seg = zdfData->tex.getRawImage();
    seg_psb = zdfData->tex.getRowSize();
    cog_x = zdfData->cog_x;
    cog_y = zdfData->cog_y;

    // get orientation map from OCSSERVER:
    inBot_or = inPort_or.read(); //blocking
    // make copy of orient fov:
    ippiCopy_32f_C1R((Ipp32f*) inBot_or->get(0).asBlob(),ocs_psb_32f,orient_fov,orient_fov_psb,zdf_size);
  
    
    //***[1]*******POLAR TRANSFORM:
    // polar transform seg about CoG.
    cart2polar_8u(seg,seg_psb, zdf_width,zdf_height, pol_seg,pol_seg_psb, cog_x,cog_y);
    // polar transform orient fov about same point as seg:
    cart2polar_32f(orient_fov,orient_fov_psb, zdf_width,zdf_height, pol_or,pol_or_psb, cog_x,cog_y); 
    //*************POLAR TRANSFORM.



    //***[2]***********IMPOSE INVARIANCE:
    // av_col = av of all non-zero entries in col c.
    av =0.0;
    nn = 0;
    for (int col=0;col<zdf_width;col++){
      av_col = 0.0; 
      n = 0;
      for (int row =0;row<zdf_height;row++){
	if (pol_seg[col*pol_seg_psb + row]!=0){
	  av_col += pol_or[col*pol_or_psb + row];
	  n++;
	}
      }
      if (n!=0){
	av_col/=n;
	av += av_col;
	nn++;
      }
    }
    // av = av of all av_cols
    if (nn!=0.0){
      av /= nn;
    }


    printf("%f\n",av); 

    //ROLL POLAR IMAGE UP BY "AV" (ORIENT INVARIANCE):
    if (av!=0.0){
      rollup_8u((int)av,pol_seg,pol_seg_psb, polar_width, polar_height, pol_seg_rolled, pol_seg_rolled_psb);

      //stretch polar map fully right (SCALING INVARIANCE):
      stretchright_8u(pol_seg_rolled, pol_seg_rolled_psb, polar_width,polar_height,  pol_seg_rolled_right, pol_seg_rolled_right_psb);
    }
    else{
      //stretch polar map fully right (SCALING INVARIANCE):
      stretchright_8u(pol_seg, pol_seg_rolled_psb, polar_width,polar_height,  pol_seg_rolled_right, pol_seg_rolled_right_psb);
 
    }

    //**************INVARIANCE.




    //convert result back to cartesian space:
    polar2cart_8u(pol_seg_rolled_right,pol_seg_rolled_right_psb,  polar_width,polar_height,  out,out_psb);




    // display orient & scale invarient output:
    d_out->display(out);



    //NOW SEND THE ORIENTED SEG TO CLASSIFIER!!

   printf(".\n");


  }
  
  //never here!
  return a->exec();
}


