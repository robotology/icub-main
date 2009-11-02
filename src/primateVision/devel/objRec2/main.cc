
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









void polar2cart_8u(Ipp8u*pol,int psb_pol,IppiSize pol_size, Ipp8u*cart,int psb_cart,IppiSize cart_size){

  int x,y;

  for(int r = 0;r<pol_size.width;r++){
    for(int t = 0;t<pol_size.height;t++){
      x = (int)(r * cos((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.width/2.0);
      y = (int)(r * sin((t*(360.0/pol_size.height) - 180.0)*IPP_PI/180.0) + cart_size.height/2.0);
      if (x<cart_size.width && x>=0 && y<cart_size.height && t>=0){
	cart[y*psb_cart + x] = pol[t*psb_pol + r];
      }
      //else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}
    }
  }
}


void cart2polar_8u(Ipp8u*cart,int psb_cart,IppiSize cart_size,  Ipp8u* pol,int psb_pol,IppiSize pol_size, int cx,int cy){
  
  int r,t; 

  for(int x=0;x<cart_size.width;x++){
    for(int y=0;y<cart_size.height;y++){
      if (cart[y*psb_cart + x] != 0){ //only if occupied
	//radius = dist of this pixel from cog pixel (cx+cart_size.width/2,cy+cart_size.height/2):
	r = (int) sqrt((x-(cx+cart_size.width/2.0))*(x-(cx+cart_size.width/2.0)) + (y-(cy+cart_size.height/2.0))*(y-(cy+cart_size.height/2.0)));
	//theta = 180 + atan(y/x) * rows/360
	t = (int) ( ( 180.0 + atan2(y-(cy+cart_size.height/2.0),x-(cx+cart_size.width/2.0))*(180.0/IPP_PI))  *  pol_size.height/360.0);

	if (r<pol_size.width && r>=0 && t<pol_size.height && t>=0){
	  pol[t*psb_pol + r] = cart[y*psb_cart + x];
	}
	//else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}
      }
    }
  }
}

void cart2polar_32f(Ipp32f*cart,int psb_cart,IppiSize cart_size,  Ipp32f* pol,int psb_pol,IppiSize pol_size, int cx,int cy){
  
  int r,t; 

  for(int x=0;x<cart_size.width;x++){
    for(int y=0;y<cart_size.height;y++){
      if (cart[y*psb_cart/4 + x] != 0.0){ //only if occupied
	//radius = dist of this pixel from cog pixel (cx+cart_size.width/2,cy+cart_size.height/2):
	r = (int) sqrt((x-(cx+cart_size.width/2.0))*(x-(cx+cart_size.width/2.0)) + (y-(cy+cart_size.height/2.0))*(y-(cy+cart_size.height/2.0)));
	//theta = 180 + atan(y/x) * rows/360
	t = (int) ( ( 180.0 + atan2(y-(cy+cart_size.height/2.0),x-(cx+cart_size.width/2.0))*(180.0/IPP_PI))  *  pol_size.height/360.0);

	if (r<pol_size.width && r>=0 && t<pol_size.height && t>=0){
	  pol[t*psb_pol/4 + r] = cart[y*psb_cart/4 + x];
	}
	//else{printf("p:%d,%d <=> c:%d,%d\n",r,t,x,y);}
      }
    }
  }
}





void rollup_8u(int n,Ipp8u*in,int psb_in, IppiSize sz,Ipp8u*out,int psb_out){
  //roll image up by n rows:
  
  IppiSize su = {sz.width,sz.height-n};
  IppiSize sl = {sz.width,n};
  
  //copy nth row downwards of 'in' to 0th row of 'out':
  ippiCopy_8u_C1R(&in[n*psb_in],psb_in,out,psb_out,su);

  //copy 0 to nth row of 'in' to (h-n)th row of 'out':
  ippiCopy_8u_C1R(in,psb_in,&out[(sz.height-n)*psb_out],psb_out,sl);

}



void stretchright_8u(Ipp8u* in, int psb_in,IppiSize sz_o, Ipp8u* out, int psb_out){

  //find rightmost occupied pixel, stretch image right by that many cols.

  //loop over image to find rightmost pixel:
  int max_right = 0;
  for (int y=0;y<sz_o.height;y++){
    for (int x=0;x<sz_o.width;x++){
      if (in[y*psb_in+x]!=0 && x>max_right){
	max_right = x;
      }
    }
  }

  IppiSize sz_i = {sz_o.width-max_right,sz_o.height};
  IppiRect r_i = {0,0,sz_o.width-max_right,sz_o.height};
  double xf = (sz_o.width-max_right)/((double)sz_o.width);

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
  int zdf_psb  = zsp.psb;
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

  //max_rad = (cent to corner) + (max_cog = cent to corner) = full diag, worst case.
  int polar_width = 128;//(int) sqrt(zdf_width*zdf_width + zdf_height*zdf_height); 
  int polar_height = 128;
  IppiSize polar_size = {polar_width,polar_height};

  int pol_seg_psb,out_psb,orient_fov_psb,pol_or_psb;
  Ipp32f *pol_or               = ippiMalloc_32f_C1(polar_width,polar_height,&pol_or_psb);
  Ipp8u  *pol_seg              = ippiMalloc_8u_C1(polar_width,polar_height,&pol_seg_psb);
  Ipp8u  *pol_seg_rolled       = ippiMalloc_8u_C1(polar_width,polar_height,&pol_seg_psb);
  Ipp8u  *pol_seg_rolled_right = ippiMalloc_8u_C1(polar_width,polar_height,&pol_seg_psb);
  Ipp32f *orient_fov           = ippiMalloc_32f_C1(zdf_width,zdf_height,&orient_fov_psb);
  Ipp8u  *out                  = ippiMalloc_8u_C1(zdf_width,zdf_height,&out_psb);

  Ipp8u* seg;


  iCub::contrib::primateVision::Display *d_in  = new iCub::contrib::primateVision::Display(zdf_size,zdf_psb,D_8U_NN,"IN");
  iCub::contrib::primateVision::Display *d_out = new iCub::contrib::primateVision::Display(zdf_size,out_psb,D_8U_NN,"ORIENT INV");





  int n,nc;
  double av,av_col;
  int cog_x,cog_y;


  printf("ObjRec2: begin..\n");
  while (1){


    printf("ObjRec2: acquiring data..\n");

    // get seg and CoG from ZDFSERVER:
    zdfData = inPort_zdf.read(); //blocking
    seg = (Ipp8u*) zdfData->tex.getRawImage();
    cog_x = zdfData->cog_x;
    cog_y = zdfData->cog_y;


    // get orientation map from OCSSERVER:
    inBot_or = inPort_or.read(); //blocking

    // make copy of orientation data in fovea:
    ippiCopy_32f_C1R((Ipp32f*) &inBot_or->get(0).asBlob()[ocs_psb_32f*(ocs_height-zdf_height)/2 + (ocs_width-zdf_width)/2],
		     ocs_psb_32f,orient_fov,orient_fov_psb,zdf_size);



    printf("ObjRec2: polar trans..\n");

    // polar transform seg about CoG:
    ippiSet_8u_C1R(0,pol_seg,pol_seg_psb,polar_size);
    cart2polar_8u(seg,zdf_psb,zdf_size, 
		  pol_seg,pol_seg_psb,polar_size, 
		  cog_x,cog_y);

    // polar transform orient fov about same point as seg:
    ippiSet_32f_C1R(0.0,pol_or,pol_or_psb,polar_size);
    cart2polar_32f(orient_fov,orient_fov_psb,zdf_size, 
		   pol_or,pol_or_psb,polar_size, 
		   cog_x,cog_y); 



    printf("ObjRec2: imposing invariance..\n");

    // av_col = av of all non-zero entries in col c.
    av = 0.0;
    nc = 0;
    for (int col=0;col<polar_width;col++){
      av_col = 0.0; 
      n = 0;
      //get av orient for this col:
      for (int row=0;row<polar_height;row++){
	if (pol_seg[row*pol_seg_psb + col]!=0){ //if occupied
	  if (pol_or[row*pol_or_psb/4 + col]!=0.0){
	    av_col += pol_or[row*pol_or_psb/4 + col];
	    n++;
	  }
	}
      }
      //got sum of entries in this col in av_col.
      if (n!=0){
	//conv to av. for this col
	av_col /= n; 
	//add to overall av.
	av += av_col;
	nc++;
      }
    }
    // got sum of cols_av.s in av
    if (nc!=0){
      //conv to av.
      av /= nc;
    }
    else{av = 0.0;}
    
    printf("av: %f\n",av); 

    //ROLL POLAR IMAGE UP BY "AV" (ORIENT INVARIANCE):
    rollup_8u((int)av,pol_seg,pol_seg_psb,polar_size, pol_seg_rolled, pol_seg_psb);
    
    //stretch polar map fully right (SCALING INVARIANCE):
    stretchright_8u(pol_seg_rolled, pol_seg_psb, polar_size,  pol_seg_rolled_right, pol_seg_psb);
 



    printf("ObjRec2: converting back to cartesian..\n");

    //convert result back to cartesian space:
    ippiSet_8u_C1R(0,out,out_psb,zdf_size);
    polar2cart_8u(pol_seg_rolled_right,pol_seg_psb,polar_size,  out,out_psb,zdf_size);




    // display orient & scale invarient output:
    d_in->display(seg);
    d_out->display(out);



    //NOW SEND THE ORIENTED SEG TO CLASSIFIER!!


  }
  
  //never here!
  return a->exec();
}


