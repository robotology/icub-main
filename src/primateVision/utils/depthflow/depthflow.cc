/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

#include "depthflow.h"


iCub::contrib::primateVision::Depthflow::Depthflow(IppiSize srcsize_, 
		     int filterSize_,
		     int filterCap_,
		     int windowSize_,
		     int minDisparity_,
		     int numDisparities_,
		     int threshold_,
		     int uniqueness_,
		     double baseline_,
		     int focus_)
{
  fullsize.width  = srcsize_.width;
  fullsize.height = srcsize_.height;
  width           = srcsize_.width;
  height          = srcsize_.height;
  d_range         = numDisparities_;
  d_offset        = minDisparity_;
  baseline        = baseline_;
  focus           = focus_;

  depthflow        = ippiMalloc_32f_C1(width, height, &psb_f);
  depthflow_ret    = ippiMalloc_32f_C1(width, height, &psb_f);
  depth_new        = ippiMalloc_32f_C1(width, height, &psb_f);
  depth_old        = ippiMalloc_32f_C1(width, height, &psb_f);
  depth_ret        = ippiMalloc_32f_C1(width, height, &psb_f);
  disp_ret         = ippiMalloc_8u_C1(width, height, &psb);
  im_l_tmp         = ippiMalloc_8u_C1(width, height, &psb);
  im_r_tmp         = ippiMalloc_8u_C1(width, height, &psb);

  dmap = new Depth(fullsize,
		   filterSize_,
		   filterCap_,
		   windowSize_,
		   minDisparity_,
		   numDisparities_,
		   threshold_,
		   uniqueness_);

}

iCub::contrib::primateVision::Depthflow::~Depthflow()
{

}


void iCub::contrib::primateVision::Depthflow::proc(Ipp8u*im_l_,Ipp8u*im_r_,int psb_i,int ixl,int iyl,int ixr,int iyr){


  //figure out stereo params:
  hd = ixl-ixr + 1;  //horiz displacement between L & R images
  px = (ixl+ixr)/2 - width/2; //centre of l-r overlap x
  py = (iyr+iyl)/2 - height/2; //centre of l-r overlap y
  mind = focus*baseline/(d_range + d_offset + hd);
  maxd = focus*baseline/(d_offset + hd);


  //SHIFT IMAGES TO ALIGN VERTICALLY:
  if (iyl>iyr){//if left higher
    tmpsize.width = fullsize.width;
    tmpsize.height = fullsize.height-(iyl-iyr);
    //copy im_l_ down by (iyr-iyl):
    ippiCopy_8u_C1R(im_l_,psb_i,&im_l_tmp[(iyl-iyr)*psb],psb,tmpsize);
    ippiCopy_8u_C1R(im_r_,psb_i,im_r_tmp,psb,fullsize);
  }
  else{//if right lower
    tmpsize.width = fullsize.width;
    tmpsize.height = fullsize.height-(iyr-iyl);
    //copy im_r_ down by (iyl-iyr):
    ippiCopy_8u_C1R(im_r_,psb_i,&im_r_tmp[(iyr-iyl)*psb],psb,tmpsize);
    ippiCopy_8u_C1R(im_l_,psb_i,im_l_tmp,psb,fullsize);
  }


  

  //Disparity-map:
  dmap->proc(im_l_tmp,im_r_tmp,psb);

 

  //convert to absolute depths:
  disp2depth(dmap->get_disp(),dmap->get_psb(),depth_new,psb_f,fullsize);


  //deal with any motion:
  //overlapping regions of old and new:
  dx=px-old_px;
  dy=py-old_py;
  dsize.width=width-abs(dx);
  dsize.height=height-abs(dy);
  

  //depthflow:
  if (dx>=0 && dy>=0){
    ippiSub_32f_C1R(&depth_old[dy*psb_f+dx],psb_f,depth_new,psb_f,depthflow,psb_f,dsize);
  }
  else if (dx<0 && dy<0){
    ippiSub_32f_C1R(depth_old,psb_f,&depth_new[-dy*psb_f-dx],psb_f,depthflow,psb_f,dsize);
  }
  else if (dx<0 && dy>=0){
    ippiSub_32f_C1R(&depth_old[dy*psb_f],psb_f,&depth_new[-dx],psb_f,depthflow,psb_f,dsize);
  }
  else {//if (dx>=0 && dy<0){
    ippiSub_32f_C1R(&depth_old[dx],psb_f,&depth_new[-dy*psb_f],psb_f,depthflow,psb_f,dsize);
  }  


  
  //copy depth_new to depth_old:
  ippiCopy_32f_C1R(depth_new,psb_f,depth_old,psb_f,fullsize);
  old_px = px;
  old_py = py;
  
  //protect results for yarp delivery:
  ippiCopy_32f_C1R(depth_new,psb_f,depth_ret,psb_f,fullsize);
  ippiCopy_32f_C1R(depthflow,psb_f,depthflow_ret,psb_f,fullsize);
  ippiCopy_8u_C1R(dmap->get_disp(),dmap->get_psb(),disp_ret,psb,fullsize);


}



void iCub::contrib::primateVision::Depthflow::disp2depth(Ipp8u*disp_,int psb_,Ipp32f*depth_,int psbf_,IppiSize s){

  double D,MD;

  if (hd<0){ippiSet_32f_C1R(0.0,depth_,psbf_,s);}             //diverging the eyes is impossible
  else{

    for (int y=0;y<s.height;y++){
      for (int x=0;x<s.width;x++){
	
	if (disp_[psb_*y+x]==0){depth_[(psbf_/4)*y+x] = 0.0;} //0 means no measurement
	else{
	  D = (disp_[psb_*y+x]-1)*d_range/255.0 + d_offset;   //image disparity
	  MD = D + hd;                                        //mosaic disparity
	  if (MD<=0.0){depth_[(psbf_/4)*y+x] = 0.0;}          //-ve MD is impossible
	  else{
	    depth_[(psbf_/4)*y+x] = focus*baseline/MD;        //depth
	  }
	}

      }
    }

  }

//  printf("hd:%d d:%f md:%f depth:%f\n",
//	 hd, 
//	 (disp_[psb_*s.height/2+s.width/2]*(d_range/255.0) + d_offset), 
//	 (disp_[psb_*s.height/2+s.width/2]*(d_range/255.0) + d_offset) + hd,
//	 depth_[(psbf_/4)*s.height/2+s.width/2]
//	 );



}

