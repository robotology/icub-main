/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include "flow.h" 
#include <ipp.h>
  
#define BORD_X 16
#define BORD_Y 16

iCub::contrib::primateVision::Flow::Flow(IppiSize imsize_,int flow_scale_)
{

  fullsize=imsize_;
  width = imsize_.width;
  height = imsize_.height;
  flow_scale = flow_scale_;

  swidth = width/flow_scale;
  sheight = height/flow_scale;

  froi.x = 0;
  froi.y = 0;
  froi.width = width;
  froi.height = height;

  sroi.x = 0;
  sroi.y = 0;
  sroi.width = swidth;
  sroi.height = sheight;
  scalesize.width = swidth;
  scalesize.height = sheight;


  //full
  //new_im = ippiMalloc_8u_C1(width, height, &psb);
  old_im = ippiMalloc_8u_C1(width, height, &psb);
  fx     = ippiMalloc_8u_C1(width, height, &psb);
  fy     = ippiMalloc_8u_C1(width, height, &psb);

  //shrunk+border
  new_shrunk = ippiMalloc_8u_C1(swidth+2*BORD_X, sheight+2*BORD_Y, &psb_b);
  old_shrunk = ippiMalloc_8u_C1(swidth+2*BORD_X, sheight+2*BORD_Y, &psb_b);
  new_shrunk_c = ippiMalloc_8u_C1(swidth+2*BORD_X, sheight+2*BORD_Y, &psb_b);
  old_shrunk_c = ippiMalloc_8u_C1(swidth+2*BORD_X, sheight+2*BORD_Y, &psb_b);
  flow       = ippiMalloc_8u_C1(swidth+2*BORD_X, sheight+2*BORD_Y, &psb_b);

  new_m  = MMXMatrixAlloc(MMT_U_8, psb_b, sheight+2*BORD_Y);
  old_m  = MMXMatrixAlloc(MMT_U_8, psb_b, sheight+2*BORD_Y);
  flow_m = MMXMatrixAlloc(MMT_U_8, psb_b, sheight+2*BORD_Y);

  //shrunk
  flow_x = ippiMalloc_8u_C1(swidth, sheight, &psb_s);
  flow_y = ippiMalloc_8u_C1(swidth, sheight, &psb_s);

  scalesize_b.width = psb_b;
  scalesize_b.height = sheight+2*BORD_Y;

}


iCub::contrib::primateVision::Flow::~Flow()
{

}

void iCub::contrib::primateVision::Flow::proc(Ipp8u* new_im_,int psb_in,int newx_, int newy_)
{

  //we know how much the window has moved in the mosaic.
  //align both current with previous to remove the effect of our own motion.

  newx=newx_;
  newy=newy_;

  //make local copy of new_im:
  //ippiCopy_8u_C1R(new_im_,width,new_im,width,fullsize);
  new_im = new_im_;

  //figure out sizes of overlapping area given camera motion:

  dx = newx-oldx;
  dy = newy-oldy;

  //printf("%d %d \n",dx,dy);

  rsize.width = scalesize.width - abs(dx);
  rsize.height = scalesize.height - abs(dy);

  if (dx>=0){
    old_roi.x = dx;
    old_roi.width = width - dx;
    new_roi.x = 0;
    new_roi.width = width - dx;
  }
  else {
    old_roi.x = 0;
    old_roi.width = width + dx;
    new_roi.x = -dx;
    new_roi.width = width + dx;
  }
  if (dy>=0){
    old_roi.y = dy;
    old_roi.height = height - dy;
    new_roi.y = 0;
    new_roi.height = height - dy;
  }
  else {
    old_roi.y = 0;
    old_roi.height = height + dy;
    new_roi.y = -dy;
    new_roi.height = height + dy;
  }


  //get and shrink overlapping region of new image, adding border:
  ippiResize_8u_C1R(new_im,fullsize,psb_in,new_roi,
		    &new_shrunk[BORD_X+BORD_Y*psb_b],psb_b,rsize,
		    1.0/((double)flow_scale),1.0/((double)flow_scale),IPPI_INTER_CUBIC);
  //populate border nicely:
  //ippiCopyConstBorder_8u_C1R(&new_shrunk[BORD_X+BORD_Y*psb_b],psb_b,rsize,new_shrunk_c,psb_b,scalesize_b,BORD_Y,BORD_X,0);
  //MMX_img_MMU8(new_shrunk_c,new_m);
  //ippiCopyReplicateBorder_8u_C1IR(&new_shrunk[BORD_X+BORD_Y*psb_b],psb_b,rsize,scalesize_b,BORD_Y,BORD_X);
  MMX_img_MMU8(new_shrunk,new_m);


  //get and shrink overlapping region of old image, adding border:
  ippiResize_8u_C1R(old_im,fullsize,psb,old_roi,
		    &old_shrunk[BORD_X+BORD_Y*psb_b],psb_b,rsize,
		    1.0/((double)flow_scale),1.0/((double)flow_scale),IPPI_INTER_CUBIC);
  //populate border nicely:
  //ippiCopyConstBorder_8u_C1R(&old_shrunk[BORD_X+BORD_Y*psb_b],psb_b,rsize,old_shrunk_c,psb_b,scalesize_b,BORD_Y,BORD_X,0);
  //MMX_img_MMU8(old_shrunk_c,old_m);
  //ippiCopyReplicateBorder_8u_C1IR(&old_shrunk[BORD_X+BORD_Y*psb_b],psb_b,rsize,scalesize_b,BORD_Y,BORD_X);
  MMX_img_MMU8(old_shrunk,old_m);


  //THE BIG CALL - get flow!:
  MMXRecursive_Depthmap_FLOW2(new_m,old_m,flow_m);

  //uncompress results:
  MMX_MMU8_img(flow_m,flow);

  //shift 4 bits to right and remove border (flowy):
  ippiRShiftC_8u_C1R(flow,psb_b,4,flow_y,psb_s,scalesize);  
  //keep right 4 bits and remove border (flowx) (AND with 00001111 (decimal 15)):
  ippiAndC_8u_C1R(flow,psb_b,15,flow_x,psb_s,scalesize);


  //fx,fy now appear:
  // 0  1  2  3  4  5  6  7 
  // N -3 -2 -1  0  1  2  3   

#if 0
  //x:
  //  //threshold out any 0s! (set N to 4)
  ippiThreshold_LTVal_8u_C1IR(flow_x,psb_s,scalesize,(Ipp8u)1,(Ipp8u)4);
  //resize result:
#endif
  ippiResize_8u_C1R(flow_x,scalesize,psb_s,sroi,
		    fx,psb,fullsize,(double)flow_scale,(double)flow_scale,IPPI_INTER_NN);


#if 0
  //y:
  //  //threshold out any 0s! (set N to 4)
  ippiThreshold_LTVal_8u_C1IR(flow_y,psb_s,scalesize,(Ipp8u)1,(Ipp8u)4);
#endif
  //resize result:
  ippiResize_8u_C1R(flow_y,scalesize,psb_s,sroi,
		    fy,psb,fullsize,(double)flow_scale,(double)flow_scale,IPPI_INTER_NN);


  //save new to old:
  ippiCopy_8u_C1R(new_im,psb_in,old_im,psb,fullsize);
  oldx = newx;
  oldy = newy;

  //prepare results for nice non-normed display:
  //expand to 0-240:
  //sub 1:
  //ippiSubC_8u_C1IRSfs(1,fx,width,fullsize,0);
  //ippiSubC_8u_C1IRSfs(1,fy,width,fullsize,0);
  //   //NOW FLOW IS:
  //   //max left:  0 (-3 pix)
  //   //no motion: 3
  //   //max right: 6 (+3 pix)

  //mul by 40:
  //ippiMulC_8u_C1IRSfs(40,fx,width,fullsize,0);
  //ippiMulC_8u_C1IRSfs(40,fy,width,fullsize,0);
  //   //NOW FLOW IS:
  //   //max left:  0
  //   //no motion: 120
  //   //max right: 240
 
}
 
