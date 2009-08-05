/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#include "mosaic.h"
#include <math.h>
#include <convert_bitdepth.h>


iCub::contrib::primateVision::Mosaic::Mosaic(IppiSize vsize_,IppiSize isize_, int psb_i_,int type_, QString qs)
{
  isize.width=isize_.width;
  isize.height=isize_.height;
  vsize.width=vsize_.width;
  vsize.height=vsize_.height;
  psb_i=psb_i_;

  type = type_;
  if (type==D_32F ||
      type==D_32FCR ||
      type==D_32FCI ||
      type==D_32FCA){
    im_ss = ippiMalloc_32f_C1(isize.width,isize.height,&psb_32f);
  }
  if ( type!=D_QIM_G && type!=D_QIM_RGB ){
    im_r  = new QImage(isize.width, isize.height, 8, 256); 
  }

  viewer = new multiFrameViewer(vsize.width, vsize.height);
  viewer->setCaption(qs);
  viewer->setFixedSize(vsize.width, vsize.height);
  viewer->show();

}




void iCub::contrib::primateVision::Mosaic::display(void *im,int ixl,int iyl)
{

  convert(im);

  int locations[2]={(vsize.width-isize.width)/2 +ixl, (vsize.height-isize.height)/2 +iyl};
  
  viewer->showViews(1,&qim,locations);

}



iCub::contrib::primateVision::Mosaic::~Mosaic()
{

}

void iCub::contrib::primateVision::Mosaic::convert(void* im)
{
  if (type == D_QIM_G || type == D_QIM_RGB){
    qim = (QImage*) im;
  }
  else if (type == D_8U){
    qim = (QImage*) p_8u((Ipp8u*) im);
  }
  else if (type == D_8U_NN){
    qim = (QImage*) p_8u_nn((Ipp8u*) im);
  }
  else if (type == D_32F){
    qim = (QImage*) p_32f((Ipp32f*) im);
  }
  else if (type == D_32FCR){
    qim = (QImage*) p_32fc_R((Ipp32fc*) im);
  }
  else if (type == D_32FCI){
    qim = (QImage*) p_32fc_I((Ipp32fc*) im);
  }
  else{// if (type == D_32FCA){
    qim = (QImage*) p_32fc_A((Ipp32fc*) im);
  }
  if (type != D_QIM_RGB){  
    for(unsigned int ui=0;ui<256;ui++) //set to B&W
      {
	qim->setColor(ui,(ui|(ui<<8)|(ui<<16)|0xff000000));
      }    
  }
}


QImage* iCub::contrib::primateVision::Mosaic::p_32f(Ipp32f*im)
{
  //convert to viewable form:
  conv_32f_to_8u(im,psb_i,(Ipp8u*)im_r->bits(),isize.width,isize);
  
  return im_r;
}

QImage* iCub::contrib::primateVision::Mosaic::p_8u(Ipp8u*im)
{
  //get min,max:
  ippiMinMax_8u_C1R(im,psb_i,isize,&min8,&max8);
  //shift min to 0.0:
  ippiSubC_8u_C1RSfs(im,psb_i,min8,im_r->bits(),isize.width,isize,0); 
  //scale image so max is 255.0:
  if (max8!=min8){
    sc8 = (Ipp8u) 255/(max8-min8);
    ippiMulC_8u_C1IRSfs(sc8,im_r->bits(),isize.width,isize,0);
  }
  
    return im_r;
}


QImage* iCub::contrib::primateVision::Mosaic::p_8u_nn(Ipp8u*im)
{

    ippiCopy_8u_C1R(im,psb_i,im_r->bits(),isize.width,isize);

    return im_r;
}

QImage* iCub::contrib::primateVision::Mosaic::p_32fc_R(Ipp32fc*im)
{
  //convert image to real:
  for (int y=0;y<isize.height;y++){ 
    for (int x=0;x<isize.width;x++){
      im_ss[y*isize.width+x] = im[y*isize.width+x].re;
    }
  }
  return  p_32f(im_ss);
}

QImage* iCub::contrib::primateVision::Mosaic::p_32fc_I(Ipp32fc*im)
{
  //convert image to imag:
  for (int y=0;y<isize.height;y++){ 
    for (int x=0;x<isize.width;x++){
      im_ss[y*isize.width+x] = im[y*isize.width+x].im;
    }
  }
  return  p_32f(im_ss);
}

QImage* iCub::contrib::primateVision::Mosaic::p_32fc_A(Ipp32fc*im)
{
  //convert image to Abs:
  for (int y=0;y<isize.height;y++){ 
    for (int x=0;x<isize.width;x++){
      im_ss[y*isize.width+x] = sqrt(im[y*isize.width+x].im*im[y*isize.width+x].im+im[y*isize.width+x].re*im[y*isize.width+x].re);
    }
  }
  return  p_32f(im_ss);
}

