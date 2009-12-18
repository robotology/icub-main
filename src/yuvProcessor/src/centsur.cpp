/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrew Danker, maintainer Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <math.h>
#include <iostream>
#include <stdlib.h>
#include "iCub/centsur.h"

#define KERNSIZE 7    //kernsize (odd, >= 3)

CentSur::CentSur(IppiSize ss_,int ngs, double sigma_)
{

    srcsize = ss_;
    ngauss = ngs;
    sigma = sigma_;

    psize         = (IppiSize*) malloc(ngauss*sizeof(IppiSize));
    proi          = (IppiRect*) malloc(ngauss*sizeof(IppiRect));
    psb_p         = (int*)      malloc(ngauss*sizeof(int));

    pyramid       = (Ipp32f**)  malloc(ngauss*sizeof(Ipp32f*));
    pyramid_gauss = (Ipp32f**)  malloc(ngauss*sizeof(Ipp32f*));
    gauss         = (Ipp32f**)  malloc(ngauss*sizeof(Ipp32f*));

    for (int ng=0;ng<ngauss;ng++){
        psize[ng].width   = (int)ceil(((double)srcsize.width)/pow(2,ng));
        psize[ng].height  = (int)ceil((((double)srcsize.height)/((double)srcsize.width))*psize[ng].width);
        proi[ng].x        = 0;
        proi[ng].y        = 0;
        proi[ng].width    = psize[ng].width;
        proi[ng].height   = psize[ng].height;
        pyramid[ng]       = (Ipp32f*) ippiMalloc_32f_C1(psize[ng].width,psize[ng].height,&psb_p[ng]);
        pyramid_gauss[ng] = (Ipp32f*) ippiMalloc_32f_C1(psize[ng].width,psize[ng].height,&psb_p[ng]);
        gauss[ng]         = (Ipp32f*) ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f); //original size!
    }

    ippiFilterGaussGetBufferSize_32f_C1R(srcsize, KERNSIZE, &pbufsize);//just re-use the bigest buffer!
    pbuf       = (Ipp8u*) malloc(pbufsize*sizeof(Ipp8u));
    im_in_32f  = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    tmp_im_32f = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    cs_tot_32f = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    cs_tot_8u  = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb_8u);
}

CentSur::~CentSur()
{

}

void CentSur::proc_im_8u(Ipp8u* im_8u, int psb_){
    //convert im precision to 32f:
    ippiConvert_8u32f_C1R(im_8u,psb_,im_in_32f,psb_32f,srcsize);
    //process as normal:
    proc_im_32f(im_in_32f,psb_32f);
}

void CentSur::proc_im_32f(Ipp32f* im_32f, int psb_in_32f_)
{
    //make image & gauss pyramids:
    make_pyramid(im_32f,psb_in_32f_);
    //reset tot cs_tot_tmp:
    ippiSet_32f_C1R(0.0,cs_tot_32f,psb_32f,srcsize);
	
    //subtractions (ABSDIFF) to make DOG pyramid:
  	//1st neighbours:  
    for (int nd=0;nd<ngauss-1;nd++){
        ippiAbsDiff_32f_C1R(gauss[nd],psb_32f,
				gauss[nd+1],psb_32f,
				tmp_im_32f,psb_32f,srcsize); 
        ippiAdd_32f_C1IR(tmp_im_32f,psb_32f,cs_tot_32f,psb_32f,srcsize);
    }

  	//2nd neighbours:
  	for (int ndd=0;ndd<ngauss-2;ndd++){
    	ippiAbsDiff_32f_C1R(gauss[ndd],psb_32f,
  				gauss[ndd+2],psb_32f,
  				tmp_im_32f,psb_32f,srcsize);
    	ippiAdd_32f_C1IR(tmp_im_32f,psb_32f,cs_tot_32f,psb_32f,srcsize);
  	}
  
  	//norm8u:
  	//conv_32f_to_8u(cs_tot_32f,psb_32f,cs_tot_8u,psb_8u,srcsize);//----------------------------------to look at conver bitdepth locally
  	Ipp32f min,max;
  	ippiMinMax_32f_C1R(cs_tot_32f,psb_32f,srcsize,&min,&max);
  	if (max==min){max=255.0;min=0.0;}
  	ippiScale_32f8u_C1R(cs_tot_32f,psb_32f,cs_tot_8u,psb_8u,srcsize,min,max);
}

void CentSur::make_pyramid(Ipp32f* im_32f,int p_32_)
{

    //copy im to pyramid[0]:
    ippiCopy_32f_C1R(im_32f,p_32_,
    	   pyramid[0],psb_p[0],
		   srcsize);

    //filter first pyramid:
    ippiFilterGaussBorder_32f_C1R(pyramid[0],
				psb_p[0],
				pyramid_gauss[0],
				psb_p[0],
				psize[0],
				KERNSIZE,
				sigma, 
				ippBorderRepl, //borderType
				0.0,           //foo
				pbuf);

    //copy filter output (within padding) to gauss:
    ippiCopy_32f_C1R(pyramid_gauss[0],psb_p[0],gauss[0],psb_32f,srcsize);
  
    //others:
    sd = 0.5;
    su = 2.0;
    for (int sg=1;sg<ngauss;sg++){
        //Downsize previous pyramid image by half:
        ippiResize_32f_C1R(pyramid[sg-1], //source 
    		       psize[sg-1],   //source size
    		       psb_p[sg-1],   //source step
    		       proi[sg-1],    //source roi
    		       pyramid[sg],   //dst
    		       psb_p[sg],     //dst step
    		       psize[sg],     //dst size
    		       sd,sd,IPPI_INTER_LINEAR);    

        //filter:
        ippiFilterGaussBorder_32f_C1R(pyramid[sg],
    				  psb_p[sg],
    				  pyramid_gauss[sg],
    				  psb_p[sg],
    				  psize[sg],
    				  KERNSIZE,
    				  sigma,
    				  ippBorderRepl,//borderType
    				  0.0,          //foo
    				  pbuf);
    
    
        //Upsize and store to gauss:
        su = pow(2,sg);
        ippiResize_32f_C1R(pyramid_gauss[sg],
    		       psize[sg],
    		       psb_p[sg],
    		       proi[sg],
    		       gauss[sg],
    		       psb_32f,
    		       srcsize,
    		       su,su,IPPI_INTER_LINEAR);    
  }
}
