/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff
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

#define KERNSIZE 3    //kernsize (odd, >= 3)

using namespace std;

CentSur::CentSur(cv::Size tmpSize,int tmpGauss, double tmpSigma)
{
    srcSize = tmpSize;
    ngauss = 0;
    ngauss = tmpGauss;
    sigma = tmpSigma;
    psize = (cv::Size*)malloc(ngauss*sizeof(cv::Size));
    proi = (cv::Rect*)malloc(ngauss*sizeof(cv::Rect));

    for (int ng=0;ng<ngauss;ng++)
	{
        psize[ng].width     = (int)ceil( (  (double)srcSize.width ) / double( 1<<ng ) );
        psize[ng].height    = (int)ceil( ( ((double)srcSize.height) / ( (double)srcSize.width) ) * psize[ng].width );
        proi[ng].x          = 0;
        proi[ng].y          = 0;
        proi[ng].width      = psize[ng].width;
        proi[ng].height     = psize[ng].height;
        pyramid[ng]         = cv::Mat ( psize[ng].height, psize[ng].width, CV_32FC1 );
        pyramid_gauss[ng]   = cv::Mat ( psize[ng].height, psize[ng].width, CV_32FC1 );
        gauss[ng]           = cv::Mat ( srcSize.height, srcSize.width, CV_32FC1 );
        
    }
    img_32f			= cv::Mat ( srcSize.height, srcSize.width, CV_32FC1 );
	csTot32f		= cv::Mat ( srcSize.height, srcSize.width, CV_32FC1 );
	csTot32fTmp		= cv::Mat ( srcSize.height, srcSize.width, CV_32FC1 );
	csTot8u			= cv::Mat ( srcSize.height, srcSize.width, CV_8UC1 );
}

CentSur::~CentSur() 
{

    free(psize);
    free(proi);

    for (int ng=0;ng<ngauss;ng++) 
    {
        pyramid[ng].release();
        pyramid_gauss[ng].release();
        gauss[ng].release();
    }
	
    img_32f.release();
	csTot32f.release();
	csTot8u.release();
	csTot32fTmp.release();
}

void CentSur::proc_im_8u(cv::Mat img_8u)
{
    //convert im precision to 32f:
	img_8u.convertTo( img_32f, CV_32FC1, 1.0/255.0 );
    //process as normal:
    proc_im_32f( img_32f );
}

void CentSur::proc_im_32f( cv::Mat im_32f )
{
    //make image & gauss pyramids:
    make_pyramid( im_32f );
    
    //reset tot cs_tot_tmp:
	csTot32f.setTo( cv::Scalar(0) );

    //subtractions (ABSDIFF) to make DOG pyramid:
  	//1st neighbours:  
    for (int nd=0; nd<ngauss-1; nd++)
	{
		cv::absdiff( gauss[nd], gauss[nd+1], csTot32fTmp);
		cv::add(csTot32fTmp, csTot32f, csTot32f);
    }

  	//2nd neighbours:
	for (int ndd=0;ndd<ngauss-2;ndd++)
	{
		cv::absdiff( gauss[ndd], gauss[ndd+2], csTot32fTmp);
        cv::add(csTot32fTmp, csTot32f, csTot32f);
	}
  	/*//norm8u:
  	double min, max;
    min = 0.0f;
    max = 0.0f;
    cv::minMaxLoc(csTot32f, &min, &max);
  	if (max == min){max=255.0f;min=0.0f;}*/

	//convert back to 8u with scale
	csTot32f.convertTo( csTot8u, CV_8UC1, 1.0 * 255.0 );
}

void CentSur::make_pyramid( cv::Mat im_32f )
{
    //copy im to pyramid[0]:
    im_32f.copyTo( pyramid[0] );
    //filter first pyramid:
    cv::Point anchor(-1,-1);
    cv::Size ksize(KERNSIZE, KERNSIZE);
    cv::boxFilter(pyramid[0], pyramid_gauss[0],-1, ksize, anchor, true, cv::BORDER_REPLICATE); 
	//cv::GaussianBlur(pyramid[0], pyramid_gauss[0], cv::Size(1, 1), sigma, 0.0, cv::BORDER_REPLICATE );
    //copy filter output to gauss:
	pyramid_gauss[0].copyTo( gauss[0] );

	double sd = 0.5;
	double su = 2.0;

    for (int sg=1;sg<ngauss;sg++)
    {
        //Downsize previous pyramid image by half:
		cv::resize(pyramid[sg-1], pyramid[sg], cv::Size(proi[sg].width, proi[sg].height), sd, sd, cv::INTER_LANCZOS4);
		//filter:
        cv::Point anchor(-1,-1);
        cv::Size ksize(KERNSIZE, KERNSIZE);
        cv::boxFilter(pyramid[sg], pyramid_gauss[sg],-1, ksize, anchor, true, cv::BORDER_REPLICATE); 
		//cv::GaussianBlur(pyramid[sg], pyramid_gauss[sg], cv::Size(1, 1), sigma, 0.0, cv::BORDER_REPLICATE );
		su = double(1<< sg); 
        //Upsize and store to gauss:
		cv::resize(pyramid_gauss[sg], gauss[sg], srcSize/*cv::Size(proi[sg].width,proi[sg].height)*/, su, su, cv::INTER_LANCZOS4);
//cv::INTER_LANCZOS4 INTER_CUBIC
    }
}
