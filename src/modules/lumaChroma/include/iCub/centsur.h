/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Andrew Dankers, maintainer Vadim Tikhanoff
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

/**
 * \file centsur.h
 * \brief An implementation modelling the centre-surround response, used for construction of spatial uniqueness maps.
 * Based on the difference-of-Gaussians pyramid approach of Itti. A single Gaussian pyramid is created. Neighbouring pyramid entries are subtracted (eg Pyramid level 0 - Pyramid level 1, 1-2, 2-3 ...etc), and so are 2nd neighbours (eg 1-3,2-4,3-5..etc), to obtain spatial uniqueness at various spatial scales. All resultant subtraction results (the difference of Gaussian maps) are summated to yield the centre-surround map output.
 *
 * \author Andrew Dankers
 * \date 2009
 * \note Release under GNU GPL v2.0
 **/

#ifndef __CENTSUR_H__
#define __CENTSUR_H__

#include <cv.h>
#include <highgui.h>
	
class CentSur 
{ 
public:
    /**
    * constructor
    */
    CentSur(cv::Size imsize, int nscale, double sigma = 1.0);
    /**
     * destructor
     */
	~CentSur();

	cv::Mat csTot32f, csTot32fTmp, csTot8u;

    /**
     * convert image to 32f precision
     */ 
    void proc_im_8u(cv::Mat im_8u);

    /**
     * process 32f image creating gauss pyramids:
     */
    void proc_im_32f( cv::Mat im_32f );

    /**
     * get center surround image in 32f precision
    */
	cv::Mat get_centsur_32f(){ return csTot32f; }

    /**
     * get center surround image in 8u precision
     */
	cv::Mat  get_centsur_norm8u(){ return csTot8u; }
    
	
private:

    cv::Size srcSize, *psize;
    cv::Rect *proi;
    cv::Mat pyramid[10], pyramid_gauss[10], gauss[10], img_32f;
    double sigma;
    int ngauss;
    cv::Ptr<cv::FilterEngine> f;
    std::vector<cv::Mat> dst;
    /**
     * creates pyramids
     */
    void make_pyramid(cv::Mat im_in);
};
#endif
//empty line to make gcc happy
