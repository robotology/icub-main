// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _IMAGEPROCESSOR_H_
#define _IMAGEPROCESSOR_H_

#include <ace/config.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//utils includes
#include <iCub/canny.h>
#include <iCub/convert_bitdepth.h>

#include <ipp.h>
#include <ipps.h>
//#include <qimage.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/*
  This code groups together a series of useful function that can be used for ImageProcessing


  Suppose we have an image source on port /source such as:
    yarpdev --device test_grabber --name /source --mode line --framerate 10

  And suppose we have an image viewer on port /view:
    yarpview --name /view

  Then we can hook this program up in between as follows:
    ./image_process --name /worker
    yarp connect /source /worker
    yarp connect /worker /view

  You should see the normal scrolling line of test_grabber, with a moving
  circle overlaid.

 */


class ImageProcessor
{
	private:
		ImageOf<PixelMono>* green_yarp;//green 1-channel image second plane of the input image
		ImageOf<PixelMono>* red_yarp; //red 1-channel image first plane of the input image
		ImageOf<PixelMono>* blue_yarp; //blue 1-channel image third plane of the input image
		Ipp8u* redGreen_ippi; //ippi image of the R+G- Opponency Colour Image  necessary for computation
		Ipp8u* greenRed_ippi; //ippi image of the G+R- Opponency Colour Image  necessary for computation
		Ipp8u* blueYellow_ippi; //ippi image of the B+Y- Opponency Colour Image  necessary for computation
		Ipp8u* greenPlane_ippi; //ippi image of the green plane necessary for computation
		Ipp8u* redPlane_ippi; //ippi image of the red plane necessary for computation
		Ipp8u* bluePlane_ippi; //ippi image of the blue plane necessary for computation
		//temporal objects for CombineMax process
		ImageOf<PixelMono>* edgesBlue;
		ImageOf<PixelMono>* edgesRed;
		ImageOf<PixelMono>* edgesGreen;
		ImageOf<PixelMono>* outputImage; //temp variable for plane extraction;
		//ImageOf<PixelMono>* outputImagePlane; //temp variable for plane extraction;
		
		ImageOf<PixelMono> *bluePlane_tmp; //temp variable for plane extraction;
		ImageOf<PixelMono> *redPlane_tmp; //temp variable for plane extraction;
		ImageOf<PixelMono> *greenPlane_tmp;//temp variable for plane extraction;
		ImageOf<PixelMono> *tmp;//temp variable for plane extraction;
		ImageOf<PixelRgb> *image_out;
		ImageOf<PixelMono> *image_tmp;

		Ipp32f* redPlane_ippi32; //temp variable for the plane extraction
		Ipp32f* bluePlane_ippi32; //temp variable for the plane extraction
		Ipp32f* greenPlane_ippi32; //temp variable for the plane extraction

		Ipp32f* redGreen_ippi32; //temp variable for the plane extraction
		Ipp32f* blueYellow_ippi32; //temp variable for the plane extraction
		Ipp32f* greenRed_ippi32; //temp variable for the plane extraction

		Ipp32f* redPlane_ippi32_f; //temp variable for the plane extraction
		Ipp32f* bluePlane_ippi32_f; //temp variable for the plane extraction
		Ipp32f* yellowPlane_ippi32_f; //temp variable for the plane extraction
		Ipp32f* greenPlane_ippi32_f; //temp variable for the plane extraction
		
		int width; //deprecated
		int height; //deprecated
	
	public:
		ImageProcessor();//default constructor
		ImageProcessor ( ImageOf<PixelRgb>* inputImage );//constructor
		ImageOf<PixelMono>* combineMax(); //combines the 3 edge images saving the maximum value for every pixel 
		ImageOf<PixelMono>* findEdges (); //the output image is the edged image of the input image along the axes x and y
		ImageOf<PixelMono>* findEdgesRedOpponency (); //finds the edges in the red plane
		ImageOf<PixelMono>* findEdgesGreenOpponency (); //finds the edges in the green plane
		ImageOf<PixelMono>* findEdgesBlueOpponency (); //finds the edges in the blue plane
		//ImageOf<PixelMono>* findEdgesRed (); //finds the edges in the red plane
		ImageOf<PixelMono>* findEdgesGreen (); //finds the edges in the green plane
		ImageOf<PixelMono>* findEdgesBlue (); //finds the edges in the blue plane
		ImageOf<PixelRgb>* getOutputImage(); //gets as output the outputImage (last image produced)
		ImageOf<PixelMono>* getRedPlane ( ImageOf<PixelRgb>* src,ImageOf<PixelMono>* tmp ); //get the Red plane of the input image
		ImageOf<PixelMono>* getBluePlane ( ImageOf<PixelRgb>* src,ImageOf<PixelMono>* tmp ); //get the Blue plane of the input image
		ImageOf<PixelMono>* getGreenPlane ( ImageOf<PixelRgb>* src,ImageOf<PixelMono>* tmp ); //get the Green plane of the input image
		ImageOf<PixelMono>* normalize ( ImageOf<PixelMono>* src ); //normalise the image between the max and min value
		ImageOf<PixelMono>*  LShiftC ( ImageOf<PixelMono> *src ); //uses IPP function to left shift the src image
		ImageOf<PixelMono>* RShiftC (ImageOf<PixelMono> *src); //uses IPP function to right shift the src image
		void minMax(ImageOf<PixelMono> *src,int* maxValue,int* minValue);
		ImageOf<PixelMono>* fullRange(ImageOf<PixelMono> *src,int *mx, int *mn);
		void colourOpponency(ImageOf<PixelRgb> *src,ImageOf<PixelMono> *redGreen,ImageOf<PixelMono> *greenRed,ImageOf<PixelMono> *blueYellow);
		void colourOpponency(ImageOf<PixelRgb> *src); //creates the color opponency images
		ImageOf<PixelMono>* lineMax(ImageOf<PixelMono> *src);
		ImageOf<PixelMono>* add(ImageOf<PixelMono> *src1,ImageOf<PixelMono> *src2); //add two images (using IPP)
		ImageOf<PixelMono>* subtract(ImageOf<PixelMono> *src1, ImageOf<PixelMono> *src2); //subtract two images (using IPP)
		void convertRGB(IppiSize sze,Ipp8u * rgba_orig, int psb_o); //convert pixel order rgba to planar order yuv channels
		void setInputImage(ImageOf<PixelRgb> *src);
		void setInputImage(ImageOf<PixelMono> *src);
		ImageOf<PixelRgb>* process (ImageOf<PixelRgb> *src); //process the src image considering the imageprocessor flags
		//--- attributs (necessarily public to be visible to static functions
		ImageOf<PixelRgb> *inImage; // input image  of the processing
		ImageOf<PixelMono>* redGreen_yarp;
		ImageOf<PixelMono>* greenRed_yarp;
		ImageOf<PixelMono>* blueYellow_yarp;
		ImageOf<PixelMono>* greenRedEdges_yarp;
		ImageOf<PixelMono>* redGreenEdges_yarp;
		ImageOf<PixelMono>* blueYellowEdges_yarp;
		ImageOf<PixelMono>* edges_yarp;
		ImageOf<PixelRgb> *portImage; //temporary image of the last processing performed
		
		ImageOf<PixelMono>* redGreenEdges; //temp images necessary for denoising
		ImageOf<PixelMono>* greenRedEdges; //temp images necessary for denoising
		ImageOf<PixelMono>* blueYellowEdges; //temp images necessary for denoising
		ImageOf<PixelMono>* edgesOutput; //temp images necessary for denoising
		//
		ImageOf<PixelMono> *bluePlane; //temp images necessary for planes extraction
		ImageOf<PixelMono> *redPlane; //temp images necessary for planes extraction
		ImageOf<PixelMono> *greenPlane; //temp images necessary for planes extraction
		ImageOf<PixelMono> *yellowPlane; //temp images necessary for planes extraction
		
		IplImage *cvImage; //tmp IPLImage necessary for edge detection
		Ipp8u* im_out;
		
		
		CANNY *cannyOperator; //--- CANNY Operator
		
		int inputImage_flag;  //--- processor flag
		int redPlane_flag; //--- processor flag
		int greenPlane_flag; //--- processor flag
		int bluePlane_flag; //--- processor flag
		int yellowPlane_flag; //--- processor flag
		int colourOpponency_flag;//--- processor flag
		int findEdges_flag; //--- processor flag
		int normalize_flag; //--- processor flag
		int combineMax_flag; //--- processor flag
		
		int blueYellow_flag;
		int redGreen_flag;
		int greenRed_flag;


		//---
		Ipp32s src0[9];
		Ipp32s src1[9];
		Ipp32s src2[9];
		Ipp32s src3[9];
		Ipp32s src4[9];
		Ipp32s src5[9];
		Ipp32s src6[9];
		Ipp32s src7[9];

		//---- 
		static const int CONVMAX_TH=100; //parameter of the findEdges function
		static const int CONVSEQ_TH=500; //parameter of the findEdges function
		
};

#endif // _IMAGEPROCESSMODULE_H_
