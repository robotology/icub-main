/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_display Display
 *
 * 
 * A simple class to rapidly display images of various formats in separate windows. Instantiates an instance of MultiFrameViewer, using only a simgle frame. Also allows for saving of the images to file in the JPEG format.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/display/display.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */


#ifndef DISPLAY_H
#define DISPLAY_H

#include <qimage.h>
#include <qstring.h>
#include <ipp.h>
#include <multiFrameViewer.h>

#define D_8U      1
#define D_32F     2
#define D_32FCR   3
#define D_32FCI   4
#define D_32FCA   5
#define D_QIM_G   6 
#define D_QIM_RGB 7 
#define D_8U_NN   8


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {
      
      
      /** 
       * A simple class to display or save images of various bitdepths within an application.
       */
      class Display
      {
	
      public:
	
	/** Constructor.
	 * @param imsize Input image width and height.
	 * @param psb Step in bytes through the input image.
	 * @param type Type of image to be displayed.
	 * @param title The title of the display window.
	 */
	Display(IppiSize imsize,int psb, int type,QString title);
	
	/** Destructor.
	 */
	~Display();
	
	/** Method to write image to display window.
	 * @param im Pointer to image.
	 */
	void display(void* im);
	
	/** Method to write image to JPEG format file.
	 * @param name Filename, eg: 'myim.jpg'.
	 */
	void save(void* im,QString name);
	
	/** Access to the underlying  multiFrameViewer object (QT4 display window widget).
	 * @return viewer Pointer to multiFrameViewer object.
	 */
	multiFrameViewer * getpviewer(){return viewer;}
	//QPainter* getQPainter(){return viewer->getQPainter();}
	
      private:
	QImage* p_8u(Ipp8u*im);
	QImage* p_8u_nn(Ipp8u*im);
	QImage* p_32f(Ipp32f*im);
	QImage* p_32fc_R(Ipp32fc*im);
	QImage* p_32fc_I(Ipp32fc*im);
	QImage* p_32fc_A(Ipp32fc*im);
	void convert(void* im);
	multiFrameViewer *viewer;
	IppiSize isize;
	int type;
	QImage *qim,*im_r;
	int psb_32f;
	int psb_i;
	Ipp32f min,max,sc;
	Ipp8u min8,max8,sc8;
	Ipp32f* im_ss;
	
      };

    }
  }
}
#endif
