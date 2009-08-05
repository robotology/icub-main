/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_multiframefiewer MultiFrameViewer
 *
 * 
 * A simple class to rapidly display multiple QImages at multiple locations within a GUI widget of specified size. Used by Mosaic and Display classes.
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/multiFrameViewer/multiFrameViewer.h
 */

/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 */

/*      multiFrameViewer:  Andrew Dankers 2003
***********************************************************************
Viewer for up to 8 equal sized QImages (why would you want more? Your 
monitor aint big enough!). Common Width ane height of each image is 
passed in "width" and "height", funnily enough.
**********************************************************************/
#ifndef MFV_H
#define MFV_H

#include <qwidget.h>
#include <qpainter.h>


using namespace std;

namespace iCub {
  namespace contrib {
    namespace primateVision {
      
      
      class multiFrameViewer : public QWidget
      {
	Q_OBJECT     
	  
	public:

	/** Constructor.
	 * @param w Input image width.
	 * @param h Input image height.
	 * @param parent Pointer to QT parent widget.
	 * @param name The name of the display window.
	 */
	multiFrameViewer(int w=0,int h=0,QWidget *parent = 0, const char *name = "mfv");
	
	/** Destructor.
	 */
	~multiFrameViewer();

	/** Method to write images to the qt widget.
	 * @param numviews Number of images to display (up to 8).
	 * @param images Reference to list of pointer 'numviews' images.
	 * @param locations Array of 'numviews' conseccutive x, then y coordinates to place images.
	 */
	void showViews(int numviews, QImage **images, int *locations);
	
	public slots:
	
	protected:
	virtual void paintEvent( QPaintEvent *);
	
      private:
	int numviews;
	QImage **viewframes;
	int * locations;
	
      };

    }
  }
}
#endif
