// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Julio Gomes
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "cv.h"
#include "cxcore.h"

class CFaceDetect
{
public:
	//vars
	/*static*/ CvMemStorage* storage;
	/*static*/ CvHaarClassifierCascade* cascade;
	static const char* cascade_name;
	CvSeq* faces;


	IplImage *mask, *in_copy, *fake_in;

	bool first;
	int AreaGreater, area, lostLag;
	CvRect auxRect,last_auxRect,rect;

	//funcs

	/**
	* Class Constructor
	*
	* Creates an instance of class CFaceDetect.
	* Don't forget to call init() to initialize all variables!
	*/
	CFaceDetect();
	~CFaceDetect(){this->close();};
	
	/**
	 * Instance initialization function
	 *
	 * <cols> and <lines> are the dimensions of the largest images to be used
	 * by this instance.
	 */
	void init(int cols, int lines, const char *fileName);

	/**
	 * Deallocation function
	 *
	 * Deallocates images and storage spaces.
	 */
	void close();

	/**
	 * Face detection function
	 *
	 * Detects all faces in input image <in>.
	 * If <in> ROI is set, a fake header is created and <in> is processed only
	 * within that ROI.
	 * Returns the number of detected faces.
	 * NOTE: Input image <in> can be RGB (tested) or Grayscale (not tested yet).
	 */
	int detect(IplImage *in);

	/**
	 * Detected faces sorting function
	 *
	 * Searches internal CvSeq <faces> for the face with greatest area.
	 * Stores bounding box in internal CvRect <auxRect> and returns it too.
	 * IMPORTANT: CALL AFTER detect(), NOT BEFORE!
	 */
	CvRect getBigFace();
	CvRect getLargestFace();
	/**
	 * Detected faces bounding boxes drawing function
	 *
	 * Draws bounding boxes of detected faces on image <in_copy> (an internal
	 * copy of <in> and marks those regions on binary image <mask>.
	 * If <in> ROI is set, a fake header is created and <in_copy> and <mask> are
	 * only processed within that ROI.
	 * IMPORTANT: CALL AFTER detect(), NOT BEFORE!
	 * IMPORTANT: input image <in> _MUST_ be the same as in detect()!
	 * NOTE: may be called before or after getBigFace(). 
	 * NOTE: input image can be RGB (tested) or Grayscale (not tested yet).
	 */
	void draw(IplImage *in);
	//inline CvSeq *getFaces(){return faces};
};
