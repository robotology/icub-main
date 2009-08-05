#ifndef _IMG_PROC_
#define _IMG_PROC_

#include <includes.h>
#include <Gui.h>
#include <Tools.h>

#include <math.h>
#include <cv.h>
#include <highgui.h>

// namespaces 
namespace thesis {
    /**
     * Image Processing.
     */
    namespace imageprocessing {
		class Connector;
		class Flow;
		class Motion;

		namespace imgproc_helpers {
			class Helpers;
		}
    }
}


// ***************************************************************************
// ********    namespace thesis::imageprocessing::imgproc_helpers     ********
// ***************************************************************************

/**
 *  Class Helpers: definition 
 *
 * ATTENTION is another namespace
 *
 */
class thesis::imageprocessing::imgproc_helpers::Helpers {
public:
	Helpers();
	~Helpers();
	bool withinBorders(int _i, int _j, int _w, int _h);
	bool withinBorders(iPoint2D _pos, iPoint2D _size);
	PixelRgb brighterPixel(PixelRgb _p1, PixelRgb _p2);
	PixelRgb threshold(PixelRgb _p1, int _threshold);
	PixelRgb binThreshold(PixelRgb _p1, int _threshold);
	PixelMono binThreshold(PixelMono _p1, int _threshold);

};


// ***************************************************************************
// ***************      namespace thesis::imageprocessing      ***************
// ***************************************************************************

/**
 *  Class Motion: definition
 */
class thesis::imageprocessing::Motion {
private:
	// Declaration of a new type: for recording
	// Declaration of hidden class variables

	int		motion_threshold;
	double	motion_percentage;
	double	optFlow_trigger;
	thesis::tools::Decider *decisionModule_ptr;

	// Declaration of hidden methods

public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	Motion (thesis::tools::Decider *_dModule_ptr, int _motionth, double _flowtrigger); 
	~Motion ();

	void	detectMotion(ImageOf<PixelMono> *_img_ptr, ImageOf<PixelMono> *_img_next_ptr);
	bool	reset();
};

// ***************************************************************************

/**
 *  Class Flow: definition
 */
class thesis::imageprocessing::Flow {
protected:
	// Declaration of hidden class variables (native)
	bool				ready;
	int					rows, cols, counter, nofFeatures, angularRange; //frame_width, frame_height;
	double				patch_percentage, trackingTrigger;

	patchInfo			markedPatches;

	dPoint2D			**optFlow_ptr;
	thesis::tools::Decider *deciderModule_ptr;

	// opencv declaration
	IplImage			*cvThisImageGRAY_ptr;
	IplImage			*cvNextImageGRAY_ptr;

	CvPoint2D32f		*cvThisImage_features_ptr;
	CvPoint2D32f		*cvNextImage_features_ptr;
	CvPoint2D32f		*tmp_ptr;
	char				*optFlow_found_features_ptr;

	// Declaration of hidden  methods
public:
	// Declaration of class variables
	// Declaration of Constructor, Destructor and methods
	//Flow (ImageOf<PixelRgb> *_img_ptr, thesis::tools::Decider *_dModule_ptr);
	Flow (ImageOf<PixelMono> *_img_ptr, thesis::tools::Decider *_dModule_ptr, double _trackingtrigger, int _nofFeatures, int _angularRange);
	~Flow();

	double				getRelevantPatchPercentage();
	int					neighbours8(iPoint2D _pos, int _criterion);
	bool				isReady();
	void				classifyFlow();
	void				patchTheFlow();
	void				updateFlow(ImageOf<PixelMono> *_thisImg_ptr, ImageOf<PixelMono> *_nextImg_ptr);
	void				processFlow(ImageOf<PixelMono> *_thisImg_ptr, ImageOf<PixelMono> *_nextImg_ptr);
	void				getOpticalFlow(ImageOf<PixelMono> *_thisImg_ptr, ImageOf<PixelMono> *_nextImg_ptr);

	//ImageOf<PixelRgb>*	getPatchedOpticalFlowAsImage(ImageOf<PixelRgb> *_img_ptr);
	//ImageOf<PixelRgb>*	getPatchedOpticalFlowAsImage(ImageOf<PixelMono> *_img_ptr);
	//ImageOf<PixelRgb>*	getOpticalFlowAsImage();

	bool				reset();
	bool				paramChange(double _trackingtrigger, int _nofFeatures, int _angularRange);
	const char*			getError();
	CvPoint2D32f*		getThisFeatures();
	CvPoint2D32f*		getNextFeatures();
	patchInfo*			getRelevantPatches();

};

// ***************************************************************************

/**
 *  Class Connector: definition 
 *
 */
class thesis::imageprocessing::Connector {
private:
	// Declaration of hidden class variables
	// Declaration of hidden  methods
public:
	// Declaration of class variables
	Motion	*motion_ptr;
	Flow	*flow_ptr;
	// Declaration of Constructor, Destructor and methods
	Connector(Motion *_motionModule_ptr, Flow *_opticalFlowModule_ptr);
	~Connector();
};


#endif
