#ifndef _TRACKING_
#define _TRACKING_

#include <includes.h>
#include <IO.h>
#include <Gui.h>
//#include <ObjectTracker.h>

#include <cv.h>
#include <vector>
// namespaces 
namespace halode {
    /**
     * Tracking of the predetermined patches, later processing the motor sensory information and "decisionmaking".
     */
    namespace tracking {
		class HandTracker;
    }
}

/**
 *  Class HandTracker: definition
 */
class halode::tracking::HandTracker {
protected:
	// Declaration of hidden class variables for the storage part
	Parameters			*param_ptr;

public:
	ImageOf<PixelRgb>	*trackImg_ptr;
	std::vector<float>	xEllipse;
	std::vector<float>	yEllipse;

	bgRange				roi;
	HandTracker	(Parameters *_params_ptr);
	~HandTracker();

	bool		isRed(PixelRgb *_p1_ptr);
	bool		isBlue(PixelRgb *_p1_ptr);
	bool		isYellow(PixelRgb *_p1_ptr);
	bool		transition(PixelRgb *_p1_ptr, PixelRgb *_p2_ptr, string _fromTo);

	int			searchObject(ImageOf<PixelRgb> *_img_ptr);

	void		trackObject(ImageOf<PixelRgb> *_img_ptr);

	iPoint2D	searchContrasts(ImageOf<PixelRgb> *_img_ptr, string _colors);

	bgRange*	getROI();
};

#endif
