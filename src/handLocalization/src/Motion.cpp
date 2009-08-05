// include header
#include <ImageProcessing.h>

// namespaces
using namespace thesis::tools;
using namespace thesis::imageprocessing;

// ***************************************************************************

/**
 * Implementation of Motion class
 *
 * This "module" is working with image difference and gets a mask of the moving objects.
 */

// Constructor
Motion::Motion(Decider *_dModule_ptr, int _motionth, double _flowtrigger) { 
	//printf("Start:\t[Motion]\n");

	// initializing the variables
	this->motion_percentage		= 0.0;
	this->motion_threshold		= _motionth; 
	this->optFlow_trigger		= _flowtrigger;
	this->decisionModule_ptr	= _dModule_ptr;

}

// Destructor
Motion::~Motion() {
	//printf("Quit:\t[Motion]\n");
}

bool Motion::reset() {
	this->motion_percentage = 0.0;
	return true;
}

/**
 *
 * This method computes all the motion in the current image, in fact it is computing the image difference
 *
*/
void Motion::detectMotion(ImageOf<PixelMono> *_img_ptr, ImageOf<PixelMono> *_img_next_ptr) {

	// declarations
	PixelMono	pix_diff;
	// initialization
	int count_motion = 0;
	// looking at each pixel
	for (int j=0; j<_img_ptr->height(); j++ ) {
		for (int i=0; i<_img_ptr->width(); i++) {
			pix_diff = abs((int)_img_next_ptr->pixel(i,j)-(int)_img_ptr->pixel(i,j));
			// if the gray value difference is smaller than a threshold the pixel is assumed as not important moving
			if (pix_diff >= this->motion_threshold) {
				count_motion ++;
			}
			// this is pixel that has another value, assumed to be moving...
		} // end for i (columns)
	} // end for j (rows)

	this->motion_percentage = ((double)count_motion / (double)(_img_ptr->height() * _img_ptr->width()));
	printf("%lf\n", this->motion_percentage);
	if (this->motion_percentage > this->optFlow_trigger) {
		this->decisionModule_ptr->setTrigger("flow");
	}
}
