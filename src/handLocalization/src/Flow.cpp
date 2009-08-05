// include header
//#include <thesis/includes.h>
#include <yarp/os/Time.h>

#include <Gui.h>
#include <Mathematics.h>
#include <ImageProcessing.h>
//#include <algorithm>
//#include <list>

// namespaces
using namespace thesis::gui;
using namespace thesis::tools;
using namespace thesis::mathematics;
using namespace thesis::imageprocessing;
using namespace thesis::imageprocessing::imgproc_helpers;

// ***************************************************************************

/**
 * Implementation of Flow class
 *
 * This "module" is working on the images within a timeperiod and save the motion information to 
 * acquire information about the moving objects in the frames
 *
 */


// Init constructor
Flow::Flow(ImageOf<PixelMono> *_img_ptr, Decider *_dModule_ptr, double _trackingtrigger, int _nofFeatures, int _angularRange) {
	//printf("Start:\t[Flow]\n");
	
	// Declarations & Initializing
	dPoint2D	initFlowDirection;

	this->ready						= false;
	this->cols						= PATCHED(_img_ptr->width());
    this->rows						= PATCHED(_img_ptr->height());
	this->deciderModule_ptr			= _dModule_ptr;
	this->trackingTrigger			= _trackingtrigger;
	this->patch_percentage			= 0.0;
	this->nofFeatures				= _nofFeatures;
	this->angularRange				= _angularRange;

	this->cvThisImage_features		= new CvPoint2D32f[_nofFeatures];
	this->cvNextImage_features		= new CvPoint2D32f[_nofFeatures];
	this->optFlow_found_features	= new char[_nofFeatures];

	initFlowDirection.x = 0.0;
	initFlowDirection.y = 0.0;

	this->optFlow_ptr	= new dPoint2D* [this->rows];
	for (int j=0; j < this->rows; j++) {
		this->optFlow_ptr[j] = new dPoint2D[this->cols];
		for (int i=0; i< this->cols; i++) {
			this->optFlow_ptr[j][i] = initFlowDirection;
		}
	}

}

Flow::~Flow() {
	
	for (int j = 0; j<this->rows; j++) {
		delete [] this->optFlow_ptr[j];
	}
	delete [] this->optFlow_ptr;
	delete [] this->optFlow_found_features;
	delete [] this->cvNextImage_features;
	delete [] this->cvThisImage_features;

	//printf("Quit:\t[Flow]\n");
}

bool Flow::reset() {
	dPoint2D	initFlowDirection;

	initFlowDirection.x = 0.0;
	initFlowDirection.y = 0.0;
	this->patch_percentage = 0.0;

	this->ready = false;
	for (int j=0; j < this->rows; j++) {
		this->optFlow_ptr[j] = new dPoint2D[this->cols];
		for (int i=0; i< this->cols; i++) {
			this->optFlow_ptr[j][i] = initFlowDirection;
		}
	}
	this->markedPatches.patch_id.x.clear();
	this->markedPatches.patch_id.y.clear();
	this->markedPatches.vector_value.x.clear();
	this->markedPatches.vector_value.y.clear();
	return true;
}

bool Flow::paramChange(double _trackingtrigger, int _nofFeatures, int _angularRange) {
	this->reset();

	this->angularRange				= _angularRange;
	this->trackingTrigger			= _trackingtrigger;
	this->nofFeatures				= _nofFeatures;

	delete [] this->optFlow_found_features;
	delete [] this->cvNextImage_features;
	delete [] this->cvThisImage_features;

	this->cvThisImage_features		= new CvPoint2D32f[_nofFeatures];
	this->cvNextImage_features		= new CvPoint2D32f[_nofFeatures];
	this->optFlow_found_features	= new char[_nofFeatures];


	return true;
}

bool Flow::isReady() {
	return this->ready;
}


void Flow::updateFlow(ImageOf<PixelMono> *_thisImg_ptr, ImageOf<PixelMono> *_nextImg_ptr) {
	if (!this->ready) {
		if (this->deciderModule_ptr->decide("flow")) {
			this->getOpticalFlow(_thisImg_ptr, _nextImg_ptr);
			this->patchTheFlow();
			//printf("update triggered\n");
			this->classifyFlow();
			//this->ready = true;
		} // end trigger if
	} else {
		this->getOpticalFlow(_thisImg_ptr, _nextImg_ptr);
	}
}

void Flow::getOpticalFlow(ImageOf<PixelMono> *_thisImg_ptr, ImageOf<PixelMono> *_nextImg_ptr) {
	
		// declaration and initialization
		//int				nofFeatures;

		//CvSize			win_OFlow;
		CvTermCriteria	oFlowTermCrit;

		//IplImage		*cvImage_ptr			= NULL;
		IplImage		*eigImage				= NULL;
		IplImage		*tmpImage				= NULL;
		IplImage		*thisPyramid			= NULL;
		IplImage		*nextPyramid			= NULL;

		float			*optFlow_feat_error = new float [this->nofFeatures];

		//win_OFlow		= cvSize(WIN_SIZE_OPTFLOW,WIN_SIZE_OPTFLOW);
		oFlowTermCrit	= cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3);

		this->cvThisImageGRAY_ptr = (IplImage*) _thisImg_ptr->getIplImage();
		this->cvNextImageGRAY_ptr = (IplImage*) _nextImg_ptr->getIplImage();

		// Good feature to track
		cvGoodFeaturesToTrack(this->cvThisImageGRAY_ptr, eigImage, tmpImage, this->cvThisImage_features, &this->nofFeatures, 0.01, 0.01, NULL);
		// Computing optical flow
		cvCalcOpticalFlowPyrLK(this->cvThisImageGRAY_ptr, this->cvNextImageGRAY_ptr, thisPyramid, nextPyramid, this->cvThisImage_features, this->cvNextImage_features, this->nofFeatures, cvSize(WIN_SIZE_OPTFLOW,WIN_SIZE_OPTFLOW), 5, this->optFlow_found_features, optFlow_feat_error, oFlowTermCrit, 0 );

		delete [] optFlow_feat_error;

}

/**
 *
 * This method places the values of every feature point into the corresponding patch of the image
 * updating the Flow with the values
 *
 **/
 void Flow::patchTheFlow() { 
	int			w, h;
	double		length, angle;
	dPoint2D	q, p, diff;

	Geometry	m;
	Statistics	stats;

	// the standard deviation formula needs all values, here they are stored within a Bottle for each patch.
	//Bottle		**values_for_stand_dev_ptr = new Bottle* [this->rows];
	dBottle2D **values_after_stand_dev_ptr = new dBottle2D* [this->rows];
    int i=0;
    int k=0;
	for (i = 0; i < this->rows; i++) {
		values_after_stand_dev_ptr[i] = new dBottle2D [this->cols];
		for (k= 0; k<this->cols;k++) {
			values_after_stand_dev_ptr[i][k].x.clear();
			values_after_stand_dev_ptr[i][k].y.clear();
		}
	}
	
	// for all features of the optical flow, do:
    int f=0;
	for (f = 0; f < this->nofFeatures; f++ ) {

		// if the feature has a value (no error = 0), do:
		if (this->optFlow_found_features[f] == 0) continue;
		if (this->cvThisImage_features[f].x != 0.0) {
			// the initial position of the flow
			p.x = this->cvThisImage_features[f].x;
			p.y = this->cvThisImage_features[f].y;

			// the flow size, and direction
			q.x = this->cvNextImage_features[f].x;
			q.y = this->cvNextImage_features[f].y;

			// the coordinates of the patch where to store the feature's optical flow
			w = PATCHED((double)p.x);
			h = PATCHED((double)p.y);
			w = MIN(w, (this->cols - 1));
			h = MIN(h, (this->rows - 1));

			diff.x = q.x - (double)p.x;
			diff.y = q.y - (double)p.y;

			// add the angular value of the flow into the bottle
			angle = (double)m.getAngle(diff);
			length = sqrt(diff.x*diff.x + diff.y*diff.y);
			if (length > 0.0) {
				values_after_stand_dev_ptr[h][w].x.addDouble(diff.x/length);
				values_after_stand_dev_ptr[h][w].y.addDouble(diff.y/length);
			}
		} // 

	} // end for f (features)

	// in order to compute the standard deviation travers another time the features. the average value is now known.
	//printf("correction:\n");
	int bottlesize = 0;
	// traverse the values of how many times the patch has been updated and "normalize" them
	// do also store the angular modulos for estimating the number of clusters
	for (int j = 0; j<this->rows; j++) {
		for (int i = 0; i<this->cols; i++) {
			bottlesize = values_after_stand_dev_ptr[j][i].x.size();
			if (bottlesize != values_after_stand_dev_ptr[j][i].y.size()) {
				printf("Fatal error in update Flow\n");
				exit(-99);
			}
			if ( bottlesize > PATCH_SIZE*0.25) {
				// compute the 90% confidence interval of normal distribution to sort out the outliers
				int  count;
				dPoint2D conf_interval_x, conf_interval_y;
				dPoint2D val, vector_sum;

				count = 0;
				vector_sum.x = 0.0;
				vector_sum.y = 0.0;

				// now everthing is known. So the features have to be processed again! To kick out the unwanted values.
				conf_interval_x = stats.confidenceIntervalND(&values_after_stand_dev_ptr[j][i].x, 90);
				conf_interval_y = stats.confidenceIntervalND(&values_after_stand_dev_ptr[j][i].y, 90);

				// only take into account the values that are within the confidence interval
				for (int b = 0; b < bottlesize; b++) {
					val.x = values_after_stand_dev_ptr[j][i].x.get(b).asDouble();
					val.y = values_after_stand_dev_ptr[j][i].y.get(b).asDouble();

					if (((val.x >= conf_interval_x.x) & (val.x <= conf_interval_x.y)) & ((val.y >= conf_interval_y.x) & (val.y <= conf_interval_y.y))) {
						vector_sum.x = vector_sum.x + val.x;
						vector_sum.y = vector_sum.y + val.y;
						count++;
					}
				} // end for b

				if (count > 0) {
					this->optFlow_ptr[j][i] = vector_sum;
				}
			} // end if bottle is not empty
			else {
				this->optFlow_ptr[j][i].x = 0.0;
				this->optFlow_ptr[j][i].y = 0.0;
			}
		} // end for i
	} // end for j
	// free the used temp memory
	for (i = 0; i < this->rows; i++) {
		delete [] values_after_stand_dev_ptr[i];
	}
	delete [] values_after_stand_dev_ptr;
	printf("patch the flow\n");
}



int Flow::neighbours8(iPoint2D _pos, int _criterion) {
	Helpers h;
	Geometry m;
	int neighbours = 0;
	iPoint2D n_tmp, size;
	dPoint2D value;
	int pos_angle, neighb_angle;
	size.x = this->cols;
	size.y = this->rows;
	// test whether the position is within the range
	if (h.withinBorders(_pos, size)) {
		pos_angle = m.getAngle(this->optFlow_ptr[_pos.y][_pos.x]);
		for (int j = -1; j < 2; j++) {
			n_tmp.y = _pos.y + j;
			for (int i = -1; i < 2; i++) {
				n_tmp.x = _pos.x + i;
				if ((n_tmp.y != _pos.y) | (n_tmp.x != _pos.x)) {
					if (h.withinBorders(n_tmp, size)) {
						value = this->optFlow_ptr[n_tmp.y][n_tmp.x];
						neighb_angle = m.getAngle(value);
						if ((neighb_angle > 0) & (abs(neighb_angle - pos_angle) <= _criterion)){
							neighbours++;
						}
					}
				} // end else (centre)
			} // end for i (columns)
		} // end for j (rows)
	}
	return neighbours;
}

void Flow::classifyFlow() {

	Geometry m;
	iPoint2D n0;
	dPoint2D n0_val;
	
	int n					= 0;
	int sumPassedPatches	= 0;
	this->patch_percentage = 0.0;

	//printf("mark equivalent flows: %d\n", this->counter-1);
	this->markedPatches.patch_id.x.clear();
	this->markedPatches.patch_id.y.clear();
	this->markedPatches.vector_value.x.clear();
	this->markedPatches.vector_value.y.clear();
	for (int j=0; j<this->rows; j++) {
		n0.y = j;
		for (int i=0; i<this->cols; i++) {
			n0.x = i;
			n0_val = this->optFlow_ptr[j][i];

			if (m.getAngle(n0_val) > 0) {
				n = this->neighbours8(n0, this->angularRange);
				if (n > 1) {
					this->markedPatches.patch_id.x.addInt(i);
					this->markedPatches.patch_id.y.addInt(j);
					this->markedPatches.vector_value.x.addDouble(n0_val.x);
					this->markedPatches.vector_value.y.addDouble(n0_val.y);
					sumPassedPatches++;
				}
			} 
		} // end for i
	} // end for j
	this->patch_percentage = ( (double)sumPassedPatches/ ( this->rows*this->cols));
	printf("percentage: %f\n", this->patch_percentage);
	if (this->patch_percentage > this->trackingTrigger) {
		this->deciderModule_ptr->setTrigger("tracking");
		this->ready = true;
	}
}

CvPoint2D32f* Flow::getThisFeatures() {
	return this->cvThisImage_features;
}

CvPoint2D32f* Flow::getNextFeatures() {
	return this->cvNextImage_features;
}

const char* Flow::getError() {
	return this->optFlow_found_features;
}

patchInfo* Flow::getRelevantPatches() {
	return &(this->markedPatches);
}

double Flow::getRelevantPatchPercentage() {
	return (this->patch_percentage);
}

