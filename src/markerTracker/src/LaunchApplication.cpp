// own includes
#include <LaunchApplication.h>
#include <Tracking.h>

#include "FitEllipse.h"

//namespaces

using namespace halode::IO;
using namespace halode::app;
using namespace halode::gui;
using namespace halode::files;
using namespace halode::tracking;

using std::vector;

// ***************************************************************************

/**
 * Implementation of the Launchapplication module
 *
 */

// *******************************************
//          variable declarations
// *******************************************

LaunchApplication::LaunchApplication (ImgOutput **_imageOutput_ptr, ImgInput **_imageInput_ptr, Parameters *_params_ptr, DataOutput **_dataOutput_ptr) {

	printf("Start:\t[Launch]\n");

	this->die					= false;
	// ports used
	this->tv_result				= _imageOutput_ptr[0];
	this->tv_alternative		= _imageOutput_ptr[1];
	
	this->imageInput1			= _imageInput_ptr[0];
	if (_params_ptr->stereovision) {
		this->imageInput2		= _imageInput_ptr[1];
	}
	else {
		this->imageInput2		= _imageInput_ptr[0];
	}

	this->storeImgsStats		= NULL;

	// imgPointers
	this->thisImgRgb1_ptr		= new ImageOf<PixelRgb>;
	this->thisImgRgb2_ptr		= new ImageOf<PixelRgb>;

	this->params_ptr			= _params_ptr;

	this->dataOutputLeft_ptr	= _dataOutput_ptr[0];
	this->dataOutputRight_ptr	= _dataOutput_ptr[1];
}

LaunchApplication::~LaunchApplication () {
	printf("Quit:\t[Launch]\n");
}

bool LaunchApplication::launch() {

	printf("Info:\t[Launch]\t{starting}\n");
	this->thisImgRgb1_ptr->copy(*this->imageInput1->readImg());

	if (this->params_ptr->stereovision) {
		this->thisImgRgb2_ptr->copy(*this->imageInput2->readImg());
	}

	int				i = 0;
	bool			cont = true;

	ImgOutputFile	imgOut1		(this->params_ptr->result_dir_1);
	ImgOutputFile	imgOut2		(this->params_ptr->result_dir_2);

	HandTracker		htmLeft		(this->params_ptr);
	HandTracker		htmRight	(this->params_ptr);

	Bottle			posBottleLeft, posBottleRight;

	Gui				g;
	iPoint2D		leftpos, rightpos;
	FitEllipse		leftellipse, rightEllipse;

	while (!die) {
		posBottleLeft.clear();
		posBottleRight.clear();

		this->thisImgRgb1_ptr->copy(*this->imageInput1->readImg());
		if (this->params_ptr->stereovision) {
			this->thisImgRgb2_ptr->copy(*this->imageInput2->readImg());
		}


		// -------------------------------------------------------------
		// if you need only a data dump, this part will be recording
		if (this->params_ptr->rec_perception) {
			char tmp1 [16];
			char tmp2 [16];
			
			sprintf(tmp1, "left_eye_%d", i);
			sprintf(tmp2, "right_eye_%d", i);
			imgOut1.write(this->thisImgRgb1_ptr, tmp1);
			if (this->params_ptr->stereovision) {
				imgOut2.write(this->thisImgRgb2_ptr, tmp2);
				//this->tv_alternative->write(this->thisImgRgb2_ptr);
			}
			//this->tv_result->write(this->thisImgRgb1_ptr);

		}
		// -------------------------------------------------------------
		else {
			// ***********************************************
			// left eye
			// ***********************************************
			leftpos = htmLeft.searchContrasts(this->thisImgRgb1_ptr, "rg");
			
			posBottleLeft.addDouble(2*(((double)leftpos.x)/((double)this->params_ptr->img_width) - 0.5));
			posBottleLeft.addDouble(2*(((double)leftpos.y)/((double)this->params_ptr->img_height) - 0.5));
			posBottleLeft.addDouble(0.0);



			// bounding box around the marker (adapts to the size)
			// bgrange stores the min and the max values of the height and the width.
			// col = width, row = height
			bgRange roi = *htmLeft.getROI();
			addRectangleOutline(*this->thisImgRgb1_ptr, g.c.azzuro, leftpos.x, leftpos.y, (int)((roi.col.y-roi.col.x)), (int)((roi.row.y-roi.row.x)));
			addCrossHair(*this->thisImgRgb1_ptr, g.c.azzuro, leftpos.x, leftpos.y, 5);
			addCircleOutline(*this->thisImgRgb1_ptr, g.c.azzuro, leftpos.x, leftpos.y, 3);

			// sending the position of the tracker out to the specified port
			this->dataOutputLeft_ptr->write(&posBottleLeft);
			
			// ***********************************************
			// right eye
			// ***********************************************
			if (this->params_ptr->stereovision) {
				rightpos = htmRight.searchContrasts(this->thisImgRgb2_ptr, "rg");
				posBottleRight.addDouble(2*(((double)rightpos.x)/((double)this->params_ptr->img_width) - 0.5));
				posBottleRight.addDouble(2*(((double)rightpos.y)/((double)this->params_ptr->img_height) - 0.5));
				posBottleRight.addDouble(0.0);
						
				// see bounding box left eye (up)
				bgRange roi = *htmRight.getROI();
				addRectangleOutline(*this->thisImgRgb2_ptr, g.c.azzuro, rightpos.x, rightpos.y, (int)((roi.col.y-roi.col.x)), (int)((roi.row.y-roi.row.x)));

				addCrossHair(*this->thisImgRgb2_ptr, g.c.azzuro, rightpos.x, rightpos.y, 5);
				addCircleOutline(*this->thisImgRgb2_ptr, g.c.azzuro, rightpos.x, rightpos.y, 3);
				
				// sending the position of the tracker out to the specified port
				this->dataOutputRight_ptr->write(&posBottleRight);

			}

			// ***********************************************
			// display
			// ***********************************************
			if (this->params_ptr->display) {
				this->tv_result->write(this->thisImgRgb1_ptr);
				//this->tv_result->write(htmLeft.trackImg_ptr);
				if (this->params_ptr->stereovision) {
					this->tv_alternative->write(this->thisImgRgb2_ptr);
					//this->tv_alternative->write(htmRight.trackImg_ptr);
				}
				else {
					this->tv_alternative->write(this->thisImgRgb1_ptr);
				}
			}
		}
		i++;
	}
	return true;
}

/**
 * Simply what the name tells you...
 *
 */
void LaunchApplication::deAllocation() {
	
	printf("deallocation\n");
	
}


string LaunchApplication::getFileName(string _fn, int _nr) {
	string fn = "";
	const int numlen = 5;
	char numb[numlen];

	fn.append(_fn);
	if (_nr == 0) {
		for (int i=0; i<(numlen-1); i++) {
			fn.append("0");
		}
	}
	else if (_nr > 0 ){
		int res = _nr*10;
		while (res < pow(10, 5)) {
			fn.append("0");
			res = res * 10;
		}
	}
	sprintf(numb,"%d",_nr);

	fn.append(numb);
	return fn;
}




