// own includes
#include <LaunchApplication.h>

//namespaces

using namespace halode::IO;
using namespace halode::app;
using namespace halode::gui;
using namespace halode::files;

// ***************************************************************************

/**
 * Implementation of the Launchapplication module
 *
 */

// *******************************************
//          variable declarations
// *******************************************

LaunchApplication::LaunchApplication (ImgOutput **_imageOutput_ptr, ImgInput **_imageInput_ptr, Parameters *_params_ptr) {

	printf("Start:\t[Launch]\n");
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

	int				max = 400;
	Gui				g;
	while (i<max) {

		// grabbing the images!!
		this->thisImgRgb1_ptr->copy(*this->imageInput1->readImg());

		// if stereovision is used
		if (this->params_ptr->stereovision) {
			this->thisImgRgb2_ptr->copy(*this->imageInput2->readImg());
			// just adding a circle to the image
			addCircleOutline(*(this->thisImgRgb1_ptr), g.c.darkred, (int)(this->params_ptr->img_width/2.0), 
				(int)(this->params_ptr->img_height/2.0), 5);
		}

		// display the output in a yarpviewer image. (actually sending the images to a port)
		if (this->params_ptr->display) {
			this->tv_result->write(this->thisImgRgb1_ptr);
			if (this->params_ptr->stereovision) {
				this->tv_alternative->write(this->thisImgRgb2_ptr);
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


/*string LaunchApplication::getFileName(string _fn, int _nr) {
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



*/
