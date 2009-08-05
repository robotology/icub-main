#ifndef _HALOAPP_
#define _HALOAPP_

#include <includes.h>

#include <IO.h>
#include <Gui.h>
#include <LaunchApplication.h>

// namespaces 
namespace halode {
    namespace app {
		class LaunchApplication;
    }
}

// ***************************************************************************
// ********                  namespace halode::tools                  ********
// ***************************************************************************

/**
 *  Class LaunchApplication: definition 
 *
 */
class halode::app::LaunchApplication {

private:
	// Declaration of hidden class variables
	Parameters					*params_ptr;

	//ports
	halode::IO::ImgInput		*imageInput1;
	halode::IO::ImgInput		*imageInput2;

	halode::IO::ImgOutput		*tv_result;
	halode::IO::ImgOutput		*tv_alternative;

	//halode::IO::DataOutput		*storeDataStats;
	halode::IO::ImgOutput		*storeImgsStats;
	halode::IO::ImgOutput		*storeImgsRslts;

	// images
	ImageOf<PixelMono> tmpImageMono;

	// imgPointers
	ImageOf<PixelMono>	*thisImgMono_ptr;
	ImageOf<PixelMono>	*nextImgMono_ptr;
	//ImageOf<PixelMono>	*tmpImg_ptr;

	ImageOf<PixelRgb>	*thisImgRgb1_ptr;
	ImageOf<PixelRgb>	*thisImgRgb2_ptr;

	//Bottle				*thisData_ptr;

	// Declaration of hidden  methods
	void deAllocation();
	string getFileName(string _fn, int _run);



public:
	LaunchApplication(halode::IO::ImgOutput **output_ptr, halode::IO::ImgInput **_imageInput, Parameters *_params_ptr);
	~LaunchApplication();
	bool launch();

};

#endif
