// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _CREFINEMENTMODULE_H_
#define _CREFINEMENTMODULE_H_

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//primatevision include
#include <multiclass.h> 
#include <iCub/convert_rgb.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace iCub::contrib::primateVision;

/*namespace iCub {
  namespace contrib {
    namespace primateVision {
		namespace MultiClass
	}
  }
};*/

/**
  * Module refines the solution after processing based on the contest
  * function1: given the input image and the probability map, extracts the whole object with max probability
  * function2: .......
  * The module accepts options typed by commandline
  * -option1: mode; possible values: MRF
  * If no option is typed by command line, the module works in DEFAULT mode
  * @author Francesco Rea
 */


class cRefinementModule : public Module {
private:
	/**
	* a port that takes as input the input image
	*/
    BufferedPort<ImageOf<PixelRgb> > portInImage; // 
	/**
	* port that takes as input the probability map
	*/
	BufferedPort<ImageOf<PixelMono> > portInProbClassA;
	/**
	* port that takes as input the probability map
	*/
	BufferedPort<ImageOf<PixelMono> > portInProbOther;//
	/**
	* port that gives the output of the module
	*/
	BufferedPort<ImageOf<PixelMono> > portOut; //
	/**
	* reference to the multiclassifier
	*/
	MultiClass* m;
	/**
	* edge map reference
	*/
	Ipp8u *edge_map;
	/**
	* input image
	*/
	Ipp8u * in;
	/**
	* converter to RGB
	*/
	Convert_RGB* ci;
	/**
	* port for various command
	*/
    Port cmdPort;
	/**
	* exexution step counter
	*/
    int ct;
	/**
	*yarp returned image
	*/
	ImageOf<PixelRgb> yarpReturnImage;  
	/**
	* YARP pointer to the output image
	*/
	ImageOf<PixelRgb> *yarpReturnImagePointer;
	/**
	* destination color image of the outPort
	*/
	IplImage* dstColor;   //
	/**
	* destination color image of the invese outPort
	*/
	IplImage* dstColor2;
	CvRect rec;
	/**
	* input image (always 320,240)
	*/
	ImageOf<PixelRgb> *imgInput; 
	/**
	* output image (always 320,240)
	*/
	ImageOf<PixelMono> *imgOut; 
	/**
	* probability map image (always 320,240)
	*/
	ImageOf<PixelMono> *imgProbClassA; 
	/**
	* probability map image Other (always 320,240)
	*/
	ImageOf<PixelMono> *imgProbOther;
	/**
	* the image that represent everything not of class A
	*/
	ImageOf<PixelMono> *imageOther;
	/**
	* the image that represent the class A
	*/
	ImageOf<PixelMono> *imageClassA;
	/**
	* size of the output
	*/
	IppiSize isize;
	/**
	* options of the module deteched from command line
	*/
	yarp::os::Property options;
	/**
	* series of parameters for the multiclass object
	*/
	MultiClass::Parameters properties;
	/**
	* mode of work of the module
	*/
	 int mode;
public:
	/**
	*opens the port and intialise the module
	* @param config configuration of the module
	*/
	bool open(Searchable& config); 
	/**
	* tries to interrupt any communications or resource usage
	*/
    bool interruptModule(); // 
	/**
	* function that set the options detected from the command line
	* @param opt options passed to the module
	*/
	void setOptions(yarp::os::Property opt);
	/**
	* function that set the properties for Classifier
	* @param prop property of the class
	*/
	void setProperties(MultiClass::Parameters prop);
	/**
	* catches all the commands that have to be executed when the module is closed
	*/
    bool close(); 
	/**
	* updates the module
	*/
	bool updateModule();

	/**
	* return the 255 complementary image of the input
	*/
	ImageOf<PixelMono>* inverse(ImageOf<PixelMono>* input);
};


#endif
