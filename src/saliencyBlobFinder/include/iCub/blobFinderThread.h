// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _BLOBFINDERTHREAD_H_
#define _BLOBFINDERTHREAD_H_

//within project includes
#include <iCub/WatershedOperator.h>
#include <iCub/SalienceOperator.h>

//IPP include
#include <ippi.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;


class blobFinderThread : public RateThread{
private:
    /**
    * port where the input image is read from
    */
    BufferedPort<ImageOf<PixelRgb> > inputPort;
    /**
    * port where the red plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > redPort;
    /**
    * port where the green plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > greenPort;
    /**
    * port where the blue plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > bluePort;
    /**
    * port where the difference of gaussian R+G- is streamed
    */
    BufferedPort<ImageOf<PixelMono> > rgPort;
    /**
    * port where the difference of gaussian G+R- is streamed
    */
    BufferedPort<ImageOf<PixelMono> > grPort;
    /**
    * port where the difference of gaussian B+Y- of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > byPort;
    /**
    * port where the yellow plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > yellowPort;
    
    /**
	* image result of the function outContrastLP
	*/
	ImageOf<PixelMono> *outContrastLP;
    
	/**
	* image result of the function meanColourLP;
	*/
	ImageOf<PixelBgr> *outMeanColourLP;
    /**
    * ipp reference to the size of the input image
    */
    IppiSize srcsize;
    /**
    * width of the input image
    */
    int width;
    /**
    * height of the input image
    */
    int height;
    /**
	* maxBLOB dimension
	*/
	int maxBLOB;
	/**
	* minBLOB dimension
	*/
	int minBLOB;
    
    /**
    * flag that indicates when the reinitiazation has already be done
    */
    bool reinit_flag;
    
    /**
     * semaphore for the respond function
     */
     Semaphore mutex;
     /**
    * execution step counter
    */
    int ct;
    /**
    * input image
    */
    ImageOf<PixelRgb> *img;
    /**
	* reference to the watershed operator
	*/
	WatershedOperator *wOperator;
	/**
	* reference to the salience operator
	*/
	SalienceOperator *salience;
    /**
	* number of blobs
	*/
	int max_tag;
    /**
	* vector of boolean which tells whether there is a blob or not
	*/
	char* blobList;
    
    
    /**
    * pointer to the 3 channels output image of the watershed algorithm
    */
    ImageOf<PixelRgb>* _outputImage3;
    
    
    /**
	* input image of the opponency R+G-
	*/
	ImageOf<PixelMono> *_inputImgRGS;
	/**
	* input image of the opponency G+R-
	*/
	ImageOf<PixelMono> *_inputImgGRS;
	/**
	* input image of the opponency B+Y-
	*/
	ImageOf<PixelMono> *_inputImgBYS;
    /**
    * pointer to the image of tags
    */
    ImageOf<PixelInt> *ptr_tagged;    
    /**
	* pointer to the input image of the red plane
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputRed; //
	/**
	* pointer to the input image of the green plane
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputGreen; //
	/**
	* pointer to the input image of the blue plane
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputBlue; //
	/**
	* pointer to the input image of the R+G- colour opponency
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputRG;  //
	/**
	* pointer to the input image of the G+R- colour opponency
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputGR;  //
	/**
	* pointer to the input image of the B+Y- colour opponency
	*/
	yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputBY; //
    /**
	*vector of tags to the sequence of blobs
	*/
	ImageOf<PixelInt>* tagged;
    /**
	* image of the fovea blob
	*/
	ImageOf<PixelMono> *blobFov;
    /**
	* saliencyTOT linear combination Ktd coefficient (TOP DOWN saliency weight)
	*/
	double salienceTD;
	/**
	* saliencyTOT linear combination Kbu coefficient (BOTTOM-UP saliency weight)
	*/
	double salienceBU;
    /**
	* red intensity of the target that has been found 
	*/
	double targetRED;
	/**
	* green intensity of the target that has been found 
	*/
	double targetGREEN;
	/**
	* blue intensity of the target that has been found 
	*/
	double targetBLUE;
    /**
	* R+G- value for the search
	*/
	int searchRG;
	/**
	* G+R- value for the search
	*/
	int searchGR;
	/**
	* B+Y- value for the search
	*/
	int searchBY;
    /**
	* flag that indicates if the images have been resized
	*/
	bool resized_flag;

    //_________ private methods ____________
    /**
    * resizes all the needed images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resizeImages(int width, int height);
    /**
    * function that extracts characteristics of all the blobs in the catalogue and save them
    * @param stable parameters that enable some lines of code for the stable version
    */
    void drawAllBlobs(bool stable);    
public:
    /**
    * default constructor
    */
    blobFinderThread();
    /**
    * destructor
    */
    ~blobFinderThread(){};
    /**
    *	initialization of the thread 
    */
    bool threadInit();
    /**
    * active loop of the thread
    */
    void run();
    /**
    *	releases the thread
    */
    void threadRelease();
    /**
    * function that reinitiases some attributes of the class
    * @param height height of the input image
    * @param width width of the input image
    */
    void reinitialise(int width,int height);
    /**
    * function the applies the watershed (rain falling) algorithm
    */
    void rain();
    /**
    * function that resets all the flags for the desired output
    */
    void resetFlags();

    //_________ public attributes _______________
    /**
    * pointer to the most salient blob
    */
    YARPBox* max_boxes;
    /**
    * flag that allows the thread to run since all the inputs are ready
    */
    bool freetorun;
    /**
	* image which is plotted in the drawing area
	*/
	ImageOf<PixelRgb> *image_out; //
    /**
	* image which is plotted in the drawing area
	*/
	ImageOf<PixelRgb> *image_out2;
     /**
    * pointer to the input image
    */
    ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;
    /**
    * pointer to the red plane input image
    */
    ImageOf<PixelMono> *ptr_inputImgRed;
    /**
    * pointer to the green plane input image
    */
    ImageOf<PixelMono> *ptr_inputImgGreen;
    /**
    * pointer to the input blue plane image
    */
    ImageOf<PixelMono> *ptr_inputImgBlue;
    /**
    * pointer to the input image R+G-
    */
    ImageOf<PixelMono> *ptr_inputImgRG;
    /**
    * pointer to the input image G+R-
    */
    ImageOf<PixelMono> *ptr_inputImgGR;
    /**
    * pointer to the input image B+Y-
    */
    ImageOf<PixelMono> *ptr_inputImgBY;
    
   /**
    * pointer to the output image of the watershed algorithm
    */
    ImageOf<PixelRgb>* _procImage;
    /**
    * pointer to the output image of the watershed algorithm
    */
    ImageOf<PixelMono>* _outputImage;
    //---------- flags --------------------------
	/**
	* flag for drawing contrastLP
	*/
	bool contrastLP_flag;
	/**
	* flag for drawing meanColourImage
	*/
	bool meanColour_flag;
	/**
	* flag for drawing blobCatalog
	*/
	bool blobCataloged_flag;
	/**
	* flag for drawing foveaBlob
	*/
	bool foveaBlob_flag;
	/**
	* flag for drawing colorVQ
	*/
	bool colorVQ_flag;
	/**
	* flag for drawing maxSaliencyBlob
	*/
	bool maxSaliencyBlob_flag;
	/**
	* flag for drawing blobList
	*/
	bool blobList_flag;
	/**
	* flag for the drawings
	*/
	bool tagged_flag;
	/**
	* flag for drawing watershed image
	*/
	bool watershed_flag;
    
};

#endif //__BLOBFINDERTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
