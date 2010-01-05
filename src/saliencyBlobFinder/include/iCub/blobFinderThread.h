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


class blobFinderThread : public Module{
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
    * pointer to the output image of the watershed algorithm
    */
    ImageOf<yarp::sig::PixelMono>* _outputImage;
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
	* flag for drawing blobCatalog
	*/
	bool blobCataloged_flag;
    /**
	*vector of tags to the sequence of blobs
	*/
	ImageOf<PixelInt>* tagged;
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
    void reinitialise(int height, int width);
    /**
    * function the applies the watershed (rain falling) algorithm
    */
    void rain();

    //_________ public attributes _______________
    
   
};

#endif //__BLOBFINDERTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
