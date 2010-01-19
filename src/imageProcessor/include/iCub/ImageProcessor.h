// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _IMAGEPROCESSOR_H_
#define _IMAGEPROCESSOR_H_

//ipp include
#include <ippi.h>

//utils includes
#include <iCub/canny.h>
#include <iCub/convert_bitdepth.h>

//openCV includes
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

const int THREAD_RATE=30;

/**
 *This code groups together a series of useful functions that can be used for ImageProcessing
 */


class ImageProcessor:public RateThread
{
    private:
        /**
        * width step of 8u images
        */
        int psb;
        /**
        * width step of 32f images
        */
        int psb32;
        /**
        * green 1-channel image second plane of the input image
        */
        ImageOf<PixelMono>* green_yarp;//
        /**
        * red 1-channel image first plane of the input image
        */
        ImageOf<PixelMono>* red_yarp; //
        /**
        * blue 1-channel image third plane of the input image
        */
        ImageOf<PixelMono>* blue_yarp; //
        /**
        * ippi image of the R+G- Opponency Colour Image  necessary for computation
        */
        Ipp8u* redGreen_ippi; //
        /**
        * ippi image of the G+R- Opponency Colour Image  necessary for computation
        */
        Ipp8u* greenRed_ippi; //
        /**
        * ippi image of the B+Y- Opponency Colour Image  necessary for computation
        */
        Ipp8u* blueYellow_ippi; //
        /**
        * ippi image of the green plane necessary for computation
        */
        Ipp8u* greenPlane_ippi; //
        /**
        * ippi image of the red plane necessary for computation
        */
        Ipp8u* redPlane_ippi; //
        /**
        * ippi image of the blue plane necessary for computation
        */
        Ipp8u* bluePlane_ippi; //
        /**
        * temporal object for CombineMax process
        */
        ImageOf<PixelMono>* edgesBlue;
        /**
        * temporal object for CombineMax process
        */
        ImageOf<PixelMono>* edgesRed;
        /**
        * temporal object for CombineMax process
        */
        ImageOf<PixelMono>* edgesGreen;
        /**
        * temp variable for plane extraction;
        */
        ImageOf<PixelMono>* outputImage; //
        
        //ImageOf<PixelMono>* outputImagePlane; //temp variable for plane extraction;
        
        /**
        * temp variable for plane extraction;
        */
        ImageOf<PixelMono> *bluePlane_tmp; //
        /**
        * temp variable for plane extraction;
        */
        ImageOf<PixelMono> *redPlane_tmp; //
        /**
        * temp variable for plane extraction;
        */
        ImageOf<PixelMono> *greenPlane_tmp;//
        /**
        * temp variable for plane extraction;
        */
        ImageOf<PixelMono> *tmp;//
        /**
        * temp variable for plane extraction;
        */
        ImageOf<PixelRgb> *image_out;
        /**
        * temp variable for plane extraction;
        */
        ImageOf<PixelMono> *image_tmp;

        /**
        * temp variable for the plane extraction
        */
        Ipp32f* redPlane_ippi32; //
        /**
        * temp variable for the plane extraction
        */
        Ipp32f* bluePlane_ippi32; //
        /**
        * temp variable for the plane extraction
        */
        Ipp32f* greenPlane_ippi32; //

        /**
        * temp variable for the plane extraction
        */
        Ipp32f* redGreen_ippi32; //
        /**
        * temp variable for the plane extraction
        */
        Ipp32f* blueYellow_ippi32; //
        /**
        * temp variable for the plane extraction
        */
        Ipp32f* greenRed_ippi32; //

        /**
        * temp variable for the plane extraction
        */
        Ipp32f* redPlane_ippi32_f; //
        /**
        * temp variable for the plane extraction
        */
        Ipp32f* bluePlane_ippi32_f; //
        /**
        * temp variable for the plane extraction
        */
        Ipp32f* yellowPlane_ippi32_f; //
        /**
        *temp variable for the plane extraction
        */
        Ipp32f* greenPlane_ippi32_f; //
        /**
        * width of the input image
        */
        int width; //
        /**
        * height of the input image
        */
        int height; //
        //redGreen opponency
        Ipp32f* inputRedGreen32; //  = ippiMalloc_32f_C1(width,height,&psb32);
        Ipp8u* outputRedGreen2; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp8u* outputRedGreen; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp8u* outputRedGreen3; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp32f* outputRedGreen32B; // = ippiMalloc_32f_C1(width,height,&psb32); 
        Ipp32f* outputRedGreen32; // = ippiMalloc_32f_C1(width,height,&psb32);

        //greenRed opponency
        Ipp32f* inputGreenRed32; // = ippiMalloc_32f_C1(width,height,&psb32);
        Ipp8u* outputGreenRed2; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp8u* outputGreenRed; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp8u* outputGreenRed3; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp32f* outputGreenRed32; // = ippiMalloc_32f_C1(width,height,&psb32);
        Ipp32f* outputGreenRed32B; // = ippiMalloc_32f_C1(width,height,&psb32);
       
        //blueYellow opponency temporary images
        Ipp32f* outputBlueYellow32; // = ippiMalloc_32f_C1(width,height,&psb32);
        Ipp32f* outputBlueYellow32B; // = ippiMalloc_32f_C1(width,height,&psb32);
        Ipp32f* inputBlueYellow32; // = ippiMalloc_32f_C1(width,height,&psb32);
        Ipp8u* outputBlueYellow; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp8u* outputBlueYellow2; // = ippiMalloc_8u_C1(width,height,&psb);
        Ipp8u* outputBlueYellow3; // = ippiMalloc_8u_C1(width,height,&psb);
        
    public:
        /**
        * default constructor
        */
        ImageProcessor();//
        /**
        * default destructor
        */
        ~ImageProcessor();//
        /**
        * generic constructor
        */
        ImageProcessor(ImageOf<PixelRgb>* inputImage );//
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
        * method that resize images once the processor knows the dimesions of the input
        */
        void resizeImages(int width, int height);
        /**
        * combines the 3 edge images saving the maximum value for every pixel 
        */
        ImageOf<PixelMono>* combineMax(); //
        /**
        * the output image is the edged image of the input image along the axes x and y
        */
        ImageOf<PixelMono>* findEdges (); //
        /**
        * finds the edges in the red plane
        */
        ImageOf<PixelMono>* findEdgesRedOpponency (); //
        /**
        * finds the edges in the green plane
        */
        ImageOf<PixelMono>* findEdgesGreenOpponency (); //
        /**
        * finds the edges in the blue plane
        */
        ImageOf<PixelMono>* findEdgesBlueOpponency (); //
        //ImageOf<PixelMono>* findEdgesRed (); //finds the edges in the red plane
        /**
        * finds the edges in the green plane
        * @return the edge image of the green plane
        */
        ImageOf<PixelMono>* findEdgesGreen (); //
        /**
        * finds the edges in the blue plane
        */
        ImageOf<PixelMono>* findEdgesBlue ();
        /**
        * function that blanks out the border of a given image
        * @param the input image which is modified by the function
        */
        void blankBorder(Ipp8u* image, int maxLines=3);
        /**
        * gets as output the outputImage (last image produced)
        */
        ImageOf<PixelRgb>* getOutputImage(); //
        /**
        * get the Red plane of the input image
        */
        ImageOf<PixelMono>* getRedPlane ( ImageOf<PixelRgb>* src,ImageOf<PixelMono>* tmp ); //
        /**
        * get the Blue plane of the input image
        */
        ImageOf<PixelMono>* getBluePlane ( ImageOf<PixelRgb>* src,ImageOf<PixelMono>* tmp ); //
        /**
        * get the Green plane of the input image
        */
        ImageOf<PixelMono>* getGreenPlane ( ImageOf<PixelRgb>* src,ImageOf<PixelMono>* tmp ); //
        /**
        * normalise the image between the max and min value
        */
        ImageOf<PixelMono>* normalize ( ImageOf<PixelMono>* src ); //
        /**
        * uses IPP function to left shift the src image
        */
        ImageOf<PixelMono>*  LShiftC ( ImageOf<PixelMono> *src ); //
        /**
        * uses IPP function to right shift the src image
        */
        ImageOf<PixelMono>* RShiftC (ImageOf<PixelMono> *src); //
        /**
        * extracts the min and the max value within an image
        * @param src source image
        * @param maxValue reference to the max integer value
        * @param maxValue reference to the min integer value
        */
        void minMax(ImageOf<PixelMono> *src,int* maxValue,int* minValue);
        /**
        * fulls the range of an image
        */
        ImageOf<PixelMono>* fullRange(ImageOf<PixelMono> *src,int *mx, int *mn);
     
        
        
        ImageOf<PixelMono> lineMax(ImageOf<PixelMono> &src);
        /**
        * add two images (using IPP)
        */
        ImageOf<PixelMono>* add(ImageOf<PixelMono> *src1,ImageOf<PixelMono> *src2); //
        /**
        * subtract two images (using IPP)
        */
        ImageOf<PixelMono>* subtract(ImageOf<PixelMono> *src1, ImageOf<PixelMono> *src2); //
        /**
        * convert pixel order rgba to planar order yuv channels
        */
        void convertRGB(IppiSize sze,Ipp8u * rgba_orig, int psb_o); //
        void setInputImage(ImageOf<PixelRgb> *src);
        /**
        * process the src image considering the imageprocessor flags
        */
        ImageOf<PixelRgb>* process (ImageOf<PixelRgb> *src); //

        //------------- PUBLIC ATTRIBUTES ------------

        /**
        * value with which the sobel mask is built
        */
        int maskSeed;
        /**
        * value of the top sobel mask
        */
        int maskTop;
        /**
        * input image  of the processing
        */
        ImageOf<PixelRgb> *inImage; // 
        /**
        * yarp image for Opponency Map R+G-
        */
        ImageOf<PixelMono>* redGreen_yarp;
        /**
        * yarp image for Opponency Map G+R-
        */
        ImageOf<PixelMono>* greenRed_yarp;
        /**
        * yarp image for Opponency Map B+Y-
        */
        ImageOf<PixelMono>* blueYellow_yarp;
        /**
        * edges yarp image of Opponency Map R+G- 
        */
        ImageOf<PixelMono>* greenRedEdges_yarp;
        /**
        * edges yarp image of Opponency Map R+G-
        */
        ImageOf<PixelMono>* redGreenEdges_yarp;
        /**
        * edges yarp image of Opponency Map R+G-
        */
        ImageOf<PixelMono>* blueYellowEdges_yarp;
        /**
        * yarp image of the composition of all the edges
        */
        ImageOf<PixelMono>* edges_yarp;
        /**
        * temporary image of the last processing performed
        */
        ImageOf<PixelRgb> *portImage; //
        /**
        * temp images necessary for denoising
        */
        ImageOf<PixelMono>* redGreenEdges; //
        /**
        * temp images necessary for denoising
        */
        ImageOf<PixelMono>* greenRedEdges; //
        /**
        * temp images necessary for denoising
        */
        ImageOf<PixelMono>* blueYellowEdges; //
        /**
        * temp images necessary for denoising
        */
        ImageOf<PixelMono>* edgesOutput; //
        //
        /**
        * temp images necessary for planes extraction
        */
        ImageOf<PixelMono> *bluePlane; //
        /**
        * temp images necessary for planes extraction
        */
        ImageOf<PixelMono> *redPlane; //
        /**
        * temp images necessary for planes extraction
        */
        ImageOf<PixelMono> *greenPlane; //
        /**
        * temp images necessary for planes extraction
        */
        ImageOf<PixelMono> *yellowPlane; //
        /**
        * tmp IPLImage necessary for edge detection 16 bit
        */
        IplImage *cvImage16; //
        /**
        * tmp IPLImage necessary for edge detection 16 bit
        */
        IplImage *cvImage8;
        Ipp8u* im_out;
        
        /**
        * CANNY Operator
        */
        CANNY *cannyOperator; //--- 
        
        /**
        * processor flag
        */
        int inputImage_flag;  //--- 
        /**
        * processor flag
        */
        int redPlane_flag; //--- 
        /**
        * processor flag
        */
        int greenPlane_flag; //--- processor flag
        /**
        * processor flag
        */
        int bluePlane_flag; //--- processor flag
        /**
        * processor flag
        */
        int yellowPlane_flag; //--- processor flag
        /**
        * processor flag
        */
        int colourOpponency_flag;//--- processor flag
        /**
        * processor flag
        */
        int findEdges_flag; //--- processor flag
        /**
        * processor flag
        */
        int normalize_flag; //--- processor flag
        /**
        * processor flag
        */
        int combineMax_flag; //--- processor flag
        /**
        * flag that allows the processing when all the inputs are ready
        */
        int canProcess_flag; //--
        /**
        * flag that indicates if there has been the resizing
        */
        bool resized_flag;
        /**
        * flag that indicates the colour opponency map is ready B+Y-
        */
        int blueYellow_flag;
        /**
        * flag that indicates the colour opponency map is ready R+G-
        */
        int redGreen_flag;
        /**
        * flag that indicates the colour opponency map is ready G+R-
        */
        int greenRed_flag;
        /**
        * flag for maximum convolution
        */
        bool CONVMAX;   
        /**
        * flag for convolution filter
        */
        bool CONVFILTER;
        /**
        * flag that allows find edges function to use sobel from IPP library
        */
        bool IPPISOBEL;
        /**
        * flag that allows find edges function to use opencv sobel
        */
        bool OPENCVSOBEL;
        /**
        * flag that allows find edges function to use a sequence of convolutions
        */
        bool CONVSEQ;

        //---
        Ipp32f src0f[9];
        Ipp32f src1f[9];
        Ipp32f src2f[9];
        Ipp32f src3f[9];
        Ipp32f src4f[9];
        Ipp32f src5f[9];
        Ipp32f src6f[9];
        Ipp32f src7f[9];
        //---- 
        Ipp32s src0s[9];
        Ipp32s src1s[9];
        Ipp32s src2s[9];
        Ipp32s src3s[9];
        Ipp32s src4s[9];
        Ipp32s src5s[9];
        Ipp32s src6s[9];
        Ipp32s src7s[9];
     
        /**
        * parameter of the findEdges function
        */
        static const int CONVMAX_TH=100; //
        /**
        * parameter of the findEdges function
        */
        static const int CONVSEQ_TH=500; //
        
};

#endif // _IMAGEPROCESSMODULE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

