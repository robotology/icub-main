// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _BLOBFINDERTHREAD_H_
#define _BLOBFINDERTHREAD_H_

//within project includes
#include <iCub/WatershedOperator.h>
#include <iCub/SalienceOperator.h>

//IPP include
#include <ippi.h>

//YARP includes
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <time.h>

//logPolar include
#include <iCub/RC_DIST_FB_logpolar_mapper.h>

using namespace yarp::os;
using namespace yarp::sig;



class blobFinderThread : public yarp::os::RateThread {
private:
    
    BufferedPort<Bottle> centroidPort; //port used for centroid position to controlGaze2
    Port triangulationPort; //port used for centroid position to iKinHead
    BufferedPort<Bottle> gazeControlPort; //port used for sending responses from triangulationPort back into i

    BufferedPort<ImageOf<PixelRgb> > inputPort; //port where the input image is read from
    BufferedPort<ImageOf<PixelMono> > edgesPort; //port where the edges image is read
    BufferedPort<ImageOf<PixelMono> > outputPort; //port that returns the image output
    BufferedPort<ImageOf<PixelRgb> > outputPort3; //port that returns the image output 3channels
    BufferedPort<ImageOf<PixelMono> > rgPort; //port where the difference of gaussian R+G- is streamed
    BufferedPort<ImageOf<PixelMono> > grPort; //port where the difference of gaussian G+R- is streamed
    BufferedPort<ImageOf<PixelMono> > byPort; //port where the difference of gaussian B+Y- of the image is streamed
    BufferedPort<ImageOf<PixelMono> > yellowPort; //port where the yellow plane of the image is streamed
    BufferedPort<ImageOf<PixelRgb> > checkPort; //port where the remapped image is streamed

    ImageOf<PixelMono> *outContrastLP; //image result of the function outContrastLP
    ImageOf<PixelMono> *tmpImage; //buffer image for received image
    ImageOf<PixelBgr> *outMeanColourLP; //image result of the function meanColourLP;
    
    IppiSize srcsize; //ipp reference to the size of the input image
    int width; //width of the input image
    int height; //height of the input image
    
    std::string name; //name of the module and rootname of the connection
    bool reinit_flag; //flag that indicates when the reinitiazation has already be done
    bool interrupted_flag; //flag that indicates when the thread has been interrupted
    bool resized_flag; //flag that indicates if the images have been resized
    Semaphore mutex; //semaphore for the respond function
    int ct; //execution step counter
    
    ImageOf<PixelRgb> *img; //input image
    ImageOf<PixelMono> *edges; //edges image
   
    WatershedOperator *wOperator; //reference to the watershed operator
    char* blobList; //vector of boolean which tells whether there is a blob or not
    
    ImageOf<PixelRgb>* _outputImage3; //pointer to the 3 channels output image of the watershed algorithm
    ImageOf<PixelRgb>* _outputImage3Merged; //pointer to the 3 channels output image plus the saliencymap
    ImageOf<PixelRgb>* _outputImage3Cart; //pointer to the cartesian image of the saliency
    ImageOf<PixelMono> *_inputImgRGS; //input image of the opponency R+G-
    ImageOf<PixelMono> *_inputImgGRS; //input image of the opponency G+R-
    ImageOf<PixelMono> *_inputImgBYS; //input image of the opponency B+Y-
    ImageOf<PixelInt> *ptr_tagged; //pointer to the image of tags
    ImageOf<PixelInt>* tagged; //vector of tags to the sequence of blobs
    ImageOf<PixelMono> *blobFov; //image of the fovea blob
    
    int searchRG; //R+G- value for the search
    int searchGR; //G+R- value for the search
    int searchBY; //B+Y- value for the search

    lp2CartPixel *l2cTable; //look-up table for cartesian reconstruction
    
    double startTimer, endTimer;

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
    
    /**
    * function that free memory allocated for the look-up table
    */
    bool freeLookupTables();
public:
    /**
    * default constructor
    */
    blobFinderThread();
    /**
    * constructor
    * param rateThread period of the processing thread
    */
    blobFinderThread(int rateThread);

    /**
    * destructor
    */
    ~blobFinderThread(){};

    /**
    *initialization of the thread 
    */
    bool threadInit();

    /**
    * active loop of the thread
    */
    void run();

    /**
    * Call this to stop the thread, this call blocks until the thread is terminated (and releaseThread() called). 
    */
    void stop();

    /**
    *releases the thread
    */
    void threadRelease();

    /**
    * function that reinitiases some attributes of the class
    * @param height height of the input image
    * @param width width of the input image
    */
    void reinitialise(int width,int height);

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();
    
    /**
    * function that gives reference to the name of the module
    * @param name of the module
    */
    void setName(std::string name);
    
    /**
    * function that returns the name of the module
    * @param str string to be added
    * @return name of the module
    */
    std::string getName(const char* str);
    
    /**
    * function the applies the watershed (rain falling) algorithm
    * @param edgesImage image representing the edges extracted from colourOpponency maps
    */
    void rain(ImageOf<PixelMono>* edgesImage);
    
    /**
    * function that resets all the flags for the desired output
    */
    void resetFlags();
    
    /**
    * streams out data on ports
    * @return return whether the operation was successful
    */
    bool outPorts();
    
    /**
    * function that reads the ports for colour RGB opponency maps
    */
    bool getOpponencies();
    
    /**
    * function that reads the ports for the RGB planes
    * @param inputImage image where we can extract planes from
    */
    bool getPlanes(ImageOf<PixelRgb>* inputImage);

    SalienceOperator *salience; //reference to the salience operator
    YARPBox* max_boxes; //pointer to the most salient blob
    bool freetorun; //flag that allows the thread to run since all the inputs are ready

    ImageOf<PixelMono> *image_out; //image which is plotted in the drawing area
    ImageOf<PixelRgb> *image_out2; //image which is plotted in the drawing area

    ImageOf<yarp::sig::PixelRgb> *ptr_inputImg; //pointer to the input image
    ImageOf<PixelMono> *ptr_inputImgRed; //pointer to the red plane input image
    ImageOf<PixelMono> *ptr_inputImgGreen; //pointer to the green plane input image
    ImageOf<PixelMono> *ptr_inputImgBlue; //pointer to the input blue plane image
    ImageOf<PixelMono> *ptr_inputImgRG; //pointer to the input image R+G-
    ImageOf<PixelMono> *ptr_inputImgGR; //pointer to the input image G+R-
    ImageOf<PixelMono> *ptr_inputImgBY; //pointer to the input image B+Y-
    ImageOf<PixelRgb>* _procImage; //pointer to the output image of the watershed algorithm
    ImageOf<PixelMono>* _outputImage; //pointer to the output image of the watershed algorithm

    bool contrastLP_flag; //flag for drawing contrastLP
    bool meanColour_flag; //flag for drawing meanColourImage
    bool blobCataloged_flag; //flag for drawing blobCatalog
    bool foveaBlob_flag; //flag for drawing foveaBlob
    bool colorVQ_flag; //flag for drawing colorVQ
    bool maxSaliencyBlob_flag; //flag for drawing maxSaliencyBlob
    bool blobList_flag; //flag for drawing blobList
    bool tagged_flag; //flag for the drawings
    bool watershed_flag; //flag for drawing watershed image
    bool filterSpikes_flag; //function that indicates if the stimuli have to be processed
    int maxBLOB; //maxBLOB dimension
    int minBLOB; //minBLOB dimension
    double salienceTD; //saliencyTOT linear combination Ktd coefficient (TOP DOWN saliency weight)
    double salienceBU; //saliencyTOT linear combination Kbu coefficient (BOTTOM-UP saliency weight)
    double targetRED; //red intensity of the target that has been found 
    double targetGREEN; //green intensity of the target that has been found 
    double targetBLUE; //blue intensity of the target that has been found 
    double constantTimeGazeControl; //value that represent the constantTimeGazeControl of the sensorial system in terms of second
    double constantTimeCentroid; //value that represent the constantTimeCentroid of the sensorial system in terms of second
    int count; //counter of cycle for maxsaliency blob
    int max_tag; //number of blobs
    int countSpikes; //number of spikes which are count to get the strongest
    int minBoundingArea; //dimension of the bounding area in saliency BU algorithm
    double pArea; //percentage of the blobdimension considered surrounding area
};

#endif //__BLOBFINDERTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
