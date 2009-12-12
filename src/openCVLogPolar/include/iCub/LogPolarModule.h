// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _LOGPOLARMODULE_H_
#define _LOGPOLARMODULE_H_


//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


//#include <iCub/RC_DIST_FB_logpolar_mapper.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/**
  * Module that extracts the log polar image of an input image.
  * The module is able to produce a fake output image composed of coloured circles
  * in order to help the processing downward in the module chain.
  * @author Francesco Rea


  * /cvLogPolarR/image:i active at
  * /cvLogPolarR/image:o active at
  * /cvLogPolarR/inverse:o active a
  * /cvLogPolarR/simulation:o activ
  * /cvLogPolarR/cog:o active at tc
  * /cvLogPolarR/target:i active at
  * /cvLogPolarR/cmd:i active at tc
  * /cvLogPolarR/sim:o active at tc

 */
class LogPolarModule : public Module {
private:
    /**
    * a port for reading and writing images
    */
    BufferedPort<ImageOf<PixelRgb> > port; // 
    /**
    * port for writing the log polar mapping
    */
    BufferedPort<ImageOf<PixelRgb> > port2; //
    /**
    * port for the inverse log polar mapping
    */
    BufferedPort<ImageOf<PixelRgb> > port3; //
    /**
    * port for the simulated output
    */
    BufferedPort<ImageOf<PixelRgb> > port4; //
    /**
    * port for the simulated output
    */
    BufferedPort<ImageOf<PixelRgb> > portSim; //
    /**
    * port for external commands
    */
    Port cmdPort;
    /**
    * port necessary to set the target point in the image
    */
    BufferedPort<Bottle> portTarget;
    /**
    * port that send a bottle that contains the cog of the returned image
    */
    BufferedPort<Bottle> portCOG;
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
    /**
    * rectangle of the image ROI
    */
    CvRect rec;
    /**
    * input image (always 320,240)
    */
    ImageOf<PixelRgb> *img; 
    /**
    * ------------
    */
    ImageOf<PixelRgb> *image2;
    /**
    * OpenCV image necessary during the second step processing
    */
    IplImage *cvImage;
    /**
    * OpenCV image necessary for copy the rectangular input image in the inverse mode
    */
    IplImage *cvImage1;
    /**
    * OpenCV image for the first step of processing (240,240 in INVERSE mode)
    */
    IplImage *cvImage2;
    /**
    * options of the module deteched from command line
    */
    Property options;
    /**
    * mode of work of the module<<
    */
     int mode;
     /**
     * bottle used for the COG
     */
     Bottle outBot1;
     /**
     * position of the target left X 
     */
     int targetLeftX;
     /**
     * position of the target left Y
     */
     int targetLeftY;
     /**
     * position of the target right X
     */
     int targetRightX;
     /**
     * position of the target right Y
     */
     int targetRightY;
     /**
     * semaphore for the respond function
     */
     yarp::os::Semaphore mutex;
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
    * catches all the commands that have to be executed when the module is closed
    */
    bool close(); 
    /**
    * respond to command coming from the command port
    */
    bool respond(const Bottle &command,Bottle &reply);
    /**
    * updates the module
    */
    bool updateModule();
    /**
    * loads a bitmap image
    */
    unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, char *filename){};
    /**
    * loads a bitmap image
    */
    unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelRgb> *src){};
    /**
    * loads a bitmap image
    */
    unsigned char* Load_Bitmap (int *X_Size, int *Y_Size, int *planes, ImageOf<PixelMono> *src){};
    /**
    * save the image as bitmap
    */
    void Save_Bitmap (unsigned char *image, int X_Size, int Y_Size, int planes,char *filename){};
};


// general saliency filter vocab's
#define SALIENCE_VOCAB_HELP VOCAB4('h','e','l','p')
#define SALIENCE_VOCAB_SET VOCAB3('s','e','t')
#define SALIENCE_VOCAB_GET VOCAB3('g','e','t')
#define SALIENCE_VOCAB_IS VOCAB2('i','s')
#define SALIENCE_VOCAB_FAILED VOCAB4('f','a','i','l')
#define SALIENCE_VOCAB_OK VOCAB2('o','k')
#define SALIENCE_VOCAB_CHILD_COUNT VOCAB2('c','c')
#define SALIENCE_VOCAB_WEIGHT VOCAB1('w')
#define SALIENCE_VOCAB_CHILD_WEIGHT VOCAB2('c','w')
#define SALIENCE_VOCAB_CHILD_WEIGHTS VOCAB3('c','w','s')
#define SALIENCE_VOCAB_NAME VOCAB2('s','1')
#define SALIENCE_VOCAB_CHILD_NAME VOCAB2('c','n')
#define SALIENCE_VOCAB_SALIENCE_THRESHOLD VOCAB2('t','h')
#define SALIENCE_VOCAB_NUM_BLUR_PASSES VOCAB2('s','2')
// directional saliency filter vocab's
#define SALIENCE_VOCAB_DIRECTIONAL_NUM_DIRECTIONS VOCAB3('d','n','d')
#define SALIENCE_VOCAB_DIRECTIONAL_NUM_SCALES VOCAB3('d','n','s')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX VOCAB3('d','s','i')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX VOCAB3('d','d','i')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES VOCAB4('d','a','n','s')
#define SALIENCE_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME VOCAB3('d','a','n')

inline bool SALIENCE_CHECK_FAILED(bool ok, yarp::os::Bottle& response) {
    if (ok) {
        if (response.get(0).isVocab() && response.get(0).asVocab() == SALIENCE_VOCAB_FAILED) {
            return false;
        }
    }
    else
        return false;

    return true;
}

#endif
