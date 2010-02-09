#include <iCub/graphicThread.h>

#define BLOB_MAXSIZE 4096
#define BLOB_MINSIZE 100


static GtkWidget *menubar;
static GtkWidget *fileMenu, *imageMenu, *helpMenu;
static GtkWidget *fileItem, *imageItem, *helpItem;
static GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
static GtkWidget *mainWindow = NULL;
//status bar of the windows
static GtkWidget *statusbar;
//drawing area
static GtkWidget *da;
// Current frame
static GdkPixbuf *frame = NULL;

// Image Receiver
static YARPImgRecv *ptr_imgRecv;

static YARPImgRecv *ptr_imgRecvRed;
static YARPImgRecv *ptr_imgRecvGreen;
static YARPImgRecv *ptr_imgRecvBlue;
static YARPImgRecv *ptr_imgRecvRG;
static YARPImgRecv *ptr_imgRecvGR;
static YARPImgRecv *ptr_imgRecvBY;

// Image to Display
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;

static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgRed;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgGreen;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgBlue;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgRG;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgGR;
static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgBY;

static yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_middleImg;
static yarp::sig::ImageOf<yarp::sig::PixelInt> *ptr_tagged;
static yarp::sig::ImageOf<yarp::sig::PixelMono>* _outputImage;
static yarp::sig::ImageOf<yarp::sig::PixelRgb>* _outputImage3;

static ImageOf<PixelMono> rgs;
static ImageOf<PixelMono> grs;
static ImageOf<PixelMono> bys;
static ImageOf<PixelMono> r2;
static ImageOf<PixelMono> g2;
static ImageOf<PixelMono> b2;


// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
// Timeout ID
static guint timeout_ID;
// Watershed operator
static WatershedOperator *_wOperator;
static SalienceOperator *_salience;
static WatershedModule *wModule;

#define _imgRecv (*(ptr_imgRecv))

#define _imgRecvRed (*(ptr_imgRecvRed))
#define _imgRecvGreen (*(ptr_imgRecvGreen))
#define _imgRecvBlue (*(ptr_imgRecvBlue))
#define _imgRecvRG (*(ptr_imgRecvRG))
#define _imgRecvGR (*(ptr_imgRecvGR))
#define _imgRecvBY (*(ptr_imgRecvBY))

#define _inputImg (*(ptr_inputImg))

#define _inputImgRed (*(ptr_inputImgRed))
#define _inputImgGreen (*(ptr_inputImgGreen))
#define _inputImgBlue (*(ptr_inputImgBlue))
#define _inputImgRG (*(ptr_inputImgRG))
#define _inputImgGR (*(ptr_inputImgGR))
#define _inputImgBY (*(ptr_inputImgBY))

#define _middleImg (*(ptr_middleImg))
#define _tagged (*(ptr_tagged))
#define _semaphore (*(ptr_semaphore))

/**
* default constructor
*/
graphicThread::graphicThread(){}
/**
* destructor
*/
graphicThread::~graphicThread(){}
/**
*	initialization of the thread 
*/
bool graphicThread::threadInit(){}
/**
* active loop of the thread
*/
void graphicThread::run(){}
/**
*	releases the thread
*/
void graphicThread::threadRelease(){}