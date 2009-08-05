#ifndef _HL_INCLUDES_
#define _HL_INCLUDES_

//------------------------------------------------------------------------------
// general definitions
//------------------------------------------------------------------------------
#define PIXEL PixelMono
#define MAX(a, b)    ((a < b) ? b : a)
#define MIN(a, b)    ((a > b) ? b : a)
#define ROUNDTOINT(a)	 ((int)floor((double)a + 0.5))
#define PI	3.141592653589793234846

//------------------------------------------------------------------------------
// File sources definitions
//------------------------------------------------------------------------------
//image source 1
/*

#define FILE_DIR "H:/Realization/Images/img_source1/"
#define FILE_NAME "collector"
#define FILE_ENDING ".ppm"
#define NUMLEN 3
#define ZEROFILLING 0
#define MINFILENUM 0
#define MAXFILENUM 499
#define MOTORINFORMATION "dump5.txt"

//image source 2
#define FILE_DIR "H:/Realization/Images/img_source2/"
#define FILE_NAME "collector"
#define FILE_ENDING ".ppm"
#define NUMLEN 3
#define ZEROFILLING 0
#define MINFILENUM 0
#define MAXFILENUM 499
#define MOTORINFORMATION "dump1.txt"
*/

/*

//image source 3a
#define FILE_DIR "H:/Realization/Images/img_source3/"
#define FILE_NAME "frame01_"
#define FILE_ENDING ".ppm"
#define NUMLEN 4
#define ZEROFILLING 1
#define MINFILENUM 0
#define MAXFILENUM 119
#define MOTORINFORMATION "arm1.log"
*/
/*
//image source 3b
#define FILE_DIR "H:/Realization/Images/img_source3/"
#define FILE_NAME "frame02_"
#define FILE_ENDING ".ppm"
#define NUMLEN 4
#define ZEROFILLING 1
#define MINFILENUM 0
#define MAXFILENUM 121
#define MOTORINFORMATION "arm2.log"
//image source 3c
#define FILE_DIR "H:/Realization/Images/img_source3/"
#define FILE_NAME "frame03_"
#define FILE_ENDING ".ppm"
#define NUMLEN 4
#define ZEROFILLING 1
#define MINFILENUM 0
#define MAXFILENUM 120
#define MOTORINFORMATION "arm3.log"

//image source 4a
#define FILE_DIR "H:/Realization/Images/img_source4/"
#define FILE_NAME "frame01_"
#define FILE_ENDING ".ppm"
#define NUMLEN 4
#define ZEROFILLING 1
#define MINFILENUM 0
#define MAXFILENUM 478
#define MOTORINFORMATION "arm1.log"
*/  

//image source 4b
#define FILE_DIR "H:/Realization/Images/img_source4/"
#define FILE_NAME "frame02_"
#define FILE_ENDING ".ppm"
#define NUMLEN 4
#define ZEROFILLING 1
#define MINFILENUM 0
#define MAXFILENUM 477
#define MOTORINFORMATION "arm2.log"
/*

//image source 4c
#define FILE_DIR "H:/Realization/Images/img_source4/"
#define FILE_NAME "frame03_"
#define FILE_ENDING ".ppm"
#define NUMLEN 4
#define ZEROFILLING 1
#define MINFILENUM 0
#define MAXFILENUM 480
#define MOTORINFORMATION "arm3.log"
*/
/*

//image source 4d
#define FILE_DIR "H:/Realization/Images/img_source4/"
#define FILE_NAME "frame04_"
#define FILE_ENDING ".ppm"
#define NUMLEN 4
#define ZEROFILLING 1
#define MINFILENUM 0
#define MAXFILENUM 480
#define MOTORINFORMATION "arm4.log"*/
//------------------------------------------------------------------------------
// Modules definitions
//------------------------------------------------------------------------------
#define MOTION_THRESHOLD 20
#define FLOW_TRIGGER 0.04
//#define BACKGROUND_THRESHOLD 3
#define UPTOFILE MAXFILENUM
#define DURATION MAXFILENUM
//#define UPTOFILE 250
//#define REC_LEN 1000
/*#define CUBESIZE_X 160
#define CUBESIZE_Y 120
#define CUBESIZE_T 200
#define CUBE_SLICES 10
#define NEIGHBOURMETHOD 4
*/
#define ANGULAR_RANGE 25
#define CLUSTERNUMBER 14
#define TIMEWINDOW 200
#define BUFFERSIZE 20

//------------------------------------------------------------------------------
// OpenCV definitions
//------------------------------------------------------------------------------
#define WIN_SIZE_OPTFLOW 3 
#define PATCH_SIZE 16
#define NOF_FEATURES_OPTFLOW 1000
#define TRACKING_TRIGGER 0.05 //(PATCH_SIZE/200)

//------------------------------------------------------------------------------
// definition dependend macros
//------------------------------------------------------------------------------
#define PATCHED(a)		(ROUNDTOINT(a  / PATCH_SIZE))
#define UNPATCHED(a)	(a*PATCH_SIZE)
#define UNPATCHEDPOS(a) (UNPATCHED(a) + ROUNDTOINT(PATCH_SIZE/2))
//includes
// YARP2
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>

// including own includes

// including standard includes
#include <string>
#include <stdio.h>
#include <iostream>
//#include <math.h>

// even if it is not very beautiful style, usings are already defined here.
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::sig::file;
using namespace yarp::os;

using std::string;

// constructs of data types for all classes
struct dPoint2D {
	double x, y;
};

struct iPoint2D {
	int x, y;
};

struct dBottle2D {
	Bottle x;
	Bottle y;
};

struct iBottle2D {
	Bottle x;
	Bottle y;
};

struct patchInfo {
	iBottle2D patch_id;
	dBottle2D vector_value;
};


#endif

