#ifndef C_PRINT
#define C_PRINT

//std
#include <iostream>
#include <ctime>
#include <list>

#include "config.h"
//OpenCV
#include <cv.h>
#include <highgui.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//#define _DEBUG

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
class C_print
{
public:
	C_print();
	~C_print();
	void print_events( ImageOf<PixelMono16> );
private:
};
#endif //C_PRINT
