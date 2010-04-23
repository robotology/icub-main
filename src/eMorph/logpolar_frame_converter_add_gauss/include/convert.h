#ifndef C_CONVERT_TO_LOG_FRAME
#define C_CONVERT_TO_LOG_FRAME

//std
#include <iostream>
#include <ctime>
#include <list>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "logPolar_tools.h"
#include "config.h"

//#define _DEBUG

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
class C_converter
{
public:
	C_converter(int, int);
	~C_converter();
	ImageOf<PixelMono16> create_frame(list<AER_struct>);
	void send_frame(ImageOf<PixelMono16>);
private:

    BufferedPort<ImageOf<PixelMono16> > port;
    ImageOf<PixelMono16> base_lImg;
    ImageOf<PixelMono16> base_clImg;

    int sign(int);
    float mean_event(int);

    int max_amount_of_event;
    int min_amount_of_event;

    C_logpolarTools lp_tools;
    int** acc;
    double** postS;
    double** t_0;

    bool** spiked;

    int n;
    int m;

    clock_t start;

    int height;
    int width;

    float mu;
    float theta;
};
#endif //C_CONVERT_TO_LOG_FRAME
