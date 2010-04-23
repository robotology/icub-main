#ifndef C_CONVERTER
#define C_CONVERTER

//std
#include <iostream>
#include <ctime>
#include <list>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>


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
    int sign(int);
    float mean_event(int);

    BufferedPort<ImageOf<PixelMono16> > port;
    ImageOf<PixelMono16> base_img;

    int max_amount_of_event;
    int min_amount_of_event;

    int** cart_pol_acc;

    clock_t start;

    int height;
    int width;
};
#endif //C_CONVERTER
