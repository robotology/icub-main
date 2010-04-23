#ifndef C_CARTESIAN_FRAME_CONVERTER
#define C_CARTESIAN_FRAME_CONVERTER

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <list>

#include "unmask.h"
#include "convert.h"
#include "sending_buffer.h"
#include "config.h"

#include <yarp/os/Network.h>

//#define _DEBUG

using namespace yarp::os;
class C_cframeConverter : public BufferedPort<C_sendingBuffer>
{
public:
    C_cframeConverter();
    ~C_cframeConverter();
    virtual void onRead(C_sendingBuffer& b);

private:
    C_unmask unmask_events;
    C_converter convert_events;

    clock_t start_u;
    clock_t start_p;
    clock_t stop;
};

#endif //C_CARTESIAN_FRAME_CONVERTER
