#ifndef C_CARTESIAN_FRAME_PRINTER
#define C_CARTESIAN_FRAME_PRINTER

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <list>


#include "print.h"
#include "config.h"

#include <yarp/os/Network.h>

//#define _DEBUG

using namespace yarp::os;
class C_cFramePrinter:public BufferedPort< ImageOf<PixelMono16> >
{
public:
    C_cFramePrinter();
    ~C_cFramePrinter();
    virtual void onRead(ImageOf<PixelMono16> &);

private:
    C_print print_events;

    clock_t start_p;
    clock_t stop;
};

#endif //C_CARTESIAN_FRAME_PRINTER
