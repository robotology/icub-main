// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef NOSE_FINDER_INC
#define NOSE_FINDER_INC

#include <yarp/sig/Image.h>
#include <yarp/os/Semaphore.h>

class NoseControl {
public:
    virtual void reset() {}
};

class NoseFinder {
private:
    yarp::os::Semaphore mutex;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> prev;
    yarp::sig::ImageOf<yarp::sig::PixelMono> mask;

public:
    NoseFinder() : mutex(1) {}

    void apply(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src,
               yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest);

    void reset();

    static NoseControl& run(int argc, char *argv[]);
};

#endif

