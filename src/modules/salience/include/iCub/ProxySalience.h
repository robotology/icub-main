// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_PROXYSALIENCE_INC
#define ICUB_PROXYSALIENCE_INC

//yarp
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <iCub/vis/Salience.h>

namespace iCub {
    namespace vis {
        class ProxySalience;
    }
}

/**
 * Collect salience output from remote programs.
 */
class iCub::vis::ProxySalience : public Salience,
            public yarp::os::TypedReaderCallback<yarp::sig::ImageOf<yarp::sig::PixelFloat> >
{
private:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > port;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> last;
    yarp::os::Semaphore mutex;
public:
    ProxySalience() : mutex(1) {
    }

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                           yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                           yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);


    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelFloat>& datum);

};

#endif
