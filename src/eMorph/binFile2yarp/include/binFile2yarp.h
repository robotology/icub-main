/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Charler Clercq, Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef C_BINFILE2YARP
#define C_BINFILE2YARP

// yarp include
#include <yarp/os/all.h>

// emorph include
#include <eventBuffer.h>

// sys include
#include <iostream>
#include <string>

class binfile2yarp : public yarp::os::Thread
{
public:
    binfile2yarp(std::string);
    ~binfile2yarp();
    virtual void run();

protected:
    yarp::os::BufferedPort<eventBuffer> port;
    FILE* raw;
    long lSize;
	char buffer[SIZE_OF_DATA]; // not particulary elegant.
    int len;

    /**
     * search the buffer to determine the duration (in timestamp units) 
     * @param sz is the current size of the buffer to parse
     * @return return the duration in steps of 10ns
     */
    int getDuration(int sz);
};

#endif //C_BINFILE2YARP
