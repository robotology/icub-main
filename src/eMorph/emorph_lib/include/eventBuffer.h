// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup 
 * @ingroup emorph_lib
 *
 * Interface for the event-based camera of the emorph project.
 *
 * Author: Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/


#ifndef __eventBuffer__
#define __eventBuffer__


#include <yarp/os/all.h>
#include <cstring>

const int SIZE_OF_DATA = 32768;

/**
 *
 */
class eventBuffer : public yarp::os::Portable
{
public:
	eventBuffer();
	eventBuffer(char*, int);
	~eventBuffer();

    void operator=(const eventBuffer&);
    eventBuffer(const eventBuffer&);

    virtual bool write(yarp::os::ConnectionWriter&);
    virtual bool read(yarp::os::ConnectionReader&);

	void set_data(char*, int);

	char* get_packet(){return packet;};
	int get_sizeOfPacket(){return size_of_the_packet;};

private:
	char* packet;
	int size_of_the_packet;
};


#endif

