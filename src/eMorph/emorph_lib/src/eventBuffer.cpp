// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * Data buffer implementation for the event-based camera.
 *
 * Author: Giorgio Metta
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#include <eventBuffer.h>

using namespace std;
using namespace yarp::os;

eventBuffer::eventBuffer()
{
	packet = new char[SIZE_OF_DATA];
	size_of_the_packet=0;
}

eventBuffer::eventBuffer(char* i_data, int i_size)
{
	packet = new char[SIZE_OF_DATA];
	memcpy(packet, i_data, i_size);
	size_of_the_packet = i_size;
}

eventBuffer::eventBuffer(const eventBuffer& buffer) {
	packet = new char[SIZE_OF_DATA];
    if (buffer.size_of_the_packet > 0)
        memcpy(packet, buffer.packet, sizeof(char) * buffer.size_of_the_packet);
    size_of_the_packet = buffer.size_of_the_packet;
}

eventBuffer::~eventBuffer()
{
	delete[] packet;
}

void eventBuffer::operator=(const eventBuffer& buffer) {
    if (buffer.size_of_the_packet > 0)
        memcpy(packet, buffer.packet, sizeof(char) * buffer.size_of_the_packet);
    size_of_the_packet = buffer.size_of_the_packet;
}

void eventBuffer::set_data(char* i_data, int i_size)
{
	memcpy(packet, i_data, i_size);
	size_of_the_packet = i_size;
}

bool eventBuffer::write(yarp::os::ConnectionWriter& connection)
{
	connection.appendInt(BOTTLE_TAG_LIST + BOTTLE_TAG_BLOB + BOTTLE_TAG_INT);
	connection.appendInt(2);        // four elements
	connection.appendInt(size_of_the_packet);
	connection.appendBlock(packet, size_of_the_packet);
	connection.convertTextMode();   // if connection is text-mode, convert!
	return true;
}

bool eventBuffer::read(yarp::os::ConnectionReader& connection)
{
	connection.convertTextMode();   // if connection is text-mode, convert!
	int tag = connection.expectInt();
	if (tag != BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
		return false;
	int ct = connection.expectInt();
	if (ct!=2)
		return false;
	size_of_the_packet = connection.expectInt();
	connection.expectBlock(packet, size_of_the_packet);
	return true;
}

