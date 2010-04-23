#include "sending_buffer.h"

C_sendingBuffer::C_sendingBuffer()
{
	packet = new char[BUFFER_SIZE];
	size_of_the_packet=0;
}
C_sendingBuffer::~C_sendingBuffer()
{
	delete[] packet;
}
void C_sendingBuffer::set_data(char* i_data, int i_size)
{

	memcpy(packet, i_data, BUFFER_SIZE);
	size_of_the_packet = i_size;
}
bool C_sendingBuffer::write(yarp::os::ConnectionWriter& connection)
{
	connection.appendInt(BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT);
	connection.appendInt(2); // four elements
	connection.appendInt(size_of_the_packet);
	connection.appendBlock (packet, BUFFER_SIZE);
	connection.convertTextMode(); // if connection is text-mode, convert!
	return true;
}
bool C_sendingBuffer::read(yarp::os::ConnectionReader& connection)
{
	connection.convertTextMode(); // if connection is text-mode, convert!
	int tag = connection.expectInt();
	if (tag!=BOTTLE_TAG_LIST+BOTTLE_TAG_BLOB+BOTTLE_TAG_INT)
		return false;
	int ct = connection.expectInt();
	if (ct!=2)
		return false;
	size_of_the_packet = connection.expectInt();
	connection.expectBlock(packet, BUFFER_SIZE);
	return true;
}
