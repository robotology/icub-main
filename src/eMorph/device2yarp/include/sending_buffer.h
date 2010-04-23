#ifndef C_SENDING_BUFFER
#define C_SENDING_BUFFER

//#include <yarp/os/Portable.h>
#include <yarp/os/all.h>

#include <cstring>

#include "config.h"

using namespace std;
using namespace yarp::os;
class C_sendingBuffer:public Portable
{
public:
	C_sendingBuffer();
	C_sendingBuffer(char*, int);
	~C_sendingBuffer();
	virtual bool write(ConnectionWriter&);
	virtual bool read(ConnectionReader&);

	void set_data(char*, int);

	char* get_packet(){return packet;};
	int get_sizeOfPacket(){return size_of_the_packet;};
private:
	char* packet;
	int size_of_the_packet;
};

#endif //C_SENDING_BUFFER
