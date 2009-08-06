#ifndef __can_string_generic__
#define __can_string_generic__

#include "messages.h"

#include <yarp/dev/CanBusInterface.h>
#define MAX_STRINGS 4

class can_string_generic
{
private:

struct data_struct{
	char text_buffer [256];
	bool complete;
	bool maybe_last_part;
	char board_id;
	int expected_length;
	int current_length;
};
	data_struct data[MAX_STRINGS];

public:
	/**
    * Default constructor.
    */
	inline can_string_generic();

	/**
    * Destructor.
    */
    inline ~can_string_generic();

	/**
    * Process a string can packet. All can packets belonging to the same string are joined
	* in the same string buffer untill the string is complete.
    * @param can_packet is the can message (CMSG* type). It must belong to the CAN_BCAST_PRINT class.
    * @return the number of the buffer where the string part has been inserted. 0 <= buffer_num <= MAX_STRINGS
    */
	inline int add_string(void* can_packet);

    /**
    * Prints a string buffer. The buffer can be already completed (all parts of the can
	* string have been already received or not). If the string contains a "hole" (a can
	* packet is not arrived yet), the string will be printed till the first missing packet. 
    * @param buffer_num is the number of the buffer.  0 <= buffer_num <= MAX_STRINGS
    * @return the content of the buffer.
    */
	inline char* print(int buffer_num);
	
	/**
    * Resets the string buffer
    * @param cbuffer_num is the number of the buffer.  0 <= buffer_num <= MAX_STRINGS
    */
	inline void  clear_string(int buffer_num);
};

can_string_generic::can_string_generic()
{
	int j=0;
	for (j=0; j<MAX_STRINGS; j++)	clear_string(j);
}

can_string_generic::~can_string_generic()
{
}

void can_string_generic::clear_string(int buffer_num)
{
	if (buffer_num >= MAX_STRINGS) return ;

	for (int i = 0; i < 256; i++) data[buffer_num].text_buffer[i]=0;
	data[buffer_num].complete=false;
	data[buffer_num].maybe_last_part=false;
	data[buffer_num].expected_length = 0;
	data[buffer_num].board_id=0;
	data[buffer_num].current_length = 0;

}

int can_string_generic::add_string(void* can_packet)
{
	int j=0;
	int string_id = 0;
	int offset = 0;
	yarp::dev::CanMessage* t;
    unsigned int id=0;
    unsigned char len=0;

	t = (yarp::dev::CanMessage*) (can_packet); 
    id=t->getId();
    len=t->getLen();
    unsigned char *candata=t->getData();
	string_id = (candata[1]>>4);
	offset = (candata[1]&0x0F);
    data[string_id].board_id = char(id>>4&0xf);

	if (string_id>=MAX_STRINGS) 
	{
		ACE_OS::printf("msg from board %d contains an ERROR! (>MAX_STRINGS)\r\n",data[string_id ].board_id);
		return -1;
	}

	for (j=0 ; j<len-2; j++)
		data[string_id].text_buffer[j+offset*6]=candata[j+2];

	if (candata[0]==CAN_BCAST_PRINT + 128)
	{
		data[string_id].maybe_last_part = true;	
		data[string_id].expected_length=offset*6+len-2;
	}

	if (data[string_id].maybe_last_part)
	{
		data[string_id].current_length=ACE_OS::strlen(data[string_id].text_buffer);

		if (data[string_id].expected_length==data[string_id].current_length)
			data[string_id].complete = true; //check me
	}
/*
	if (data[string_id].complete)
	{
		print(string_id);
		clear_string(string_id);
	}
*/
/*
	//DEBUG ONLY
	ACE_OS::printf("%d %d\r\n",string_id,offset);
	
	for (j=0 ; j<50; j++)
		ACE_OS::printf("%c",data[string_id].text_buffer[j]);

	ACE_OS::printf("\r\n");

	for (j=0 ; j<50; j++)
		ACE_OS::printf("%d",data[string_id].text_buffer[j]);

	ACE_OS::printf("\r\n");
*/
	if (data[string_id].complete) return string_id;
	else return -1;
}

char* can_string_generic::print(int buffer_num)
{
	if (buffer_num>=MAX_STRINGS) return 0;

	//DEBUG ONLY
	//ACE_OS::printf("msg from board %d, (buf:%d) : %s \r\n",data[buffer_num].board_id ,buffer_num, data[buffer_num].text_buffer);
	ACE_OS::printf("msg from board %d: %s \r\n",data[buffer_num].board_id , data[buffer_num].text_buffer);
	
	return data[buffer_num].text_buffer;
}


#endif
