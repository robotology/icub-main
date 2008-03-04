// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2008 The RobotCub Consortium
 * Author: Marco Randazzo, Marco Maggiali
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __can_string2__
#define __can_string2__

#include "messages.h"

template<class T>
class can_string2
{
    enum __MAX_STRING
    {
        MAX_STRINGS=4,
    };

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
	can_string2();

	/**
    * Destructor.
    */
    ~can_string2();

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

template<class T>
can_string2<T>::can_string2()
{
	int j=0;
	for (j=0; j<MAX_STRINGS; j++)	clear_string(j);
}

template<class T>
can_string2<T>::~can_string2()
{
}

template<class T>
void can_string2<T>::clear_string(int buffer_num)
{
	if (buffer_num >= MAX_STRINGS) return ;

	for (int i = 0; i < 256; i++) data[buffer_num].text_buffer[i]=0;
	data[buffer_num].complete=false;
	data[buffer_num].maybe_last_part=false;
	data[buffer_num].expected_length = 0;
	data[buffer_num].board_id=0;
	data[buffer_num].current_length = 0;

}

template<class T>
int can_string2<T>::add_string(void* can_packet)
{
	int j=0;
	int string_id = 0;
	int offset = 0;
	T* t;

	t = (T*) (can_packet); 
	string_id = (t->data[1]>>4);
	offset = (t->data[1]&0x0F);
    data[string_id].board_id = char(t->id>>4&0xf);

	if (string_id>=MAX_STRINGS) 
	{
		printf("msg from board %d contains an ERROR! (>MAX_STRINGS)\r\n",data[string_id ].board_id);
		return -1;
	}

	for (j=0 ; j<t->len-2; j++)
		data[string_id].text_buffer[j+offset*6]=t->data[j+2];

	if (t->data[0]==ICUBCANPROTO_PER_MC_MSG__PRINT + 128)
	{
		data[string_id].maybe_last_part = true;	
		data[string_id].expected_length=offset*6+t->len-2;
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

template<class T>
char* can_string2<T>::print(int buffer_num)
{
	if (buffer_num>=MAX_STRINGS) return 0;

	//DEBUG ONLY
	//ACE_OS::printf("msg from board %d, (buf:%d) : %s \r\n",data[buffer_num].board_id ,buffer_num, data[buffer_num].text_buffer);
	printf("msg from board %d: %s \r\n",data[buffer_num].board_id , data[buffer_num].text_buffer);
	
	return data[buffer_num].text_buffer;
}


#endif
