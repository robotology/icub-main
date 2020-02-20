/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Davide Pollarolo
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

#ifndef CAN_STRING_ETH_H
#define CAN_STRING_ETH_H

#include "messages.h"
#include <string.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#define MAX_STRINGS 4

class CanFrame
{
    private:
    uint8_t size;
    uint16_t id;
    uint64_t candata;

    public:
    CanFrame();
    ~CanFrame();

    void setId (uint16_t id);
    void setSize(uint8_t size);
    void setCanData(uint64_t data);

    uint16_t getId();
    uint8_t getSize();
    uint64_t getData();
};

CanFrame::CanFrame()
{
  this->size = 0; this->id = 0; this->candata = 0;
}

CanFrame::~CanFrame()
{
}


void CanFrame::setId(uint16_t id)
{
   this->id = id;
}

void CanFrame::setSize(uint8_t size)
{
   this->size = size;
}

void CanFrame::setCanData(uint64_t data)
{
   this->candata = data;
}

uint16_t CanFrame::getId()
{
   return this->id;
}

uint8_t CanFrame::getSize()
{
   return this->size;
}

uint64_t CanFrame::getData()
{
   return this->candata;
}


class can_string_eth
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
        data_struct data_can[MAX_STRINGS];
public:
        /**
    * Default constructor.
    */
        inline can_string_eth();

        /**
    * Destructor.
    */
        inline ~can_string_eth();

        /**
        * Process a string can packet. All can packets belonging to the same string are joined
        * in the same string buffer untill the string is complete.
        * @param can_packet is the can message (CMSG* type). It must belong to the CAN_BCAST_PRINT class.
        * @return the number of the buffer where the string part has been inserted. 0 <= buffer_num <= MAX_STRINGS
        */
        inline int add_string(CanFrame* can_packet);
         /**
        * Resets the string buffer
        * @param cbuffer_num is the number of the buffer.  0 <= buffer_num <= MAX_STRINGS
        */
        inline void  clear_string(int buffer_num);

        inline char* get_string(int buffer_num);
};

can_string_eth::can_string_eth()
{
    int j=0;
    for (j=0; j<MAX_STRINGS; j++)	clear_string(j);
}

can_string_eth::~can_string_eth()
{
}

int can_string_eth::add_string(CanFrame* can_packet)
{
    int j=0;
    int string_id = 0;
    int offset = 0;
    unsigned int id=0;
    unsigned int size=0;

    id=can_packet->getId();
    size=can_packet->getSize();
    uint64_t candatabytes = can_packet->getData();
    char *candata=(char*)&candatabytes;
    int code = candata[0]&0xFF;

    string_id = (candata[1]&0xF0)>>4;
    offset    = (candata[1]&0x0F);
    data_can[string_id].board_id = char(id>>4&0xf);

    //yWarning("CANPrint msg info: ID->%d, COUNTER->%d, SIZE->%d, CODE->0x%.2x", id, offset, size, code);

    if (string_id>=MAX_STRINGS)
    {
        yError("CANPrint msg from board %d contains an ERROR! (>MAX_STRINGS)\n",data_can[string_id ].board_id);
        return -1;
    }
    for (j=0 ; j<size-2; j++)
        data_can[string_id].text_buffer[j+offset*6]=candata[j+2];

    if (code==ICUBCANPROTO_PER_MC_MSG__PRINT + 128)
    {
        data_can[string_id].maybe_last_part = true;
        data_can[string_id].expected_length=offset*6+size-2;
    }

    if (data_can[string_id].maybe_last_part)
    {
        data_can[string_id].current_length=strlen(data_can[string_id].text_buffer);

        if (data_can[string_id].expected_length==data_can[string_id].current_length)
            data_can[string_id].complete = true; //check me
    }
/*
        if (data_can[string_id].complete)
        {
                print(string_id);
                clear_string(string_id);
        }
*/
    if (data_can[string_id].complete) return string_id;
    else return -1;
}

void can_string_eth::clear_string(int buffer_num)
{
	if (buffer_num >= MAX_STRINGS) return ;

	for (int i = 0; i < 256; i++) data_can[buffer_num].text_buffer[i]=0;
	data_can[buffer_num].complete=false;
	data_can[buffer_num].maybe_last_part=false;
	data_can[buffer_num].expected_length = 0;
	data_can[buffer_num].board_id=0;
	data_can[buffer_num].current_length = 0;

}

char* can_string_eth::get_string(int buffer_num)
{
    if ((buffer_num < 0) || (buffer_num > MAX_STRINGS))
    {
       yError("Can't get CANPrint msg from board %d (>MAX_STRINGS)\n",data_can[buffer_num].board_id);
       return NULL;
    }
    return data_can[buffer_num].text_buffer;
}

#endif // CAN_STRING_ETH_H

