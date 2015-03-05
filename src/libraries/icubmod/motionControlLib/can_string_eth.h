/*
 * Copyright (C) 2014 The RobotCub Consortium
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
#include <can_string_generic.h>
#include <yarp/dev/CanBusInterface.h>
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





class can_string_eth :public can_string_generic
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

        inline char* get_string(int buffer_num);
};

can_string_eth::can_string_eth()
{
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

    string_id = (candata[1]>>4);
    offset = (candata[1]&0x0F);
    data[string_id].board_id = char(id>>4&0xf);
    if (string_id>=MAX_STRINGS)
    {
        yError("CANPrint msg from board %d contains an ERROR! (>MAX_STRINGS)\n",data[string_id ].board_id);
        return -1;
    }
    for (j=0 ; j<size-2; j++)
        data[string_id].text_buffer[j+offset*6]=candata[j+2];

    if (candata[0]==ICUBCANPROTO_PER_MC_MSG__PRINT + 128)
    {
        data[string_id].maybe_last_part = true;
        data[string_id].expected_length=offset*6+size-2;
    }

    if (data[string_id].maybe_last_part)
    {
        data[string_id].current_length=strlen(data[string_id].text_buffer);

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
        if (data[string_id].complete) return string_id;
            else return -1;
}

char* can_string_eth::get_string(int buffer_num)
{
    if ((buffer_num < 0) || (buffer_num > MAX_STRINGS))
    {
       yError("Can't get CANPrint msg from board %d (>MAX_STRINGS)\n",data[buffer_num].board_id);
       return NULL;
    }
    return data[buffer_num].text_buffer;
}

#endif // CAN_STRING_ETH_H

