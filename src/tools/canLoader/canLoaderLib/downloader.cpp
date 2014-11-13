// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "driver.h"
#include "ethDriver.h"
#include "downloader.h"
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <stdlib.h> //added for abs
#include <string.h>

#include <canProtocolLib/iCubCanProtocol.h>
#include <canProtocolLib/iCubCanProto_types.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace std;

#define CLEAR_RXBUFFER m_candriver->receive_message(rxBuffer,64,0.001);
//*****************************************************************/
void drv_sleep (double time)
{
    yarp::os::Time::delay(time/1000);
}

//*****************************************************************/
//utility function for conversion of hex string to int
int axtoi(char *hexStg)
{
    int n = 0;         // position in string
    int m = 0;         // position in digit[] to shift
    int count;         // loop index
    int intValue = 0;  // integer value of hex string
    int digit[5];      // hold values to convert
    while (n < 4) {
        if (hexStg[n]=='\0')
            break;
        if (hexStg[n] > 0x29 && hexStg[n] < 0x40 ) //if 0 to 9
            digit[n] = hexStg[n] & 0x0f;            //convert to int
        else if (hexStg[n] >='a' && hexStg[n] <= 'f') //if a to f
            digit[n] = (hexStg[n] & 0x0f) + 9;      //convert to int
        else if (hexStg[n] >='A' && hexStg[n] <= 'F') //if A to F
            digit[n] = (hexStg[n] & 0x0f) + 9;      //convert to int
        else break;
        n++;
    }
    count = n;
    m = n - 1;
    n = 0;
    while(n < count) {
        // digit[n] is value of hex digit at position n
        // (m << 2) is the number of positions to shift
        // OR the bits into return value
        intValue = intValue | (digit[n] << (m << 2));
        m--;   // adjust the position to set
        n++;   // next digit to process
    }
    return (intValue);
}

//*****************************************************************/

int cDownloader::build_id(int source, int dest)
{
    return (ICUBCANPROTO_CLASS_BOOTLOADER << 8) + ( source << 4) + dest;
}

int cDownloader::get_src_from_id (int id)
{
    // 000 1111 0000
    return ((id >> 4) & 0x0F);
}

int cDownloader::get_dst_from_id (int id)
{
    // 000 0000 1111
    return (id & 0x0F);
}

//*****************************************************************/

cDownloader::cDownloader()
{
    board_list = NULL;
    board_list_size = 0;
    connected = false;
    m_candriver=NULL;
    sprsPage=0;
    canbus_id=-1;
}


//*****************************************************************/

int cDownloader::stopdriver()
{
    if (m_candriver !=NULL)
        {
            m_candriver->destroyBuffer(txBuffer);
            m_candriver->destroyBuffer(rxBuffer);
            delete m_candriver;
            m_candriver=NULL;
            connected = false;
        }
    return 0;
}

//*****************************************************************/

int cDownloader::initdriver(Searchable &config)
{
    if (m_candriver !=NULL)
        {
            delete m_candriver;
            m_candriver=NULL;
            connected = false;
        }

    if (config.find("device").asString()=="EMS")
    {
        m_candriver = new eDriver;
    }
    else
    {
        m_candriver = new cDriver;
    }

    if (m_candriver->init(config)==-1)
        {
            if (m_candriver)
                {
                    delete m_candriver;
                    m_candriver=NULL;
                    connected = false;
                }
            return -1;
        }

    txBuffer=m_candriver->createBuffer(1);
    rxBuffer=m_candriver->createBuffer(MAX_READ_MSG);
    connected = true;

    canbus_id = config.check("canDeviceNum");
    return 0;
}

//*****************************************************************/
int cDownloader::strain_save_to_eeprom  (int target_id)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     // Send transmission command to strain board
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= 0x09;
    int ret = m_candriver->send_message(txBuffer, 1);
    // check if send_message was successful
    if (ret==0)
        {
            yError ("Unable to send message\n");
            return -1;
        }

    return 0;
}

//*****************************************************************/
int cDownloader::sg6_get_amp_gain      (int target_id, char channel, unsigned int& gain1, unsigned int& gain2 )
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }
     
     // Send read gain command to strain board
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x1D;
     txBuffer[0].getData()[1]= channel;
     int ret = m_candriver->send_message(txBuffer, 1);
     // check if send_message was successful
     if (ret==0)
     {
         yError ("Unable to send message\n");
         return -1;
     }

     drv_sleep(3);

     //read gain
     int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
        {
          if (rxBuffer[i].getData()[0]==0x1D &&
             rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
             {
                 int ret_channel = rxBuffer[i].getData()[1];
                 if (ret_channel == channel)
                 {
                    gain1 = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                    gain2 = rxBuffer[i].getData()[4]<<8 | rxBuffer[i].getData()[5];
                    return 0;
                 }
                 else
                 {
                    yError ("sg6_get_amp_gain : invalid response\n");
                    return -1;
                 }
             }
        }

     return -1;
}

//*****************************************************************/
int cDownloader::sg6_set_amp_gain      (int target_id, char channel, unsigned int  gain1, unsigned int  gain2 )
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    //set amp gain
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(6);
    txBuffer[0].getData()[0]= 0x1E;
    txBuffer[0].getData()[1]= channel;
    txBuffer[0].getData()[2]= gain1 >> 8;
    txBuffer[0].getData()[3]= gain1 & 0xFF;
    txBuffer[0].getData()[4]= gain2 >> 8;
    txBuffer[0].getData()[5]= gain2 & 0xFF;
    int ret = m_candriver->send_message(txBuffer, 1);

     return 0;
}

//*****************************************************************/
int cDownloader::strain_get_adc(int target_id, char channel, unsigned int& adc, int type)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     // Send read channel command to strain board
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(3);
     txBuffer[0].getData()[0]= 0x0C;
     txBuffer[0].getData()[1]= channel;
     txBuffer[0].getData()[2]= type;
     int ret = m_candriver->send_message(txBuffer, 1);
     // check if send_message was successful
     if (ret==0)
         {
            yError ("Unable to send message\n");
            return -1;
        }

     drv_sleep(3);

     //read adc
     int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
        {
          if (rxBuffer[i].getData()[0]==0x0C &&
             rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
             {
                 int ret_channel = rxBuffer[i].getData()[1];
                 if (ret_channel == channel)
                 {
                    adc = rxBuffer[i].getData()[3]<<8 | rxBuffer[i].getData()[4];
                    return 0;
                 }
                 else
                 {
                    yError ("strain_get_adc : invalid response\n");
                    return -1;
                 }
             }
        }

     return -1;
}

//*****************************************************************/
int cDownloader::strain_get_offset(int target_id, char channel, unsigned int& offset)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    //read dac
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x0B;
    txBuffer[0].getData()[1]= channel;

    CLEAR_RXBUFFER
    int ret = m_candriver->send_message(txBuffer, 1);

    drv_sleep(3);

    int read_messages = m_candriver->receive_message(rxBuffer,1);
    for (int i=0; i<read_messages; i++)
    {
        if (rxBuffer[i].getData()[0]==0x0B &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                int ret_channel = rxBuffer[i].getData()[1];
                if (channel==ret_channel)
                {
                    offset = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                    return 0;
                }
                else
                {
                    yError ("strain_get_offset : invalid response\n");
                    return -1;
                }
                
            }
    }

    return -1;
}
//*****************************************************************/
int cDownloader::strain_get_calib_bias     (int target_id, char channel, signed int& bias)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //read current bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x14;
     txBuffer[0].getData()[1]= channel;
     int ret = m_candriver->send_message(txBuffer, 1);

     int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x14 &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                bias = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                return 0;
            }
     }
     return -1;
}
//*****************************************************************/
int cDownloader::strain_set_calib_bias     (int target_id)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //set calib bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x13;
     txBuffer[0].getData()[1]= 1;
     int ret = m_candriver->send_message(txBuffer, 1);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_set_calib_bias     (int target_id, char channel, int bias)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //set calib bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(5);
     txBuffer[0].getData()[0]= 0x13;
     txBuffer[0].getData()[1]= 2;
     txBuffer[0].getData()[2]= channel;
     txBuffer[0].getData()[3]= bias >> 8;
     txBuffer[0].getData()[4]= bias & 0xFF;

     int ret = m_candriver->send_message(txBuffer, 1);

     return 0;
}

//*****************************************************************/
int cDownloader::strain_reset_calib_bias (int target_id)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //reset calib bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x13;
     txBuffer[0].getData()[1]= 0;
     int ret = m_candriver->send_message(txBuffer, 1);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_get_curr_bias     (int target_id, char channel, signed int& bias)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //read current bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x16;
     txBuffer[0].getData()[1]= channel;
     int ret = m_candriver->send_message(txBuffer, 1);

     int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x16 &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                bias = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                return 0;
            }
     }
     return -1;
}
//*****************************************************************/
int cDownloader::strain_set_curr_bias     (int target_id)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //set curr bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x15;
     txBuffer[0].getData()[1]= 1;
     int ret = m_candriver->send_message(txBuffer, 1);

      return 0;
}

//*****************************************************************/
int cDownloader::strain_set_curr_bias     (int target_id, char channel, int bias)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //set calib bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(5);
     txBuffer[0].getData()[0]= 0x15;
     txBuffer[0].getData()[1]= 2;
     txBuffer[0].getData()[3]= channel;
     txBuffer[0].getData()[4]= bias >> 8;
     txBuffer[0].getData()[5]= bias & 0xFF;

     int ret = m_candriver->send_message(txBuffer, 1);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_reset_curr_bias     (int target_id)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //reset curr bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x15;
     txBuffer[0].getData()[1]= 0;
     int ret = m_candriver->send_message(txBuffer, 1);

      return 0;
}

//*****************************************************************/
int cDownloader::strain_set_serial_number (int target_id, const char* serial_number)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    //set dac
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(8);
    txBuffer[0].getData()[0]= 0x19;
    txBuffer[0].getData()[1]= serial_number[0];
    txBuffer[0].getData()[2]= serial_number[1];
    txBuffer[0].getData()[3]= serial_number[2];
    txBuffer[0].getData()[4]= serial_number[3];
    txBuffer[0].getData()[5]= serial_number[4];
    txBuffer[0].getData()[6]= serial_number[5];
    txBuffer[0].getData()[7]= serial_number[6];
    int ret = m_candriver->send_message(txBuffer, 1);

    return 0;
}

//*****************************************************************/
int cDownloader::strain_get_serial_number (int target_id, char* serial_number)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //read serial number
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(1);
     txBuffer[0].getData()[0]= 0x1A;

     CLEAR_RXBUFFER
     int ret = m_candriver->send_message(txBuffer, 1);

     drv_sleep(5);

      int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x1A &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                serial_number[0] = rxBuffer[i].getData()[1];
                serial_number[1] = rxBuffer[i].getData()[2];
                serial_number[2] = rxBuffer[i].getData()[3];
                serial_number[3] = rxBuffer[i].getData()[4];
                serial_number[4] = rxBuffer[i].getData()[5];
                serial_number[5] = rxBuffer[i].getData()[6];
                serial_number[6] = rxBuffer[i].getData()[7];
                serial_number[7] = 0;
                return 0;
            }
     }
     return -1;
}

//*****************************************************************/
int cDownloader::strain_get_eeprom_saved (int target_id, bool* status)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //read eeprom status (saved/not saved)
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(1);
     txBuffer[0].getData()[0]= 0x1B;

     CLEAR_RXBUFFER
     int ret = m_candriver->send_message(txBuffer, 1);

     drv_sleep(5);

      int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x1B &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                *status = rxBuffer[i].getData()[1]!=0;
                return 0;
            }
     }
     return -1;
}
//*****************************************************************/
int cDownloader::strain_get_matrix_gain     (int target_id, unsigned int& gain)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //read matrix gain
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(1);
     txBuffer[0].getData()[0]= 0x12;

     CLEAR_RXBUFFER
     int ret = m_candriver->send_message(txBuffer, 1);

     drv_sleep(5);

      int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x12 &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                gain = rxBuffer[i].getData()[1];
                return 0;
            }
     }
     return -1;
}

//*****************************************************************/
int cDownloader::strain_set_matrix_gain     (int target_id, unsigned int  gain)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //set matrix
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x11;
     txBuffer[0].getData()[1]= gain;

     int ret = m_candriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}

//*****************************************************************/
int cDownloader::strain_get_full_scale     (int target_id, unsigned char channel, unsigned int&  full_scale)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //read matrix gain
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x18;
     txBuffer[0].getData()[1]= channel;

     int ret = m_candriver->send_message(txBuffer, 1);

     drv_sleep(5);

      int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x18 &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                full_scale = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                return 0;
            }
     }
     return -1;
}
//*****************************************************************/
int cDownloader::strain_set_full_scale     (int target_id, unsigned char channel,  unsigned int full_scale)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

      //set matrix
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(4);
     txBuffer[0].getData()[0]= 0x17;
     txBuffer[0].getData()[1]= channel;
     txBuffer[0].getData()[2]= full_scale >> 8;
     txBuffer[0].getData()[3]= full_scale & 0xFF;

     int ret = m_candriver->send_message(txBuffer, 1);
      drv_sleep(5);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_get_matrix_rc     (int target_id, char r, char c, char matrix_id, unsigned int& elem)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //read dac
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(4);
     txBuffer[0].getData()[0]= 0x0A;
     txBuffer[0].getData()[1]= r;
     txBuffer[0].getData()[2]= c;
     txBuffer[0].getData()[3]= matrix_id;

     CLEAR_RXBUFFER
     int ret = m_candriver->send_message(txBuffer, 1);

     drv_sleep(3);

     int read_messages = m_candriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x0A &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                int ret_r = rxBuffer[i].getData()[1];
                int ret_c = rxBuffer[i].getData()[2];
                if (r==ret_r && c==ret_c)
                {
                    elem = rxBuffer[i].getData()[3]<<8 | rxBuffer[i].getData()[4];
                    return 0;
                }
                else
                {
                    yError ("strain_get_matrix_rc : invalid response\n");
                    return -1;
                }
            }
     }
     return -1;
}

//*****************************************************************/
int cDownloader::strain_set_matrix_rc     (int target_id, char r, char c, char matrix_id, unsigned int  elem)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

     //set matrix
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(6);
     txBuffer[0].getData()[0]= 0x03;
     txBuffer[0].getData()[1]= r;
     txBuffer[0].getData()[2]= c;
     txBuffer[0].getData()[3]= elem >> 8;
     txBuffer[0].getData()[4]= elem & 0xFF;
     txBuffer[0].getData()[5]= matrix_id;
     int ret = m_candriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}

//*****************************************************************/
int cDownloader::strain_set_offset(int target_id, char channel, unsigned int offset)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError (" Driver not ready\n");
            return -1;
        }

    //set dac
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(4);
    txBuffer[0].getData()[0]= 0x04;
    txBuffer[0].getData()[1]= channel;
    txBuffer[0].getData()[2]= offset >> 8;
    txBuffer[0].getData()[3]= offset & 0xFF;
    int ret = m_candriver->send_message(txBuffer, 1);
/*
    int read_messages = m_candriver->receive_message(rxBuffer);
    for (int i=0; i<read_messages; i++)
    {
        if (rxBuffer[i].getData()[0]==0x0B)
            {
                offset = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                return 0;
            }
    }
    return -1;
*/
    return 0;
}


//*****************************************************************/
int cDownloader::strain_start_sampling    (int target_id)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    // Send transmission command to strain board
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x07;
    txBuffer[0].getData()[1]= 0x01;
    int ret = m_candriver->send_message(txBuffer, 1);
    // check if send_message was successful
    if (ret==0)
        {
            yError ("Unable to send message\n");
            return -1;
        }
    return 0;
}

//*****************************************************************/
int cDownloader::strain_stop_sampling    (int target_id)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    // Send transmission command to strain board
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x07;
    txBuffer[0].getData()[1]= 0x02;
    int ret = m_candriver->send_message(txBuffer, 1);
    // check if send_message was successful
    if (ret==0)
        {
            yError ("Unable to send message\n");
            return -1;
        }
    return 0;
}

//*****************************************************************/
int cDownloader::strain_calibrate_offset  (int target_id, unsigned int middle_val)
{
     // check if driver is running
     if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    int channel=0;
    int i=0;
    int ret =0;
    long error = 0;
    unsigned int measure = 0;
    unsigned int dac = 0;
    int cycle =0;
    int read_messages;

    for (channel=0; channel<6; channel++)
    {
        // Send read channel command to strain board
        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(3);
        txBuffer[0].getData()[0]= 0x0C;
        txBuffer[0].getData()[1]= channel;
        txBuffer[0].getData()[2]= 0;
        ret = m_candriver->send_message(txBuffer, 1);
        // check if send_message was successful
        if (ret==0)
            {
                yError ("Unable to send message\n");
                return -1;
            }
        //read adc
        read_messages = m_candriver->receive_message(rxBuffer,1);
        for (i=0; i<read_messages; i++)
        {
            if (rxBuffer[i].getData()[0]==0x0C)
                {
                    measure = rxBuffer[i].getData()[3]<<8 | rxBuffer[i].getData()[4];
                    break;
                }
        }

        //read dac
        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(2);
        txBuffer[0].getData()[0]= 0x0B;
        txBuffer[0].getData()[1]= channel;
        ret = m_candriver->send_message(txBuffer, 1);

        read_messages = m_candriver->receive_message(rxBuffer,1);
        for (i=0; i<read_messages; i++)
        {
            if (rxBuffer[i].getData()[0]==0x0B)
                {
                    dac = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                    break;
                }
        }

        error = long(measure) - long(middle_val);
        cycle=0;

        while (abs(error)>128 && cycle<0x03FF)
        {
            if (error>0) dac--;
            else         dac++;

            if (dac>0x3ff) dac = 0x3ff;
            if (dac<0)       dac = 0;

            // Send transmission command to strain board
            txBuffer[0].setId((2 << 8) + target_id);
            txBuffer[0].setLen(4);
            txBuffer[0].getData()[0]= 0x04;
            txBuffer[0].getData()[1]= channel;
            txBuffer[0].getData()[2]= dac >> 8;
            txBuffer[0].getData()[3]= dac & 0xFF;
            int ret = m_candriver->send_message(txBuffer, 1);

            //wait
            drv_sleep(3);

            // Send read channel command to strain board
            txBuffer[0].setId((2 << 8) + target_id);
            txBuffer[0].setLen(3);
            txBuffer[0].getData()[0]= 0x0C;
            txBuffer[0].getData()[1]= channel;
            txBuffer[0].getData()[2]= 0;
            ret = m_candriver->send_message(txBuffer, 1);
            // check if send_message was successful
            if (ret==0)
            {
                yError ("Unable to send message\n");
                return -1;
            }
            //read adc
            read_messages = m_candriver->receive_message(rxBuffer, 1);
            for (i=0; i<read_messages; i++)
            {
                if (rxBuffer[i].getData()[0]==0x0C)
                    {
                        measure = rxBuffer[i].getData()[3]<<8 | rxBuffer[i].getData()[4];
                        break;
                    }
            }

            error = long(measure) - long(middle_val);
            cycle++;
        }

    }

    return 0;
}

//*****************************************************************/
int cDownloader::get_serial_no       (int target_id, char* serial_no)
{
    int i;
    if (serial_no == NULL) return -1;

    memset (serial_no,0,8);

    // check if driver is running
    if (m_candriver == NULL)
    {
        yError ("Driver not ready\n");
        return -1;
    }

    for (i=0; i<board_list_size; i++)
    {
        if (board_list[i].pid==target_id &&
            board_list[i].type==icubCanProto_boardType__strain)
        {
            this->strain_get_serial_number(target_id,serial_no);
        }
    }

    return 0;
}

//*****************************************************************/
int cDownloader::get_board_info       (int target_id, char* board_info)
{
    int i;
    if (board_info == NULL) return -1;

    memset (board_info,0x3f,32);

    // check if driver is running
    if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    //riceve la risposta
    int read_messages = 0; //m_candriver->receive_message(rxBuffer, 10, 0);

    // Send command
    txBuffer[0].setId(build_id(ID_MASTER, target_id));
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= ICUBCANPROTO_BL_GET_ADDITIONAL_INFO;
    int ret = m_candriver->send_message(txBuffer, 1);
//    ret=0;
    // check if send_message was successful
    if (ret==0)
        {
            yError ("Unable to send message\n");
            return -1;
        }

    //pause
    //drv_sleep(10);

    //riceve la risposta
    read_messages = m_candriver->receive_message(rxBuffer, 64, 1);

    //One (or more) answers received
    int endString=0;
    int j=0;

    //reset the addtional info string
    for (j=0; j<31; j++) board_info[j]=0;

    //fills the additional info string
    for (i=0; i<read_messages; i++)
        {

#if 0
            if (rxBuffer[i].getData()[0]==ICUBCANPROTO_BL_GET_ADDITIONAL_INFO)
                {
                    fprintf(stderr, "%.4x ", rxBuffer[i].getId());
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[0]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[1]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[2]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[3]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[4]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[5]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[6]);
                    fprintf(stderr, "%.2x\n", rxBuffer[i].getData()[7]);
                }
#endif

            if (rxBuffer[i].getData()[0]==ICUBCANPROTO_BL_GET_ADDITIONAL_INFO && rxBuffer[i].getLen()==6)
                {
                    int part = rxBuffer[i].getData()[1];
                    for (j = 0; j< 4; j++)
                        {
                            int tmp=part*4+j;
                            board_info[tmp]=rxBuffer[i].getData()[j+2];
                            if (tmp>endString)
                                {
                                    endString=tmp;
                                }
                        }
                }
        }

    return 0;
}


//*****************************************************************/
int cDownloader::change_board_info(int target_id, char* board_info)
{
    // check if driver is running
    if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    // Send command
    int counter =0;
    int ret =0;
    int j = 0;

    for (counter = 0 ; counter < 8; counter++)
        {
            //do {}
            //while (CAN1_getStateTX () == 0) ;
            {
                txBuffer[0].getData()[0]= ICUBCANPROTO_BL_SET_ADDITIONAL_INFO;
                txBuffer[0].getData()[1]= counter;
                txBuffer[0].setId(build_id(ID_MASTER, target_id));
                txBuffer[0].setLen(6);
                for (j=0; j<4; j++)
                    txBuffer[0].getData()[2+j] = board_info[j+counter*4];
                ret |= m_candriver->send_message(txBuffer, 1);
            }
        }

    // check if send_message was successful
    if (ret==0)
        {
            yError ("Unable to send message\n");
            return -1;
        }

    //pause
    drv_sleep(500);

    // update the board list
    initschede();

    return 0;
}

//*****************************************************************/
int cDownloader::change_card_address(int target_id, int new_id, int board_type)
{
    int i = 0;

    // check if driver is running
    if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    switch (board_type)
    {
        case icubCanProto_boardType__strain:
        case icubCanProto_boardType__skin:
        case icubCanProto_boardType__mais:
        case icubCanProto_boardType__6sg:
            txBuffer[0].setId((0x02 << 8) + (ID_MASTER << 4) + target_id);
            txBuffer[0].setLen(2);
            txBuffer[0].getData()[0]= ICUBCANPROTO_POL_MC_CMD__SET_BOARD_ID;
            txBuffer[0].getData()[1]= new_id;
        break;

        case icubCanProto_boardType__dsp:
        case icubCanProto_boardType__pic:
        case icubCanProto_boardType__2dc:
        case icubCanProto_boardType__4dc:
        case icubCanProto_boardType__bll:
        case icubCanProto_boardType__2foc:
        case icubCanProto_boardType__jog:
            txBuffer[0].setId((ID_MASTER << 4) + target_id);
            txBuffer[0].setLen(2);
            txBuffer[0].getData()[0]= ICUBCANPROTO_POL_MC_CMD__SET_BOARD_ID;
            txBuffer[0].getData()[1]= new_id;
        break;

        default:
            yError ("Unknown board type for change of CAN address\n");
        return -1;

    }

    int ret = m_candriver->send_message(txBuffer, 1);

    // check if send_message was successful
    if (ret==0)
        {
            yError ("Unable to send message\n");
            return -1;
        }
        // pause
    drv_sleep(500);
    // update the board list
    initschede();

    return 0;
}
//*****************************************************************/

int cDownloader::initschede()
{
    int i;

    // check if driver is running
    if (m_candriver == NULL)
        {
            yError ("Driver not ready\n");
            return -1;
        }

    // Send command
    txBuffer[0].setId(build_id(ID_MASTER,ID_BROADCAST));
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BROADCAST;
    int ret = m_candriver->send_message(txBuffer, 1);

    // check if send_message was successful
    if (ret==0)
        {
            yError ("Unable to send message\n");
            return -1;
        }

    // pause
    drv_sleep(300);

    //riceve la risposta
    bool done=false;
    int read_messages=0;
    while(!done)
        {
            read_messages = m_candriver->receive_message(rxBuffer);
            //Timeout: no answers
            if (read_messages==0)
                {
                    yError ("No answers\n");
                    return -1;
                }

            //One (or more) answers received
            //Counts the number of the boards
            board_list_size = 0;
            for (i=0; i<read_messages; i++)
                {
                    ///////
#if 0
                    fprintf(stderr, "%.4x ", rxBuffer[i].getId());
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[0]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[1]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[2]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[3]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[4]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[5]);
                    fprintf(stderr, "%.2x ", rxBuffer[i].getData()[6]);
                    fprintf(stderr, "%.2x\n", rxBuffer[i].getData()[7]);
#endif
                    ////////////////
                    if ((rxBuffer[i].getData()[0]==ICUBCANPROTO_BL_BROADCAST) &&
                        ((rxBuffer[i].getLen()==4)||(rxBuffer[i].getLen()==5)))
                        board_list_size++;
                }

            if (board_list_size==0)
                {
                    //                    printf ("No Boards found\n");
                    // return -1;
                    static int times=0;
                    times++;
                    if (times==100)
                        {
                            yError ("No Boards found\n");
                            return -1;
                        }
                }
            else
                {
                    done=true;
                }

        }


    //Create the list of the boards
    if (board_list !=NULL) delete board_list;
    board_list= new sBoard[board_list_size];

    int j = 0;
    for (i=0; i<read_messages; i++)
        {
            if ((rxBuffer[i].getData()[0]==ICUBCANPROTO_BL_BROADCAST) &&
                ((rxBuffer[i].getLen()==4)||(rxBuffer[i].getLen()==5)))   //old board firmware (backward compatibility)
                {
                    board_list[j].pid     = (rxBuffer[i].getId() >> 4) & 0x0F;
                    board_list[j].type    = rxBuffer[i].getData()[1];
                    board_list[j].version = rxBuffer[i].getData()[2];
                    board_list[j].release = rxBuffer[i].getData()[3];
                    board_list[j].status  = BOARD_RUNNING;
                    board_list[j].selected  = false;
                    board_list[j].eeprom =false;
                    memset (board_list[j].serial,  0, 8);
                    memset (board_list[j].add_info,  0, 32);
                    if (rxBuffer[i].getLen()==4)
                        board_list[j].build = 0;
                    else
                        board_list[j].build = rxBuffer[i].getData()[4];
                    j++;
                }
        }

    for (i=0; i<board_list_size; i++)
        {
            char board_info [32];
            get_board_info       (board_list[i].pid, board_info);
            strcpy (board_list[i].add_info,  board_info);
            //pause
            drv_sleep(10);
        }

    for (i=0; i<board_list_size; i++)
        {
            char serial_no [32];
            get_serial_no       (board_list[i].pid, serial_no);
            strcpy (board_list[i].serial,  serial_no);
            //pause
            drv_sleep(10);
        }

    printf ("CONNECTED: %d Boards found on bus %d\n", board_list_size, this->canbus_id);
    return 0;
}


//*****************************************************************/
int cDownloader::get_canbus_id()
{
    return canbus_id;
}

//*****************************************************************/

int cDownloader::startscheda(int board_pid, bool board_eeprom, int board_type)
{
    // check if driver is running
    if (m_candriver == NULL)
        {
            yError ("START_CMD: Driver not ready\n");
            return -1;
        }

    switch (board_type)
    {
    case icubCanProto_boardType__dsp:
    case icubCanProto_boardType__pic:
    case icubCanProto_boardType__2dc:
    case icubCanProto_boardType__4dc:
    case icubCanProto_boardType__bll:
        {
        // Send command
        txBuffer[0].setId(build_id(ID_MASTER,board_pid));
        txBuffer[0].setLen(1);
        txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BOARD;

        //makes the first jump
        m_candriver->send_message(txBuffer, 1);
        drv_sleep(250);
        }
        break;
    case icubCanProto_boardType__skin:
    case icubCanProto_boardType__strain:
    case icubCanProto_boardType__mais:
    case icubCanProto_boardType__2foc:
    case icubCanProto_boardType__6sg:
    case icubCanProto_boardType__jog:
    case icubCanProto_boardType__unknown:
        {
        // Send command
        txBuffer[0].setId(build_id(ID_MASTER,board_pid));
        txBuffer[0].setLen(2);
        txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BOARD;
        txBuffer[0].getData()[1]= (int) board_eeprom;

        //makes the first jump
        m_candriver->send_message(txBuffer, 1);
        drv_sleep(1500);
        }
        break;
    }
    int ret = m_candriver->send_message(txBuffer, 1);

    // check if send_message was successful
    if (ret==0)
        {
            yError ("START_CMD: Unable to send message\n");
            return -1;
        }

    //pause
     drv_sleep(500);

    // riceve la risposta
    int read_messages = m_candriver->receive_message(rxBuffer);


    //One (or more) answers received
    for (int i=0; i<read_messages; i++)
        {
            if (rxBuffer[i].getData()[0]==ICUBCANPROTO_BL_BOARD &&
                (((rxBuffer[i].getId() >> 4) & 0x0F) == board_pid) &&
                (((rxBuffer[i].getId() >> 8) & 0x07) == ICUBCANPROTO_CLASS_BOOTLOADER))
                {
                    //received ACK from board
                    //printf ("START_CMD: ACK received from board: %d\n", board_pid);
                    return 0;
                }
        }
  // return 0;  //DEBUG

    //ERROR
    yError ("START_CMD: No ACK received from board %d\n", board_pid);
    return -1;

}

//*****************************************************************/

int cDownloader::stopscheda(int board_pid)
{
    // check if driver is running
    if (m_candriver == NULL)
        {
            yError ("STOP_CMD: Driver not ready\n");
            return -1;
        }

    // Send command
    txBuffer[0].setId(build_id(ID_MASTER,board_pid));
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= ICUBCANPROTO_BL_END;
    int ret = m_candriver->send_message(txBuffer, 1);

    // check if send_message was successful
    if (ret==0)
        {
            yError ("STOP_CMD: Unable to send message\n");
            return -1;
        }

    //pause
    drv_sleep(5);

    // riceve la risposta
    int read_messages = m_candriver->receive_message(rxBuffer);

    //One (or more) answers received
    for (int i=0; i<read_messages; i++)
        {
            if (rxBuffer[i].getData()[0]==ICUBCANPROTO_BL_END &&
                (((rxBuffer[i].getId() >> 4) & 0x0F) == board_pid || board_pid == 15 ) &&
                (((rxBuffer[i].getId() >> 8) & 0x07) == ICUBCANPROTO_CLASS_BOOTLOADER))
                {
                    //received ACK from board
                    //printf ("STOP_CMD: ACK received from board: %d\n", board_pid);
                    return 0;
                }
        }

    //ERROR
    yError ("TOP_CMD: No ACK received from board %d\n", board_pid);
    return -1;
}

//*****************************************************************/

int getvalue(char* line, int len)
{
    char hexconv_buffer[5];
    memset (hexconv_buffer, '\0', sizeof(hexconv_buffer) );
    strncpy(hexconv_buffer,line,len);
    return axtoi (hexconv_buffer);
}

//*****************************************************************/
int cDownloader::verify_ack(int command, CanBuffer rxBuffer,int read_messages)
{
    int i,k;

/*
    for(int m=0;m<read_messages;m++)
        {
            fprintf(stderr, "%4x %d %d %d\n",
                    rxBuffer[m].getId(),
                    rxBuffer[m].getLen(),
                    rxBuffer[m].getData()[0],
                    rxBuffer[m].getData()[1]);
        }
*/

    for (i=0; i<board_list_size; i++)
        {
            if (board_list[i].selected==true)
                if (board_list[i].status == BOARD_WAITING ||
                    board_list[i].status == BOARD_DOWNLOADING)
                    {
                        board_list[i].status = BOARD_WAITING_ACK;

                        for (k=0; k<read_messages; k++)
                            {
                                if ((rxBuffer[k].getData()[0]==command) &&
                                    (rxBuffer[k].getLen() == 2) &&
                                    (rxBuffer[k].getData()[1]==1))
                                    {
                                        if (board_list[i].pid == get_src_from_id(rxBuffer[k].getId()))
                                            board_list[i].status=BOARD_DOWNLOADING;
                                    }
                            }
                    }
        }

    for (i=0; i<board_list_size; i++)
        {
            if (board_list[i].selected==true && board_list[i].status == BOARD_WAITING_ACK)
                {
                  //@@@@  board_list[i].status=BOARD_ERR;
                    return -1;
                }
        }
    return 0;
}



//*****************************************************************/
// Return values:
// 0  one line downloaded, continuing the download...
// 1  Current downloading, everything OK
// -1 Fatal error

int cDownloader::download_motorola_line(char* line, int len, int board_pid)
{
    static double now;
    static double prev;

    now=Time::now();
    double dT=now-prev;
    prev=now;

    // fprintf(stderr, "dT:%.2lf [ms]\n", dT*1000);

    char  sprsRecordType=0;
    unsigned long int  sprsChecksum=0;
    int  sprsMemoryType=1;
    long unsigned int  sprsAddress;
    int  sprsLength;
    int  i,j,k;
    int ret =0;
    int read_messages=0;

    for (i=2; i<len; i=i+2)
        {
            int value= getvalue(line+i,2);
            sprsChecksum+= value;
            // printf ("chk: %d %d\n", value, sprsChecksum);

        }

    if ((sprsChecksum & 0xFF) == 0xFF)
        {
            //  printf ("Checksum OK\n");
        }
    else
        {
            yError ("Failed Checksum\n");
            return -1;
        }

    //state: WAIT
    if (!(line[0] == 'S'))
        {
            yError ("start tag character not found\n");
            return -1;
        }
    i=1;

    //state: TYPE
    sprsRecordType=char(*(line+i));
    i++;

    //state: LENGTH
    sprsLength=getvalue(line+i,2)-4-1;
    i=i+2;

    switch (sprsRecordType)
        {
        case SPRS_TYPE_0:

            return 0;

        case SPRS_TYPE_3:

            //state: ADDRESS
            sprsAddress=getvalue(line+i,4);
            i+=4;

            if (sprsAddress==0x0020)
                sprsMemoryType=1;
            else
                sprsMemoryType=0;

            sprsAddress=getvalue(line+i,4);
            i+=4;

            //state: SEND
            txBuffer[0].setId(build_id(ID_MASTER,board_pid));
            txBuffer[0].setLen(5);
            txBuffer[0].getData()[0]= ICUBCANPROTO_BL_ADDRESS;
            txBuffer[0].getData()[1]= sprsLength;
            txBuffer[0].getData()[2]= (unsigned char) ((sprsAddress) & 0x00FF);
            txBuffer[0].getData()[3]= (unsigned char) ((sprsAddress>>8) & 0x00FF);
            txBuffer[0].getData()[4]= sprsMemoryType;

            //send here
            ret = m_candriver->send_message(txBuffer,1);

            // check if send_message was successful
            if (ret==0)
                {
                    yError ("Unable to send message\n");
                    return -1;
                }

            // pause
            // drv_sleep(5);

            //prepare packet
            int tmp, rest;
            if ((sprsLength%6) == 0)
                {
                    tmp=sprsLength / 6;
                    rest=6;
                }
            else
                {
                    tmp=sprsLength / 6 + 1;
                    rest=sprsLength % 6;
                }

            for (j=1; j<= tmp; j++)
                {
                    txBuffer[0].getData()[0]=ICUBCANPROTO_BL_DATA;
                    if (j<tmp) txBuffer[0].setLen(7);
                    else txBuffer[0].setLen(rest+1);

                    for (k=1; k<=6; k++)
                        {
                            txBuffer[0].getData()[k] = getvalue(line+i+((k-1)*2+((j-1)*12)),2);
                        }

                    //send here
                    ret = m_candriver->send_message(txBuffer,1);

                    // check if send_message was successful
                    if (ret==0)
                        {
                            yError ("Unable to send message\n");
                            return -1;
                        }

                    //pause
                    //  drv_sleep(5);
                }

            //pause
            // drv_sleep(10);

            //receive one ack for the whole line
            double passed;
            passed=Time::now()-now;
            // fprintf(stderr, "Passed:%.2lf [ms]\n", passed*1000);
            read_messages = m_candriver->receive_message(rxBuffer,nSelectedBoards);
            // fprintf(stderr, "%u\n", read_messages);
            //   fprintf(stderr, "Skipping ack\n");
            //return verify_ack(ICUBCANPROTO_BL_DATA, rxBuffer, read_messages);
            return 0;
            break;
        case SPRS_TYPE_7:

            //state: SEND
            txBuffer[0].setId(build_id(ID_MASTER,board_pid));
            txBuffer[0].setLen(5);
            txBuffer[0].getData()[0]= ICUBCANPROTO_BL_START;
            txBuffer[0].getData()[4]= getvalue(line+i,2); i+=2;
            txBuffer[0].getData()[3]= getvalue(line+i,2); i+=2;
            txBuffer[0].getData()[2]= getvalue(line+i,2); i+=2;
            txBuffer[0].getData()[1]= getvalue(line+i,2);

            //send here
            ret = m_candriver->send_message(txBuffer, 1);

            // check if send_message was successful
            if (ret==0)
                {
                    yError ("Unable to send message\n");
                    return -1;
                }

            //pause
            drv_sleep(10+5);

            // riceve la risposta
            read_messages = m_candriver->receive_message(rxBuffer);
            verify_ack(ICUBCANPROTO_BL_START, rxBuffer, read_messages);
            return 0;

            break;


        default:
            yError ("wrong format tag character %c (hex:%X)\n", sprsRecordType, sprsRecordType);
            return -1;

            break;
        }

    yError ("Can't reach here!\n");
    return -1;
}

//*****************************************************************/
// This function read one line of the hexintel file and send it to a board using the correct protocol
// Return values:
// 0  one line downloaded, continuing the download...
// 1  Current downloading, everything OK
// -1 Fatal error

int cDownloader::download_hexintel_line(char* line, int len, int board_pid, bool eeprom, int board_type)
{
    char               sprsRecordType=0;
    unsigned int       sprsState;
    unsigned long int  sprsChecksum=0;
    int                sprsMemoryType=0;
    long unsigned int  sprsAddress=0;
    int                sprsLength=0;
    unsigned int       sprsData[50];
    int  i,j,k;
    int ret =0;
    int read_messages=0;

    for (i=1; i<len; i=i+2)
    {
        int value= getvalue(line+i,2);
        sprsChecksum+= value;
    //    printf ("chk: %d %d\n", value, sprsChecksum);
    }
    sprsChecksum = (sprsChecksum & 0xFF);
    if (sprsChecksum == 0x00)
    {
      //    printf ("Checksum OK\n");
    }
    else
    {
      yError ("Failed Checksum\n");
      return -1;
     }

    sprsState=SPRS_STATE_WAIT;
    // Init of parsing process
    do
    {
        switch (sprsState)
        {
        case SPRS_STATE_WAIT:
            //check the first character of the line
            if (!(line[0] == ':'))
            {
                printf ("start tag character not found\n");
                return -1;
            }
            else
            {
                sprsState=SPRS_STATE_LENGTH;
                i=1;
            }
        break;
        case SPRS_STATE_TYPE:

            sprsRecordType=char(*(line+i+1));//char(getvalue(line+i,2));//(char)(*(line+i));
            i=i+2;
            if (sprsLength==0)
                sprsState=SPRS_STATE_CHECKSUM;
            else
                sprsState=SPRS_STATE_DATA;
        break;
        case SPRS_STATE_LENGTH:

            sprsLength= getvalue(line+i,2);
            i=i+2;
            sprsState=SPRS_STATE_ADDRESS;
        break;
        case SPRS_STATE_ADDRESS:

            sprsAddress=getvalue(line+i,4);
            i=i+4;
            sprsState=SPRS_STATE_TYPE;
        break;
        case SPRS_STATE_DATA:

            switch (sprsRecordType)
            {
            case SPRS_TYPE_0:

                for (k=0;k<sprsLength;k++)
                {
                    sprsData[k]=getvalue(line+i,2);
                    i=i+2;
                }
            break;

            case SPRS_TYPE_4:

                sprsPage=getvalue(line+i,4);
                i=i+4;
            break;
            }
            sprsState=SPRS_STATE_CHECKSUM;
        break;
        case SPRS_STATE_CHECKSUM:

            sprsState=SPRS_STATE_WAIT;
            if (sprsChecksum==0)
            {
                switch (sprsRecordType)
                {
                case SPRS_TYPE_0:

                    //if (sprsPage==0)
                    {
                        //SEND
                        txBuffer[0].setId(build_id(ID_MASTER,board_pid));
                        txBuffer[0].setLen(7);
                        txBuffer[0].getData()[0]= ICUBCANPROTO_BL_ADDRESS;
                        txBuffer[0].getData()[1]= sprsLength;
                        txBuffer[0].getData()[2]= (unsigned char) ((sprsAddress) & 0x00FF);
                        txBuffer[0].getData()[3]= (unsigned char) ((sprsAddress>>8) & 0x00FF);
                        txBuffer[0].getData()[4]= sprsMemoryType;
                        txBuffer[0].getData()[5]= (unsigned char) ((sprsPage) & 0x00FF);
                        txBuffer[0].getData()[6]= (unsigned char) ((sprsPage >>8) & 0x00FF);
                    }
                    //send here
                    ret = m_candriver->send_message(txBuffer,1);
                    // check if send_message was successful
                    if (ret==0)


                    {
                        yError ("Unable to send message\n");
                        return -1;
                    }
                    //pause
                    drv_sleep(10);

                    //prepare packet
                    int tmp, rest;
                    if ((sprsLength%6) == 0)
                        {
                            tmp=sprsLength / 6;
                            rest=6;
                        }
                    else
                        {
                            tmp=sprsLength / 6 + 1;
                            rest=sprsLength % 6;
                        }

                    for (j=1; j<= tmp; j++)
                    {
                        txBuffer[0].getData()[0]=ICUBCANPROTO_BL_DATA;
                        if (j<tmp) txBuffer[0].setLen(7);
                        else txBuffer[0].setLen(rest+1);

                        for (k=1; k<=6; k++)
                            {
                                txBuffer[0].getData()[k] = sprsData[(k-1)+((j-1)*6)];//getvalue(line+i+((k-1)*2+((j-1)*12)),2);
                            }

                        //send here
                        ret = m_candriver->send_message(txBuffer,1);

                        // check if send_message was successful
                        if (ret==0)
                            {
                                yError ("Unable to send message\n");
                                return -1;
                            }
                        //pause
                        drv_sleep(5);
                    }
                    //receive one ack for the whole line
                    read_messages = m_candriver->receive_message(rxBuffer,nSelectedBoards, 10);
                    ret=verify_ack(ICUBCANPROTO_BL_DATA, rxBuffer, read_messages);
                    //DEBUG

    //                return 0;
                    return ret;
                break;

                case SPRS_TYPE_1:

                    //SEND
                    txBuffer[0].setId(build_id(ID_MASTER,board_pid));
                    txBuffer[0].setLen(5);
                    txBuffer[0].getData()[0]= ICUBCANPROTO_BL_START;
                    txBuffer[0].getData()[1]= 0;
                    txBuffer[0].getData()[2]= 0;
                    txBuffer[0].getData()[3]= 0;
                    txBuffer[0].getData()[4]= 0;

                    //send here
                    ret = m_candriver->send_message(txBuffer,1);
                    // check if send_message was successful
                    if (ret==0)
                    {
                        yError ("Unable to send message\n");
                        return -1;
                    }
                    //pause
                    drv_sleep(5);
                    //receive the ack from the board
                    read_messages = m_candriver->receive_message(rxBuffer);
                    ret=verify_ack(ICUBCANPROTO_BL_START, rxBuffer, read_messages);
                    //DEBUG
                    //return 0;
                    return ret;

                break;
            //    case SPRS_TYPE_4:
            //        break;
            //        return 0;
                default:
                    return 0;
                }
            } //end if
            else
            {
            yError ("Checksum Error\n");
            }
        break;
        } //end switch
    }
    while(true);

    yError ("Can't reach here!\n");
    return -1;
}
//*****************************************************************/

int cDownloader::open_file(std::string file)
{
    progress=0;
    filestr.close();
    filestr.clear();
    filestr.open (file.c_str(), fstream::in);
    if (!filestr.is_open())
        {
            yError ("Error opening file!\n");
            return -1;
        }

    file_length=0;
    char buffer[256];
    while (!filestr.eof())
        {
            filestr.getline (buffer,256);
            file_length++;
        }
    //printf ("length: %d\n",file_length);

    filestr.close();
    filestr.clear();
    filestr.open (file.c_str(), fstream::in);
    if (!filestr.is_open())
        {
            yError ("Error opening file!\n");
            return -1;
        }

    return 0;
}

//*****************************************************************/
// Return values:
// 0  Download terminated, everything OK
// 1  Current downloading, everything OK
// -1 Fatal error in sending one command
int cDownloader::download_file(int board_pid, int download_type, bool board_eeprom)
{

    if (!filestr.is_open())
        {
            yError ("File not open!\n");
            return -1;
        }

    char buffer[256];
    int ret = 0;

    nSelectedBoards=0;
    int i=0;
    for (i=0; i<board_list_size; i++)
        {
            if (board_list[i].selected==true)
                nSelectedBoards++;
        }

    if (!filestr.eof())
        {
            filestr.getline (buffer,256);

            //avoid to download empty lines
            if (strlen(buffer)!=0)
                {
                    switch (download_type)
                    {
                        case icubCanProto_boardType__dsp:
                        case icubCanProto_boardType__2dc:
                        case icubCanProto_boardType__4dc:
                        case icubCanProto_boardType__bll:
                             ret = download_motorola_line(buffer, strlen(buffer), board_pid);
                        break;
                        case icubCanProto_boardType__pic:
                        case icubCanProto_boardType__skin:
                        case icubCanProto_boardType__strain:
                        case icubCanProto_boardType__mais:
                        case icubCanProto_boardType__2foc:
                        case icubCanProto_boardType__jog:
                        case icubCanProto_boardType__6sg:
                             ret = download_hexintel_line(buffer, strlen(buffer), board_pid, board_eeprom, download_type);

                        break;
                        case icubCanProto_boardType__unknown:
                        default:
                             ret =-1;
                        break;
                    }
                    if (ret != 0)
                        {
                            // fatal error during download, abort
                            filestr.close();
                            return -1;
                        }
                }

            progress++;
            //everything OK
            return 1;
        }
    else
        {
            filestr.close();
            filestr.clear();
            //download terminated OK
            return 0;
        }
}
