// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 RobotCub Consortium
 * Author: Marco Maggiali, Marco Randazzo, Alessandro Scalzo, Marco Accame
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "driver.h"
#include "downloader.h"
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <stdlib.h> //added for abs
#include <string.h>

#include <canProtocolLib/iCubCanProtocol.h>
#include <canProtocolLib/iCubCanProto_types.h>

#include "strain.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace std;


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

cDownloader::cDownloader(bool verbose)
{
    _verbose = verbose;
    board_list = NULL;
    board_list_size = 0;
    connected = false;
    m_idriver=NULL;
    sprsPage=0;
    set_canbus_id(-1);
}

void cDownloader::set_verbose(bool verbose)
{
    _verbose = verbose;
}

void cDownloader::set_external_logger(void *caller, void (*logger)(void *, const std::string &))
{
    _externalLoggerFptr = logger;
    _externalLoggerCaller = caller;
}

void cDownloader::Log(const std::string &msg)
{
    yDebug() << "$" << msg;

    if(NULL != _externalLoggerFptr)
    {
        _externalLoggerFptr(_externalLoggerCaller, "$ " + msg);
    }
}


//*****************************************************************/

int cDownloader::stopdriver()
{
    if (m_idriver !=NULL)
        {
#if defined(DOWNLOADER_USE_IDRIVER2)
            txBuffer.resize(0);
            rxBuffer.resize(0);
#else
            m_idriver->destroyBuffer(txBuffer);
            m_idriver->destroyBuffer(rxBuffer);
#endif
            delete m_idriver;
            m_idriver=NULL;
            connected = false;
        }
    return 0;
}

//*****************************************************************/

int cDownloader::initdriver(Searchable &config, bool verbose)
{
    _verbose = verbose;

    set_external_logger(NULL, NULL);

    int ret = 0; // 0 is ok, -1 is failure, -2 is retry ...
    if (m_idriver !=NULL)
        {
            delete m_idriver;
            m_idriver=NULL;
            connected = false;
        }

    int tmp = 0;

    if (config.find("device").asString()=="ETH")
    {
#if defined(DOWNLOADER_USE_IDRIVER2)
    m_idriver = new eDriver2;
#else
        m_idriver = new eDriver;
#endif
        tmp = config.check("canid")?config.find("canid").asInt():CanPacket::everyCANbus;
        if((1 != tmp) && (2 != tmp))
        {
            tmp = CanPacket::everyCANbus;
        }
    }
    else
    {
#if defined(DOWNLOADER_USE_IDRIVER2)
        m_idriver = new cDriver2;
#else
        m_idriver = new cDriver;
#endif
        tmp = config.check("canDeviceNum")?config.find("canDeviceNum").asInt():99;
    }

    if (0 != (ret = m_idriver->init(config, _verbose)))
        {
            if (m_idriver)
                {
                    delete m_idriver;
                    m_idriver=NULL;
                    connected = false;
                }
            return ret;
        }

    set_canbus_id(tmp);


#if defined(DOWNLOADER_USE_IDRIVER2)
    txBuffer.resize(1);
    txBuffer[0].setCanBus(tmp);
    rxBuffer.resize(MAX_READ_MSG);
    for(int i=0; i<MAX_READ_MSG; i++)
    {
        rxBuffer[i].setCanBus(tmp);
    }
#else
    txBuffer=m_idriver->createBuffer(1);
    rxBuffer=m_idriver->createBuffer(MAX_READ_MSG);
#endif
    connected = true;


    return ret;
}

//*****************************************************************/
int cDownloader::strain_save_to_eeprom  (int bus, int target_id, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     // Send transmission command to strain board
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= 0x09;
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }

    drv_sleep(5);

    int read_messages = m_idriver->receive_message(rxBuffer,1);
    for (int i=0; i<read_messages; i++)
    {
    if (rxBuffer[i].getData()[0]==0x09 &&
        rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
        {
            if(rxBuffer[i].getData()[1]!=0)
            {
                 if(_verbose) yInfo ("Data has been saved in EEprom correctly\n");
                return 1;
            }
            else
            {
                 if(_verbose) yError ("Error in data saving in EEprom \n");
                 return 0;
            }
        }
    }
    if(_verbose) yError ("Save_to_eeprom didn't receive answer...maybe strain firmware is obsolete \n");
    return -1;

}

//*****************************************************************/
//int cDownloader::sg6_obsolete_get_amp_gain      (int bus, int target_id, char channel, unsigned int& gain1, unsigned int& gain2 )
//{
//#if 1
//    yError ("sg6_obsolete_get_amp_gain() is obsolete. \n");
//    return -1;
//#else
//     // check if driver is running
//     if (m_idriver == NULL)
//        {
//            if(_verbose) yError ("Driver not ready\n");
//            return -1;
//        }
//#error command ID 0x1D is not available anymore
//     // Send read gain command to strain board
//     txBuffer[0].setId((2 << 8) + target_id);
//     txBuffer[0].setLen(2);
//     txBuffer[0].getData()[0]= 0x1D;
//     txBuffer[0].getData()[1]= channel;
//     set_bus(txBuffer[0], bus);
//     int ret = m_idriver->send_message(txBuffer, 1);
//     // check if send_message was successful
//     if (ret==0)
//     {
//         if(_verbose) yError ("Unable to send message\n");
//         return -1;
//     }

//     drv_sleep(3);

//     //read gain
//     int read_messages = m_idriver->receive_message(rxBuffer,1);
//     for (int i=0; i<read_messages; i++)
//        {
//          if (rxBuffer[i].getData()[0]==0x1D &&
//             rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
//             {
//                 int ret_channel = rxBuffer[i].getData()[1];
//                 if (ret_channel == channel)
//                 {
//                    gain1 = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
//                    gain2 = rxBuffer[i].getData()[4]<<8 | rxBuffer[i].getData()[5];
//                    return 0;
//                 }
//                 else
//                 {
//                    if(_verbose) yError ("sg6_get_amp_gain : invalid response\n");
//                    return -1;
//                 }
//             }
//        }

//     return -1;
//#endif
//}

//*****************************************************************/
//int cDownloader::sg6_obsolete_set_amp_gain      (int bus, int target_id, char channel, unsigned int  gain1, unsigned int  gain2 )
//{
//#if 1
//    yError ("sg6_obsolete_set_amp_gain() is obsolete. \n");
//    return -1;
//#else
//     // check if driver is running
//     if (m_idriver == NULL)
//        {
//            if(_verbose) yError ("Driver not ready\n");
//            return -1;
//        }
//#error command ID 0x1E is not available anymore
//    //set amp gain
//    txBuffer[0].setId((2 << 8) + target_id);
//    txBuffer[0].setLen(6);
//    txBuffer[0].getData()[0]= 0x1E;
//    txBuffer[0].getData()[1]= channel;
//    txBuffer[0].getData()[2]= gain1 >> 8;
//    txBuffer[0].getData()[3]= gain1 & 0xFF;
//    txBuffer[0].getData()[4]= gain2 >> 8;
//    txBuffer[0].getData()[5]= gain2 & 0xFF;
//    set_bus(txBuffer[0], bus);
//    int ret = m_idriver->send_message(txBuffer, 1);

//     return 0;
//#endif
//}

//*****************************************************************/
int cDownloader::strain_get_adc(int bus, int target_id, char channel, unsigned int& adc, int type, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     // Send read channel command to strain board
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(3);
     txBuffer[0].getData()[0]= 0x0C;
     txBuffer[0].getData()[1]= channel;
     txBuffer[0].getData()[2]= type;
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     // check if send_message was successful
     if (ret==0)
         {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }

     drv_sleep(3);

     //read adc
     int read_messages = m_idriver->receive_message(rxBuffer,1);
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
                    if(_verbose) yError ("strain_get_adc : invalid response\n");
                    return -1;
                 }
             }
        }

     return -1;
}

//*****************************************************************/
int cDownloader::strain_get_offset(int bus, int target_id, char channel, unsigned int& offset, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

    //read dac
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x0B;
    txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);

    clean_rx();
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);

    drv_sleep(3);

    int read_messages = m_idriver->receive_message(rxBuffer,1);
    for (int i=0; i<read_messages; i++)
    {
        if (rxBuffer[i].getData()[0]==0x0B &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                int ret_channel = rxBuffer[i].getData()[1] & 0x0f;
                if (channel==ret_channel)
                {
                    offset = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                    return 0;
                }
                else
                {
                    if(_verbose) yError ("strain_get_offset : invalid response\n");
                    return -1;
                }
                
            }
    }

    return -1;
}
//*****************************************************************/
int cDownloader::strain_get_calib_bias     (int bus, int target_id, char channel, signed int& bias, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

      //read current bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x14;
     txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);

     int read_messages = m_idriver->receive_message(rxBuffer,1);
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

// this funtion set the calib bias = to the negative of adc for every channel, hence it must be only for strain_regset_inuse
//*****************************************************************/
int cDownloader::strain_set_calib_bias     (int bus, int target_id, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     //set calib bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x13;
     txBuffer[0].getData()[1]= 1;
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_set_calib_bias     (int bus, int target_id, char channel, int bias, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     //set calib bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(5);
     txBuffer[0].getData()[0]= 0x13;
     txBuffer[0].getData()[1]= 2;
     txBuffer[0].getData()[2]= ((regset << 4) & 0xf0) | (channel & 0x0f);
     txBuffer[0].getData()[3]= bias >> 8;
     txBuffer[0].getData()[4]= bias & 0xFF;

     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}

// this funtion set the calib bias = 0 for every channel, hence it must be only for strain_regset_inuse
//*****************************************************************/
int cDownloader::strain_reset_calib_bias (int bus, int target_id, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     //reset calib bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x13;
     txBuffer[0].getData()[1]= 0;
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_get_curr_bias     (int bus, int target_id, char channel, signed int& bias, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

      //read current bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x16;
     txBuffer[0].getData()[1]= channel;
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);

     int read_messages = m_idriver->receive_message(rxBuffer,1);
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
int cDownloader::strain_set_curr_bias     (int bus, int target_id, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     //set curr bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x15;
     txBuffer[0].getData()[1]= 1;
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

      return 0;
}

//*****************************************************************/
int cDownloader::strain_set_curr_bias     (int bus, int target_id, char channel, int bias, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
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

     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_reset_curr_bias     (int bus, int target_id, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     //reset curr bias
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x15;
     txBuffer[0].getData()[1]= 0;
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

      return 0;
}

//*****************************************************************/
int cDownloader::strain_set_serial_number (int bus, int target_id, const char* serial_number, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
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
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
    drv_sleep(5);

    return 0;
}

//*****************************************************************/
int cDownloader::strain_get_serial_number (int bus, int target_id, char* serial_number, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

      //read serial number
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(1);
     txBuffer[0].getData()[0]= 0x1A;

     clean_rx();
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);

     drv_sleep(5);

      int read_messages = m_idriver->receive_message(rxBuffer,1);
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
int cDownloader::strain_get_eeprom_saved (int bus, int target_id, bool* status, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

      //read eeprom status (saved/not saved)
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(1);
     txBuffer[0].getData()[0]= 0x1B;

     clean_rx();
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);

     drv_sleep(5);

     int read_messages = m_idriver->receive_message(rxBuffer,1);
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
int cDownloader::strain_get_matrix_gain     (int bus, int target_id, unsigned int& gain, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     // read matrix gain
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(1);
     txBuffer[0].getData()[0]= 0x12;

     clean_rx();
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);

     drv_sleep(5);

     int read_messages = m_idriver->receive_message(rxBuffer,1);
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
//int cDownloader::strain_set_matrix(int bus, int target_id, int matrix, string *errorstring)
//{
//    if(0 == matrix)
//    {
//        return 0;
//    }

//    if(NULL != errorstring)
//    {
//        *errorstring += "cDownloader::strain_set_matrix() cannot set a non zero matrix number";
//        //*errorstring += std::to_string(matrix); // c++ 11 ...........
//    }
//    return -1;
//}

//*****************************************************************/

//int cDownloader::strain_get_matrix(int bus, int target_id, int &matrix, string *errorstring)
//{
//    matrix = 0;
//    return 0;
//}


int cDownloader::strain_set_regulationset(int bus, int target_id, int regset, int regsetmode, string *errorstring)
{
#if 0
    return 0;
#else

    // check if driver is running
    if (m_idriver == NULL)
       {
           if(_verbose) yError ("Driver not ready\n");
           return -1;
       }

    if(strain_regset_inuse == regset)
    {
        return -1;
    }

    // set regulation set
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x3D;
    txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (regsetmode & 0x0f);

    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
    drv_sleep(5);

    return 0;

#endif
}

int cDownloader::strain_get_regulationset(int bus, int target_id, int &regset, const int regsetmode, string *errorstring)
{
#if 0
    regset = strain_regset_one;
    return 0;
#else
    // send the command

    // check if driver is running
    if (m_idriver == NULL)
       {
           if(_verbose) yError ("Driver not ready\n");
           return -1;
       }

   // read reg set
   txBuffer[0].setId((2 << 8) + target_id);
   txBuffer[0].setLen(2);
   txBuffer[0].getData()[0]= 0x3E;
   txBuffer[0].getData()[1]= (regsetmode & 0x0f);

   clean_rx();
   set_bus(txBuffer[0], bus);
   int ret = m_idriver->send_message(txBuffer, 1);

   drv_sleep(3);

   int read_messages = m_idriver->receive_message(rxBuffer,1);
   for (int i=0; i<read_messages; i++)
   {
       if (rxBuffer[i].getData()[0]== 0x3E &&
           rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
           {
               int rregset = (rxBuffer[i].getData()[1] >> 4) & 0x0F;
               int rregsetmode = (rxBuffer[i].getData()[1]) & 0x0F;
               if (rregsetmode == regsetmode)
               {
                   regset = rregset;
                   return 0;
               }
               else
               {
                   if(_verbose) yError ("strain_get_regulationset : invalid response\n");
                   return -1;
               }

           }
   }

   return -1;
#endif
}

//*****************************************************************/
int cDownloader::strain_set_matrix_gain     (int bus, int target_id, unsigned int  gain, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     // set matrix gain
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x11;
     txBuffer[0].getData()[1]= gain;

     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}

int cDownloader::strain_set_amplifier_regs(int bus, int target_id, unsigned char channel, const strain2_ampl_regs_t &ampregs, int regset, string *errorstring)
{
    // check if driver is running
    if (m_idriver == NULL)
    {
        if(_verbose) yError ("Driver not ready\n");
        return -1;
    }

    // not for normal strain

    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(8);
    txBuffer[0].getData()[0]= 0x2B;
    txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
    txBuffer[0].getData()[2]= ampregs.data[0]; // lsb of gd
    txBuffer[0].getData()[3]= ampregs.data[1]; // msb of gd
    txBuffer[0].getData()[4]= ampregs.data[2];
    txBuffer[0].getData()[5]= ampregs.data[3]; // vc0
    txBuffer[0].getData()[6]= ampregs.data[4];
    txBuffer[0].getData()[7]= ampregs.data[5];
    set_bus(txBuffer[0], bus);
    //yDebug("strain_set_amplifier_regs() is sending: [%x, %x, %x, %x, %x, %x, %x, %x]", txBuffer[0].getData()[0], txBuffer[0].getData()[1], txBuffer[0].getData()[2], txBuffer[0].getData()[3], txBuffer[0].getData()[4], txBuffer[0].getData()[5], txBuffer[0].getData()[6], txBuffer[0].getData()[7]);

    m_idriver->send_message(txBuffer, 1);

    // i wait some 10 ms
    yarp::os::Time::delay(0.010);


    return 0;
}


int cDownloader::strain_get_amplifier_regs(int bus, int target_id, unsigned char channel, strain2_ampl_regs_t &ampregs, int regset, string *errorstring)
{
    // check if driver is running
    if (m_idriver == NULL)
    {
        if(_verbose) yError ("Driver not ready\n");
        return -1;
    }

    // not for normal strain

    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x2A;
    txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);;
    set_bus(txBuffer[0], bus);

    //yDebug("strain_get_amplifier_regs() is sending: [%x, %x, %x, %x, %x, %x, %x, %x]", txBuffer[0].getData()[0], txBuffer[0].getData()[1], txBuffer[0].getData()[2], txBuffer[0].getData()[3], txBuffer[0].getData()[4], txBuffer[0].getData()[5], txBuffer[0].getData()[6], txBuffer[0].getData()[7]);

    m_idriver->send_message(txBuffer, 1);

    // i wait some 10 ms
    yarp::os::Time::delay(0.010);

    int rm = m_idriver->receive_message(rxBuffer, 1, 1.0);
    for(int i=0; i<rm; i++)
    {
        if (rxBuffer[i].getData()[0]==0x2A)
        {
            ampregs.data[0] = rxBuffer[i].getData()[2];
            ampregs.data[1] = rxBuffer[i].getData()[3];
            ampregs.data[2] = rxBuffer[i].getData()[4];
            ampregs.data[3] = rxBuffer[i].getData()[5];
            ampregs.data[4] = rxBuffer[i].getData()[6];
            ampregs.data[5] = rxBuffer[i].getData()[7];
            //uint8_t from = rxBuffer[i].getData()[1];
            //yDebug("from %d: [%x, %x, %x, %x, %x, %x]", from, rxBuffer[i].getData()[2], rxBuffer[i].getData()[3], rxBuffer[i].getData()[4], rxBuffer[i].getData()[5], rxBuffer[i].getData()[6], rxBuffer[i].getData()[7]);
        }
        break;
    }


    return 0;
}

float cDownloader::strain_amplifier_discretegain2float(strain2_ampl_discretegain_t c)
{
    static const float mapofgains[ampl_gain_numberOf] =
    {
        48, 36, 24, 20, 16, 10, 8, 6, 4
    };

    unsigned int index = static_cast<unsigned int>(c);
    if(index >= ampl_gain_numberOf)
    {
        if(_verbose) yError ("strain_amplifier_discretegain2float(): cannot convert to a valid value\n");
        return 0.0f;
    }
    return mapofgains[index];
}

int cDownloader::strain_set_amplifier_discretegain(int bus, int target_id, unsigned char channel, strain2_ampl_discretegain_t ampcfg, int regset, string *errorstring)
{
    // check if driver is running
    if (m_idriver == NULL)
    {
        if(_verbose) yError ("Driver not ready\n");
        return -1;
    }

    // not for normal strain

    #define _NUMofREGS       6


        // constant value of offset registers
//        const uint8_t _cfg1map[ampl_gain_numberOf][_NUMofREGS] =
//        {
//            {0x00, 0x40, 0x46, 0x25, 0x00, 0x80},   // gain = 48
//            {0x00, 0x10, 0x46, 0x25, 0x00, 0x80},   // gain = 36
//            {0x00, 0x40, 0x42, 0x25, 0x00, 0x80},   // gain = 24 [or 0x26 instead of 0x42]
//            {0x00, 0x20, 0x42, 0x25, 0x00, 0x80},   // gain = 20 [or 0x26 instead of 0x42]
//            {0x00, 0x00, 0x42, 0x25, 0x00, 0x80},   // gain = 16 [or 0x26 instead of 0x42]
//            {0x00, 0xC0, 0x02, 0x25, 0x00, 0x80},   // gain = 10 [or 0x10 instead of 0x02]
//            {0x00, 0x80, 0x02, 0x25, 0x00, 0x80},   // gain = 08 [or 0x10 instead of 0x02]
//            {0x00, 0x40, 0x02, 0x25, 0x00, 0x80},   // gain = 06 [or 0x10 instead of 0x02]
//            {0x00, 0x40, 0x00, 0x25, 0x00, 0x80}    // gain = 04
//        };

    // offset all equal to 32k-1
    static const uint8_t _cfg1map[ampl_gain_numberOf][_NUMofREGS] =
    {
        {0x00, 0x40, 0x46, 0x1f, 0xb1, 0x7f},   // gain = 48
        {0x00, 0x10, 0x46, 0x2a, 0x80, 0x80},   // gain = 36
        {0x00, 0x40, 0x42, 0x3e, 0x62, 0x7f},   // gain = 24 [or 0x26 instead of 0x42]
        {0x00, 0x20, 0x42, 0x4b, 0x15, 0x80},   // gain = 20 [or 0x26 instead of 0x42]
        {0x00, 0x00, 0x42, 0x5e, 0x72, 0x80},   // gain = 16 [or 0x26 instead of 0x42]
        {0x00, 0xC0, 0x02, 0x64, 0xf6, 0x6e},   // gain = 10 [or 0x10 instead of 0x02]
        {0x00, 0x80, 0x02, 0x64, 0x29, 0x62},   // gain = 08 [or 0x10 instead of 0x02]
        {0x00, 0x40, 0x02, 0x64, 0xd4, 0x4c},   // gain = 06 [or 0x10 instead of 0x02]
        {0x00, 0x40, 0x00, 0x64, 0x29, 0x22}    // gain = 04
    };


    unsigned int index = static_cast<unsigned int>(ampcfg);

    if(index >= ampl_gain_numberOf)
    {
        if(_verbose) yError ("strain_set_amplifier_discretegain(): cannot convert to a valid index\n");
        return -1;
    }


    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(8);
    txBuffer[0].getData()[0]= 0x2B;
    txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
    txBuffer[0].getData()[2]= _cfg1map[index][0]; // lsb of gd
    txBuffer[0].getData()[3]= _cfg1map[index][1]; // msb of gd
    txBuffer[0].getData()[4]= _cfg1map[index][2];
    txBuffer[0].getData()[5]= _cfg1map[index][3]; // vc0
    txBuffer[0].getData()[6]= _cfg1map[index][4];
    txBuffer[0].getData()[7]= _cfg1map[index][5];
    set_bus(txBuffer[0], bus);
    if(_verbose) yDebug("strain_set_amplifier_discretegain() is sending: [%x, %x, %x, %x, %x, %x, %x, %x]", txBuffer[0].getData()[0], txBuffer[0].getData()[1], txBuffer[0].getData()[2], txBuffer[0].getData()[3], txBuffer[0].getData()[4], txBuffer[0].getData()[5], txBuffer[0].getData()[6], txBuffer[0].getData()[7]);

    m_idriver->send_message(txBuffer, 1);

    // i wait some 10 ms
    yarp::os::Time::delay(0.010);


    return 0;
}

int cDownloader::strain_get_amplifier_gain_offset      (int bus, int target_id, unsigned char channel, float &gain, uint16_t &offset, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x20;
     txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
     set_bus(txBuffer[0], bus);
     m_idriver->send_message(txBuffer, 1);


//     for(int nr=0; nr<1; nr++)
     {
         int rm = m_idriver->receive_message(rxBuffer, 1, 1.0);
         for(int i=0; i<rm; i++)
         {
             if (rxBuffer[i].getData()[0]==0x20)
             {

                 // uint8_t chn = rxBuffer[i].getData()[1] & 0x0f;
                 uint16_t g16 = static_cast<uint16_t>(rxBuffer[i].getData()[2]) | static_cast<uint16_t>(rxBuffer[i].getData()[3]) << 8;
                 float fg = static_cast<float>(g16) / 100;
                 gain = fg;
                 uint16_t o16 = static_cast<uint16_t>(rxBuffer[i].getData()[4]) | static_cast<uint16_t>(rxBuffer[i].getData()[5]) << 8;
                 offset = o16;
             }
             break;
         }
     }

     return 0;
}

int cDownloader::strain_set_amplifier_gain_offset(int bus, int target_id, unsigned char channel, float gain, uint16_t offset, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

#if 0
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(7);
     txBuffer[0].getData()[0]= 0x21;
     txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
     txBuffer[0].getData()[2]= 0; // mode is set both of them
     uint16_t gg = static_cast<uint16_t>(gain*100.0f);
     txBuffer[0].getData()[3] = (gg & 0x00ff);             // little endian
     txBuffer[0].getData()[4] = (gg & 0xff00) >> 8;        // little endian
     txBuffer[0].getData()[5] = (offset & 0x00ff);         // little endian
     txBuffer[0].getData()[6] = (offset & 0xff00) >> 8;    // little endian

     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;

#else

     strain::amplifier::PGA308 pga;

     strain::amplifier::DiscreteParams dp;
     strain::amplifier::PGA308::Registers regs;

     strain::amplifier::WideParams wp;

     wp.load(gain, offset);
     if(false == convert(wp, dp))
     {
        if(_verbose) yError ("cannot load this pair gain-offset into the strain2\n");
        return -1;
     }

     // load dp and ...
     pga.import(dp, &regs);
     strain2_ampl_regs_t reg2tx;
     size_t s;
     regs.fill(reg2tx.memory(), s);
     // now in reg2tx we have what we want to tx.

     return strain_set_amplifier_regs(bus, target_id, channel, reg2tx, regset, errorstring);

#endif
}


//*****************************************************************/
int cDownloader::strain_get_full_scale      (int bus, int target_id, unsigned char channel, unsigned int&  full_scale, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

      //read matrix gain
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(2);
     txBuffer[0].getData()[0]= 0x18;
     txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);

     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);

     drv_sleep(5);

     int read_messages = m_idriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        std::uint8_t data1 = rxBuffer[0].getData()[1];
        std::uint8_t rs = (data1 >> 4) & 0x0f;
        std::uint8_t cc = (data1 ) & 0x0f;

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
int cDownloader::strain_set_full_scale      (int bus, int target_id, unsigned char channel, unsigned int full_scale, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     // set fs
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(4);
     txBuffer[0].getData()[0]= 0x17;
     txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
     txBuffer[0].getData()[2]= full_scale >> 8;
     txBuffer[0].getData()[3]= full_scale & 0xFF;

     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
      drv_sleep(5);

     return 0;
}
//*****************************************************************/
int cDownloader::strain_get_matrix_rc     (int bus, int target_id, char r, char c, unsigned int& elem, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     // read dac
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(3);
     txBuffer[0].getData()[0]= 0x0A;
     txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (r & 0x0f);
     txBuffer[0].getData()[2]= c;

     clean_rx();
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);

     drv_sleep(3);

     int read_messages = m_idriver->receive_message(rxBuffer,1);
     for (int i=0; i<read_messages; i++)
     {
        if (rxBuffer[i].getData()[0]==0x0A &&
            rxBuffer[i].getId()==(2 << 8) + (target_id<<4))
            {
                int ret_r = (rxBuffer[i].getData()[1]) & 0x0f;
                int ret_c = rxBuffer[i].getData()[2];
                if ((r==ret_r) && (c==ret_c))
                {
                    elem = rxBuffer[i].getData()[3]<<8 | rxBuffer[i].getData()[4];
                    return 0;
                }
                else
                {
                    if(_verbose) yError ("strain_get_matrix_rc : invalid response\n");
                    return -1;
                }
            }
     }
     return -1;
}

//*****************************************************************/
int cDownloader::strain_set_matrix_rc     (int bus, int target_id, char r, char c, unsigned int  elem, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

     //set matrix
     txBuffer[0].setId((2 << 8) + target_id);
     txBuffer[0].setLen(5);
     txBuffer[0].getData()[0]= 0x03;
     txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (r & 0x0f);
     txBuffer[0].getData()[2]= c;
     txBuffer[0].getData()[3]= elem >> 8;
     txBuffer[0].getData()[4]= elem & 0xFF;
     set_bus(txBuffer[0], bus);
     int ret = m_idriver->send_message(txBuffer, 1);
     drv_sleep(5);

     return 0;
}

//*****************************************************************/
int cDownloader::strain_set_offset(int bus, int target_id, char channel, unsigned int offset, int regset, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError (" Driver not ready\n");
            return -1;
        }

    //set dac
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(4);
    txBuffer[0].getData()[0]= 0x04;
    txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);;
    txBuffer[0].getData()[2]= offset >> 8;
    txBuffer[0].getData()[3]= offset & 0xFF;
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
    drv_sleep(5);
/*
    int read_messages = m_idriver->receive_message(rxBuffer);
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


int cDownloader::strain_acquire_start(int bus, int target_id, uint8_t txratemilli, bool calibmode, strain_acquisition_mode_t acqmode, string *errorstring)
{
    // check if driver is running
    if (m_idriver == NULL)
       {
           if(_verbose) yError ("Driver not ready\n");
           return -1;
       }

    int ret = 0;


    if (strain_acquisition_mode_polling == acqmode)
    {
        return 0;
    }

    //yDebug() << "cDownloader::strain_acquire_start() from" << bus << target_id;

    // Send transmission rate to strain board
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x08;
    txBuffer[0].getData()[1]= txratemilli;
    set_bus(txBuffer[0], bus);
    ret = m_idriver->send_message(txBuffer, 1);
    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send txrate set message\n");
            return -1;
        }

   // Send transmission command to strain board
   txBuffer[0].setId((2 << 8) + target_id);
   txBuffer[0].setLen(2);
   txBuffer[0].getData()[0]= 0x07;
   txBuffer[0].getData()[1]= (false == calibmode) ? (0x03) : (0x00);
   set_bus(txBuffer[0], bus);
   ret = m_idriver->send_message(txBuffer, 1);
   // check if send_message was successful
   if (ret==0)
       {
           if(_verbose) yError ("Unable to send start tx message\n");
           return -1;
       }

   strain_is_acquiring_in_calibratedmode = calibmode;

   return 0;

}

int cDownloader::strain_acquire_stop(int bus, int target_id, strain_acquisition_mode_t acqmode, string *errorstring)
{
    // check if driver is running
    if (m_idriver == NULL)
       {
           if(_verbose) yError ("Driver not ready\n");
           return -1;
       }

    if (strain_acquisition_mode_polling == acqmode)
    {
        return 0;
    }

   // Send transmission command to strain board
   txBuffer[0].setId((2 << 8) + target_id);
   txBuffer[0].setLen(2);
   txBuffer[0].getData()[0]= 0x07;
   txBuffer[0].getData()[1]= 0x02;
   set_bus(txBuffer[0], bus);
   int ret = m_idriver->send_message(txBuffer, 1);
   // check if send_message was successful
   if (ret==0)
       {
           if(_verbose) yError ("Unable to send stop tx message\n");
           return -1;
       }


   // we should empty the rx buffer for a while...

   const double TOUT = 0.100;
   const size_t maxframes = rxBuffer.size() - 1;
   for(size_t n=0; n<10; n++)
   {
        int read_messages = m_idriver->receive_message(rxBuffer, maxframes, TOUT);
        //yDebug() << "cDownloader::strain_acquire_stop() has removed" << read_messages << "can frames from rxbuffer";
        if(0 == read_messages)
        {
            break;
        }
   }

   return 0;
}

int cDownloader::strain_acquire_get(int bus, int target_id, vector<strain_value_t> &values, const unsigned int howmany, void (*updateProgressBar)(void*, float), void *arg, strain_acquisition_mode_t acqmode, const unsigned int maxerrors, string *errorstring)
{
    // i must read howmany pairs of can frames of type 0xA and 0xB. to simplify i assume that they will arrive in pairs.


    values.clear();

    if(0 == howmany)
    {
        return -1;
    }

    unsigned int errorcount = 0;
    

    if (strain_acquisition_mode_polling == acqmode)
    {
        if(_verbose)
        {
            yDebug() << "cDownloader::strain_acquire_get() is using polling";
        }

        for (unsigned int s = 0; s < howmany; s++)
        {
            cDownloader::strain_value_t sv;
            sv.valid = true;

            for (int c = 0; c < 6; c++)
            {
                //drv_sleep(3);
                unsigned int adc = 0;
                int rr = strain_get_adc(bus, target_id, c, adc, 0);
                if (-1 == rr)
                {
                    errorcount++;
                    yDebug() << "error in acquisition of an adc channel " << c << "incrementing error counter to" << errorcount;
                    if (errorcount >= maxerrors)
                    {
                        yError() << "reached" << maxerrors << "reception errors in adc acquisition: must quit";
                        return -1;
                    }
                }
                sv.channel[c] = adc;
            }
            values.push_back(sv);

            if (NULL != updateProgressBar)
            {
                float perc = (0 != howmany) ? (static_cast<float>(s + 1) / static_cast<float>(howmany)) : (100.0);
                updateProgressBar(arg, perc);
            }
        }

        return 0;
    }

    const double TOUT = 3.0;

    // purge from possible acks of strain1...

    m_idriver->receive_message(rxBuffer, 2, TOUT);
    m_idriver->receive_message(rxBuffer, 2, TOUT);


    for(unsigned int s=0; s<howmany; s++)
    {

        if(NULL != updateProgressBar)
        {
            float perc = (0 != howmany) ? (static_cast<float>(s+1)/static_cast<float>(howmany)) : (100.0);
            updateProgressBar(arg, perc);
        }

        int read_messages = m_idriver->receive_message(rxBuffer, 2, TOUT);
        strain_value_t sv;

        sv.channel[0] = 0x66666;
        sv.channel[4] = 0x66666;

        if(2 == read_messages)
        {
            for(unsigned int i=0; i<2; i++)
            {
                uint32_t id = rxBuffer[i].getId();
                uint8_t type = id & 0xf;

                if((0xA != type) && (0xB != type))
                {
                    yError() << "cDownloader::strain_acquire_get() has detected strange can frames of type = " << type << ".... operation aborted";
                    char rxframe[128] = {0};
                    snprintf(rxframe, sizeof(rxframe), "l = %d, id = 0x%x, d = 0x[%x %x %x %x %x %x %x %x]", rxBuffer[i].getLen(), rxBuffer[i].getId(),
                                        rxBuffer[i].getData()[0], rxBuffer[i].getData()[1], rxBuffer[i].getData()[2], rxBuffer[i].getData()[3],
                                        rxBuffer[i].getData()[4], rxBuffer[i].getData()[5], rxBuffer[i].getData()[6], rxBuffer[i].getData()[7]);

                    yError() << "frame -> " << rxframe;
                    continue;
                    //return -1;
                }

                // values in little endian
                uint16_t x = static_cast<uint16_t>(rxBuffer[i].getData()[0]) | (static_cast<uint16_t>(rxBuffer[i].getData()[1]) << 8);
                uint16_t y = static_cast<uint16_t>(rxBuffer[i].getData()[2]) | (static_cast<uint16_t>(rxBuffer[i].getData()[3]) << 8);
                uint16_t z = static_cast<uint16_t>(rxBuffer[i].getData()[4]) | (static_cast<uint16_t>(rxBuffer[i].getData()[5]) << 8);

                sv.saturated = (6 == rxBuffer[i].getLen()) ? (true) : (false);
                sv.valid = true;

                sv.calibrated = strain_is_acquiring_in_calibratedmode;

                if(0xA == type)
                {
                    //yDebug() << "RX 0xA";

                    sv.channel[0] = x;
                    sv.channel[1] = y;
                    sv.channel[2] = z;
                    if(false == sv.saturated)
                    {
                        void *tmp = &rxBuffer[i].getData()[6];
                        icubCanProto_strain_forceSaturationInfo_t *sinfo = reinterpret_cast<icubCanProto_strain_forceSaturationInfo_t*>(tmp);
                        if(1 == sinfo->thereIsSaturationInAtLeastOneChannel)
                        {
                            sv.saturationinfo[0] = static_cast<icubCanProto_strain_saturationInfo_t>(sinfo->saturationInChannel_0);
                            sv.saturationinfo[1] = static_cast<icubCanProto_strain_saturationInfo_t>(sinfo->saturationInChannel_1);
                            sv.saturationinfo[2] = static_cast<icubCanProto_strain_saturationInfo_t>(sinfo->saturationInChannel_2);
                        }
                    }
                }
                else if(0xB == type)
                {
                    //yDebug() << "RX 0xB";

                    sv.channel[3] = x;
                    sv.channel[4] = y;
                    sv.channel[5] = z;
                    if(false == sv.saturated)
                    {
                        void *tmp = &rxBuffer[i].getData()[6];
                        icubCanProto_strain_torqueSaturationInfo_t *sinfo = reinterpret_cast<icubCanProto_strain_torqueSaturationInfo_t*>(tmp);
                        if(1 == sinfo->thereIsSaturationInAtLeastOneChannel)
                        {
                            sv.saturationinfo[3] = static_cast<icubCanProto_strain_saturationInfo_t>(sinfo->saturationInChannel_3);
                            sv.saturationinfo[4] = static_cast<icubCanProto_strain_saturationInfo_t>(sinfo->saturationInChannel_4);
                            sv.saturationinfo[5] = static_cast<icubCanProto_strain_saturationInfo_t>(sinfo->saturationInChannel_5);
                        }
                    }
                }
                else
                {
                    //yDebug() << "RX ??";
                }

            }

            if((0x66666 == sv.channel[0]) || (0x66666 == sv.channel[4]))
            {
                yDebug() << "cDownloader::strain_acquire_get(): did not receive two consecutive 0xA and 0xB. please check!";
            }
            else
            {
                values.push_back(sv);
            }
        }
        else
        {
            errorcount ++;
            yDebug() << "in streaming mode did not received two messages but " << read_messages << "incrementing error counter to" << errorcount;
            if(errorcount >= maxerrors)
            {
                yError() << "reached" << maxerrors << "reception errors: must quit";
                return -1;
            }
        }

    }


    return 0;
}



//*****************************************************************/
int cDownloader::strain_start_sampling    (int bus, int target_id, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

    // Send transmission command to strain board
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x07;
    txBuffer[0].getData()[1]= 0x01;
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }
    return 0;
}

//*****************************************************************/
int cDownloader::strain_stop_sampling    (int bus, int target_id, string *errorstring)
{
     // check if driver is running
     if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

    // Send transmission command to strain board
    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(2);
    txBuffer[0].getData()[0]= 0x07;
    txBuffer[0].getData()[1]= 0x02;
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }
    return 0;
}

// usage:
// strain:      get the single value guiTarget as a [-32k, +32k) from the widget. put it inside a vector.
//              use an empty gains vector
//              std::vector<int16_t> targets;
//              targets.push_back(guiTarget);
//              std::vector<strain2_ampl_discretegain_t> gains(0);
//              strain_calibrate_offset2(bus, id, icubCanProto_boardType__strain, gains, targets);
// strain2:     get the guiTargets as [-32k, +32k) values from the widget. put them all inside a vector which must become of size 6.
//              std::vector<int16_t> targets;
//              targets.push_back(guiTargets[0]); targets.push_back(guiTargets[1]); etc....
//              get the 6 gains and put them into a vector
//              std::vector<strain2_ampl_discretegain_t> gains;
//              gains.push_back(g0); etc....
//              strain_calibrate_offset2(bus, id, icubCanProto_boardType__strain, gains, targets);

int cDownloader::strain_calibrate_offset2  (int bus, int target_id, icubCanProto_boardType_t boardtype, const std::vector<strain2_ampl_discretegain_t> &gains, const std::vector<int16_t> &targets, string *errorstring)
{

    if(icubCanProto_boardType__strain == boardtype)
    {
        int16_t i16 = (targets.empty()) ? (0) : (targets[0]);

        return strain_calibrate_offset2_strain1(bus, target_id, i16, errorstring);
    }
    else if(icubCanProto_boardType__strain2 == boardtype)
    {
        return strain_calibrate_offset2_strain2(bus, target_id, gains, targets, errorstring);
    }

    return -1;
}


//*****************************************************************/
int cDownloader::strain_calibrate_offset  (int bus, int target_id, icubCanProto_boardType_t boardtype, unsigned int middle_val, string *errorstring)
{
    // the calibration of the offset is meaningful only for the calibration set in use.
    const int regset = strain_regset_inuse;

    // check if driver is running
    if (m_idriver == NULL)
    {
       if(_verbose) yError ("Driver not ready\n");
       return -1;
    }

    int daclimit = 0x3ff;
    int dacstep = 1;
    long tolerance = 256;

#define STRAIN2_USE_NEW_MODE

#if !defined(STRAIN2_USE_NEW_MODE)
    if(icubCanProto_boardType__strain2 == boardtype)
    {
        daclimit = 0xffff;
        dacstep = 16;
        tolerance = 64;
    }
#else
    if(icubCanProto_boardType__strain2 == boardtype)
    {
        yDebug() << "strain2-amplifier-tuning: see the various STEP-x";

        const uint8_t everychannel = 0x0f;
        tolerance = 256;
        uint8_t samples2average = 8; // if zero, the board uses its default (= 4)

#define TESTMODE_STRAIN2
#undef TESTMODE_STRAIN2_SAMEGAIN

//#define DEBUGLEVEL_MAXIMUM

#if defined(TESTMODE_STRAIN2)

    const unsigned int NUMofCHANNELS = 6;


#if defined(TESTMODE_STRAIN2_SAMEGAIN)

        const int i2u_all = index24;

        yDebug() << "imposing gain =" << gainvalues[i2u_all] << "in every channel";

        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(8);
        txBuffer[0].getData()[0]= 0x2B;
        txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (everychannel & 0x0f);
        txBuffer[0].getData()[2]= cfg1map[i2u_all][0]; // lsb of gd
        txBuffer[0].getData()[3]= cfg1map[i2u_all][1]; // msb of gd
        txBuffer[0].getData()[4]= cfg1map[i2u_all][2];
        txBuffer[0].getData()[5]= cfg1map[i2u_all][3]; // vc0
        txBuffer[0].getData()[6]= cfg1map[i2u_all][4];
        txBuffer[0].getData()[7]= cfg1map[i2u_all][5];
        set_bus(txBuffer[0], bus);
        yDebug("sending: [%x, %x, %x, %x, %x, %x, %x, %x]", txBuffer[0].getData()[0], txBuffer[0].getData()[1], txBuffer[0].getData()[2], txBuffer[0].getData()[3], txBuffer[0].getData()[4], txBuffer[0].getData()[5], txBuffer[0].getData()[6], txBuffer[0].getData()[7]);

        m_idriver->send_message(txBuffer, 1);

        // i wait some 100 ms
        yarp::os::Time::delay(1.0);

#else


#if defined(DEBUGLEVEL_MAXIMUM)

        yDebug() << "reading (gain, offset) of front end amplifiers";

        for(int c=0; c<NUMofCHANNELS; c++)
        {
            float gaain = 0;
            uint16_t ooffset = 0;
            strain_get_amplifier_gain_offset(bus, target_id, c, gaain, ooffset, regset, errorstring);
            yDebug("channel %d: gain = %f, offset = %d", c, gaain, ooffset);
            //strain_set_amplifier_gain_offset(bus, target_id, c, gaain+1.0f, ooffset+128, regset, errorstring);
            //yarp::os::Time::delay(1.0);
            //gaain = 0; ooffset = 0;
            //strain_get_amplifier_gain_offset(bus, target_id, c, gaain, ooffset, regset, errorstring);
            //yDebug("channel %d: new gain = %f, new offset = %d", c, gaain, ooffset);
        }


        yDebug() << "i get the amplifier reg config";

        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(2);
        txBuffer[0].getData()[0]= 0x2A;
        txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (everychannel & 0x0f);
        set_bus(txBuffer[0], bus);
        m_idriver->send_message(txBuffer, 1);

        yDebug() << "i print the amplifier reg config";

        for(int nr=0; nr<NUMofCHANNELS; nr++)
        {
            int rm = m_idriver->receive_message(rxBuffer, 1, 1.0);
            for(int i=0; i<rm; i++)
            {
                if (rxBuffer[i].getData()[0]==0x2A)
                {

                    uint8_t from = rxBuffer[i].getData()[1];
                    yDebug("from %d: [%x, %x, %x, %x, %x, %x]", from, rxBuffer[i].getData()[2], rxBuffer[i].getData()[3], rxBuffer[i].getData()[4], rxBuffer[i].getData()[5], rxBuffer[i].getData()[6], rxBuffer[i].getData()[7]);
                }
                break;
            }
        }

        yarp::os::Time::delay(2.0);

#endif // #if defined(DEBUGLEVEL_MAXIMUM)


        // step1: i set the amplifiers


        // the chosen gains:
        const strain2_ampl_discretegain_t ampsets[NUMofCHANNELS] =
        {
            ampl_gain08, ampl_gain24, ampl_gain24,
            ampl_gain10, ampl_gain10, ampl_gain24
        };

        yDebug() << "strain2-amplifier-tuning: STEP-1. imposing gains which are different of each channel";

        for(int channel=0; channel<NUMofCHANNELS; channel++)
        {
            yDebug() << "strain2-amplifier-tuning: STEP-1. on channel" << channel << "we impose gain =" << strain_amplifier_discretegain2float(ampsets[channel]);

            strain_set_amplifier_discretegain(bus, target_id, channel, ampsets[channel], regset, errorstring);

            // i wait some time
            yarp::os::Time::delay(1.0);
        }


#endif // #else of #if defined(TESTMODE_STRAIN2_SAMEGAIN)


        // step2: i read back gains just to be sure ..

        yDebug() << "strain2-amplifier-tuning: STEP-2. reading (gain, offset) of front end amplifiers";

        for(int c=0; c<NUMofCHANNELS; c++)
        {
            float gaain = 0;
            uint16_t ooffset = 0;
            strain_get_amplifier_gain_offset(bus, target_id, c, gaain, ooffset, regset, errorstring);
            yDebug("strain2-amplifier-tuning: STEP-2. channel %d: gain = %f, offset = %d", c, gaain, ooffset);
        }

        yarp::os::Time::delay(2.0);


#if defined(DEBUGLEVEL_MAXIMUM)

        yDebug() << "i get the amplifier reg config";

        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(2);
        txBuffer[0].getData()[0]= 0x2A;
        txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (everychannel & 0x0f);
        set_bus(txBuffer[0], bus);
        m_idriver->send_message(txBuffer, 1);

        yDebug() << "i print the amplifier reg config";

        for(int nr=0; nr<NUMofCHANNELS; nr++)
        {
            int rm = m_idriver->receive_message(rxBuffer, 1, 1.0);
            for(int i=0; i<rm; i++)
            {
                if (rxBuffer[i].getData()[0]==0x2A)
                {

                    uint8_t from = rxBuffer[i].getData()[1];
                    yDebug("from %d: [%x, %x, %x, %x, %x, %x]", from, rxBuffer[i].getData()[2], rxBuffer[i].getData()[3], rxBuffer[i].getData()[4], rxBuffer[i].getData()[5], rxBuffer[i].getData()[6], rxBuffer[i].getData()[7]);
                }
                break;
            }
        }

        yarp::os::Time::delay(2.0);

#endif // #if defined(DEBUGLEVEL_MAXIMUM)



#endif


        // step3: autocalib

        yDebug() << "strain2-amplifier-tuning: STEP-3. regularisation of ADC to " << middle_val;
        yDebug() << "strain2-amplifier-tuning: STEP-3. other params: mae tolerence is" << tolerance << "and samples2average =" << samples2average;


        // sending an autocalib message
        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(8);
        txBuffer[0].getData()[0]= 0x22;
        txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (everychannel & 0x0f);
        txBuffer[0].getData()[2]= 0; // mode oneshot, the only possible so far
        txBuffer[0].getData()[3]= middle_val & 0x00ff;          // little endian
        txBuffer[0].getData()[4]= (middle_val >> 8) & 0x00ff;   // little endian
        txBuffer[0].getData()[5]= tolerance & 0x00ff;          // little endian
        txBuffer[0].getData()[6]= (tolerance >> 8) & 0x00ff;   // little endian
        txBuffer[0].getData()[7]= samples2average;
        set_bus(txBuffer[0], bus);
        yDebug("strain2-amplifier-tuning: STEP-3. sent message = [%x, %x, %x, %x, %x, %x, %x, %x]", txBuffer[0].getData()[0], txBuffer[0].getData()[1], txBuffer[0].getData()[2], txBuffer[0].getData()[3], txBuffer[0].getData()[4], txBuffer[0].getData()[5], txBuffer[0].getData()[6], txBuffer[0].getData()[7]);
        int ret = m_idriver->send_message(txBuffer, 1);
        // check if send_message was successful
        if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }

        // now wait for a reply for most 3 seconds
        double TOUT = 3.0;

        yDebug() << "strain2-amplifier-tuning: STEP-3. results ...";

        int read_messages = m_idriver->receive_message(rxBuffer, 1, TOUT);
        for(int i=0; i<read_messages; i++)
        {
            if (rxBuffer[i].getData()[0]==0x22)
            {
                yDebug("strain2-amplifier-tuning: STEP-3. received message = [%x, %x, %x, %x, %x, %x, %x, %x]", rxBuffer[0].getData()[0], rxBuffer[0].getData()[1], rxBuffer[0].getData()[2], rxBuffer[0].getData()[3], rxBuffer[0].getData()[4], rxBuffer[0].getData()[5], rxBuffer[0].getData()[6], rxBuffer[0].getData()[7]);

                //dac = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                uint8_t noisychannelmask = rxBuffer[i].getData()[2];
                uint8_t algorithmOKmask = rxBuffer[i].getData()[3];
                uint8_t finalmeasureOKmask = rxBuffer[i].getData()[4];
                uint16_t mae =  (static_cast<uint32_t>(rxBuffer[i].getData()[6]))       |
                                (static_cast<uint32_t>(rxBuffer[i].getData()[7]) << 8);

                if((0x3f == algorithmOKmask) && (0x3f == finalmeasureOKmask))
                {
                    yDebug() << "strain2-amplifier-tuning: STEP-3. OK. regularisation to value" << middle_val << "is done and MAE = " << mae;
                    if(0 != noisychannelmask)
                    {
                        yDebug() << "however we found some noisy channels";
                        yDebug("noisychannelmask = 0x%x, algorithmOKmask = 0x%x, finalmeasureOKmask = 0x%x, mae = %d", noisychannelmask, algorithmOKmask, finalmeasureOKmask, mae);

                    }

                }
                else
                {
                    if(0x3f != algorithmOKmask)
                    {
                        yDebug() << "strain2-amplifier-tuning: STEP-3. KO. regularisation to value" << middle_val << "has sadly failed because algorithm found required values out of range of registers CFG0.OS or ZDAC.";
                    }
                    else
                    {
                        yDebug() << "strain2-amplifier-tuning: STEP-3. KO. regularisation to value" << middle_val << "has failed because MAE error is high on some channels.";
                    }

                    yDebug("noisychannelmask = 0x%x, algorithmOKmask = 0x%x, finalmeasureOKmask = 0x%x, mae = %d", noisychannelmask, algorithmOKmask, finalmeasureOKmask, mae);
                    for(uint8_t i=0; i<NUMofCHANNELS; i++)
                    {
                        if((algorithmOKmask & (0x01<<i)) == 0)
                        {
                            yDebug() << "calibration fails in channel" << i;
                        }
                        if((finalmeasureOKmask & (0x01<<i)) == 0)
                        {
                            yDebug() << "mae is high in channel" << i;
                        }
                    }
                }
                break;
            }
        }


        return 0;
    }
#endif


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
        // yDebug() << "starting OFFSET of channel" << channel;
        // Send read channel command to strain board
        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(3);
        txBuffer[0].getData()[0]= 0x0C;
        txBuffer[0].getData()[1]= channel;
        txBuffer[0].getData()[2]= 0;
        set_bus(txBuffer[0], bus);
        ret = m_idriver->send_message(txBuffer, 1);
        // check if send_message was successful
        if (ret==0)
            {
                if(_verbose) yError ("Unable to send message\n");
                return -1;
            }
        //read adc
        read_messages = m_idriver->receive_message(rxBuffer,1);
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
        txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
        set_bus(txBuffer[0], bus);
        ret = m_idriver->send_message(txBuffer, 1);

        read_messages = m_idriver->receive_message(rxBuffer,1);
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

        while (abs(error)>tolerance && cycle<daclimit)
        {
            if (error>0) dac -= dacstep;
            else         dac += dacstep;

            if (dac>daclimit)       dac = daclimit;
            if (dac<0)              dac = 0;

            //yDebug() << "iter" << cycle << "err = " << error << "next dac =" << dac;

            // Send transmission command to strain board
            txBuffer[0].setId((2 << 8) + target_id);
            txBuffer[0].setLen(4);
            txBuffer[0].getData()[0]= 0x04;
            txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
            txBuffer[0].getData()[2]= dac >> 8;
            txBuffer[0].getData()[3]= dac & 0xFF;
            set_bus(txBuffer[0], bus);
            int ret = m_idriver->send_message(txBuffer, 1);

            //wait
            drv_sleep(3);

            // Send read channel command to strain board
            txBuffer[0].setId((2 << 8) + target_id);
            txBuffer[0].setLen(3);
            txBuffer[0].getData()[0]= 0x0C;
            txBuffer[0].getData()[1]= channel;
            txBuffer[0].getData()[2]= 0;
            set_bus(txBuffer[0], bus);
            ret = m_idriver->send_message(txBuffer, 1);
            // check if send_message was successful
            if (ret==0)
            {
                if(_verbose) yError ("Unable to send message\n");
                return -1;
            }
            //read adc
            read_messages = m_idriver->receive_message(rxBuffer, 1);
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
int cDownloader::get_serial_no       (int bus, int target_id, char* serial_no)
{
    int ret = -1;
    int i;
    if (serial_no == NULL) return -1;

    memset (serial_no,0,8);

    // check if driver is running
    if (m_idriver == NULL)
    {
        if(_verbose) yError ("Driver not ready\n");
        return -1;
    }

    for (i=0; i<board_list_size; i++)
    {
        if ((board_list[i].pid==target_id) &&
            (board_list[i].type==icubCanProto_boardType__strain) || (board_list[i].type==icubCanProto_boardType__strain2))
        {
            this->strain_get_serial_number(bus, target_id, serial_no);
            ret = 0;
        }
    }

    return ret;
}


#define EOCANPROT_D_CREATE_CANID(clss, orig, dest)                ( (((clss)&0xF) << 8) | (((orig)&0xF) << 4) | ((dest)&0xF) )

//#warning add controls vs sending it in broadcast or to all buses
int cDownloader::get_firmware_version(int bus, int target_id, eObrd_cantype_t boardtype, eObrd_info_t *info, bool &noreply)
{
    noreply = true;

    if(NULL == info)
    {
        return -1;
    }
    // check if driver is running
    if(NULL == m_idriver)
    {
        if(_verbose) yError ("cDownloader::get_firmware_version(): driver not ready\n");
        return -1;
    }

    int read_messages = 0;

    // reset the answer
    info->type = boardtype;
    info->firmware.major = info->firmware.minor = info->firmware.build = 0;
    info->protocol.major = info->protocol.minor = 0;

    txBuffer[0].setLen(3);
    txBuffer[0].getData()[0] = 0; // fill it later on
    txBuffer[0].getData()[1] = 0;
    txBuffer[0].getData()[2] = 0; // we send a (0, 0) prototocol version.

    //#warning -> check if sending a get-prot-version message with wrong prot version is OK or not (hopefully it will not send boards in hw fault).

    // prepare command. it depends on board type.

    bool boardisMC = false;
    switch(boardtype)
    {
        case eobrd_cantype_dsp:
        case eobrd_cantype_mc4:
        case eobrd_cantype_2dc:
        case eobrd_cantype_bll:
        case eobrd_cantype_foc:
        {
            boardisMC = true;
            txBuffer[0].setId(EOCANPROT_D_CREATE_CANID(ICUBCANPROTO_CLASS_POLLING_MOTORCONTROL, 0, target_id));
            txBuffer[0].getData()[0] = ICUBCANPROTO_POL_MC_CMD__GET_FIRMWARE_VERSION;
        } break;

        case eobrd_cantype_mtb:
        case eobrd_cantype_strain:
        case eobrd_cantype_mais:
        case eobrd_cantype_6sg:
        case eobrd_cantype_mtb4:
        case eobrd_cantype_strain2:
        case eobrd_cantype_rfe:
        case eobrd_cantype_sg3:
        case eobrd_cantype_psc:
        case eobrd_cantype_mtb4w:
        {
            boardisMC = false;
            txBuffer[0].setId(EOCANPROT_D_CREATE_CANID(ICUBCANPROTO_CLASS_POLLING_ANALOGSENSOR, 0, target_id));
            txBuffer[0].getData()[0] = ICUBCANPROTO_POL_AS_CMD__GET_FW_VERSION;
        } break;

        default:
        {
            if(_verbose) yError ("cDownloader::get_firmware_version(): this board %d is not supported. returning all zeros\n", boardtype);
            return -2;
        }
    }


    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
    if(ret==0)
    {
        if(_verbose) yError ("Unable to send message\n");
        return -1;
    }

    read_messages = m_idriver->receive_message(rxBuffer, 1, 1.0);

    if(0 == read_messages)
    {   // it does not support teh message
        return 0;
    }

    noreply = false;

    for (int i=0; i<read_messages; i++)
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

        if ((txBuffer[0].getData()[0] == rxBuffer[i].getData()[0]) && (8 == rxBuffer[i].getLen()))
        {
            info->type               = rxBuffer[i].getData()[1];
            info->firmware.major     = rxBuffer[i].getData()[2];
            info->firmware.minor     = rxBuffer[i].getData()[3];
            info->firmware.build     = rxBuffer[i].getData()[4];
            info->protocol.major     = rxBuffer[i].getData()[5];
            info->protocol.minor     = rxBuffer[i].getData()[6];
        }
        else
        {
            yWarning() << "unknown message";
        }
    }

    return 0;
}

//*****************************************************************/
int cDownloader::get_board_info       (int bus, int target_id, char* board_info)
{
    int i;
    if (board_info == NULL) return -1;

    memset (board_info,0x3f,32);

    // check if driver is running
    if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

    //riceve la risposta
    int read_messages = 0; //m_idriver->receive_message(rxBuffer, 10, 0);

    // Send command
    txBuffer[0].setId(build_id(ID_MASTER, target_id));
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= ICUBCANPROTO_BL_GET_ADDITIONAL_INFO;
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);
//    ret=0;
    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }

    //pause
    //drv_sleep(10);

    //riceve la risposta
    read_messages = m_idriver->receive_message(rxBuffer, 64, 1.0);

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
int cDownloader::change_board_info(int bus, int target_id, char* board_info)
{
    // check if driver is running
    if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
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
                {
                    txBuffer[0].getData()[2+j] = board_info[j+counter*4];
                }
                set_bus(txBuffer[0], bus);
                ret |= m_idriver->send_message(txBuffer, 1);
            }
        }

    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }

    //pause
    drv_sleep(500);

    // update the board list
    initschede();

    return 0;
}

//*****************************************************************/
int cDownloader::change_card_address(int bus, int target_id, int new_id, int board_type)
{
    int i = 0;

    // check if driver is running
    if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

    switch (board_type)
    {
        case icubCanProto_boardType__strain:
        case icubCanProto_boardType__skin:
        case icubCanProto_boardType__mais:
        case icubCanProto_boardType__6sg:
        case icubCanProto_boardType__mtb4:
        case icubCanProto_boardType__strain2:
        case icubCanProto_boardType__rfe:
        case icubCanProto_boardType__sg3:
        case icubCanProto_boardType__psc:
        case icubCanProto_boardType__mtb4w:
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
            if(_verbose) yError ("Unknown board type for change of CAN address\n");
        return -1;

    }

    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);

    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
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
    if (m_idriver == NULL)
        {
            if(_verbose) yError ("Driver not ready\n");
            return -1;
        }

    // Send discovery command

    int ret = 0;

#if defined(DOWNLOADER_ETH_SUPPORTS_MULTIBUS)

    if(iDriver2::eth_driver2 == m_idriver->type())
    {
        // in here we send the discovery command on the selected CAN bus (CAN1 / CAN2) or on both
        if(CanPacket::everyCANbus == get_canbus_id())
        {
            if(_verbose) yDebug("working on every CAN bus");
        }

        set_bus(txBuffer[0], get_canbus_id());
        txBuffer[0].setId(build_id(ID_MASTER,ID_BROADCAST));
        txBuffer[0].setLen(1);
        txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BROADCAST;
        ret = m_idriver->send_message(txBuffer, 1);

    }
    else
    {
        // we send the discovery command only on one bus
        set_bus(txBuffer[0], get_canbus_id());
        txBuffer[0].setId(build_id(ID_MASTER,ID_BROADCAST));
        txBuffer[0].setLen(1);
        txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BROADCAST;
        ret = m_idriver->send_message(txBuffer, 1);
    }

#else

    if(CanPacket::everyCANbus == get_canbus_id())
    {
        if(_verbose) yDebug("Discovery on every CAN bus is not allowed: reverting to bus CAN1");
        set_canbus_id(1);
    }

    // we send discovery only on the relevant CAN bus.
    set_bus(txBuffer[0], get_canbus_id());

    txBuffer[0].setId(build_id(ID_MASTER,ID_BROADCAST));
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BROADCAST;
    ret = m_idriver->send_message(txBuffer, 1);

#endif

    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }

    // pause
    drv_sleep(300);

    // riceve la risposta
    bool done=false;
    int read_messages=0;
    while(!done)
        {
            read_messages = m_idriver->receive_message(rxBuffer);
            //Timeout: no answers
            if (read_messages==0)
                {
                    if(_verbose) yError ("No answers\n");
                    return -1;
                }

            //One (or more) answers received
            //Counts the number of the boards
            board_list_size = 0;
            for (i=0; i<read_messages; i++)
                {
                    ///////
#if 0
                    //fprintf(stderr, "id   %.4x ", rxBuffer[i].getId());
                    fprintf(stderr, "CAN%d:%.2x, l=%d", get_bus(rxBuffer[i]), rxBuffer[i].getId(), rxBuffer[i].getLen());
                    fprintf(stderr, "d[0] %.2x ", rxBuffer[i].getData()[0]);
                    fprintf(stderr, "d[1] %.2x ", rxBuffer[i].getData()[1]);
                    fprintf(stderr, "d[2] %.2x ", rxBuffer[i].getData()[2]);
                    fprintf(stderr, "d[3] %.2x ", rxBuffer[i].getData()[3]);
                    fprintf(stderr, "d[4] %.2x ", rxBuffer[i].getData()[4]);
                    fprintf(stderr, "d[5] %.2x ", rxBuffer[i].getData()[5]);
                    fprintf(stderr, "d[6] %.2x ", rxBuffer[i].getData()[6]);
                    fprintf(stderr, "d[7] %.2x\n", rxBuffer[i].getData()[7]);
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
                            if(_verbose) yError ("No Boards found\n");
                            return -1;
                        }
                }
            else
                {
                    done=true;
                }

        }

//    if(_verbose) yDebug ("received all answers to FF\n");


    //Create the list of the boards
    if (board_list !=NULL) delete board_list;
    board_list= new sBoard[board_list_size];

    int j = 0;
    for (i=0; i<read_messages; i++)
        {
            if ((rxBuffer[i].getData()[0]==ICUBCANPROTO_BL_BROADCAST) &&
                ((rxBuffer[i].getLen()==4)||(rxBuffer[i].getLen()==5)))   //old board firmware (backward compatibility)
                {
#if defined(DOWNLOADER_USE_IDRIVER2)
                    board_list[j].bus     = rxBuffer[i].getCanBus();
#else
                    board_list[j].bus    =  get_canbus_id();
#endif
                    board_list[j].pid     = (rxBuffer[i].getId() >> 4) & 0x0F;
                    board_list[j].type    = rxBuffer[i].getData()[1];
                    board_list[j].applicationisrunning = (5 == rxBuffer[i].getLen()) ? (true) : (false); // the application replies with a message of len 5, the bootloader 4.
                    board_list[j].appl_vers_major = rxBuffer[i].getData()[2];
                    board_list[j].appl_vers_minor = rxBuffer[i].getData()[3];
                    board_list[j].status  = BOARD_RUNNING;
                    board_list[j].selected  = false;
                    board_list[j].eeprom =false;
                    memset (board_list[j].serial,  0, 8);
                    memset (board_list[j].add_info,  0, 32);
                    if (rxBuffer[i].getLen()==4)
                        board_list[j].appl_vers_build = -1;
                    else
                        board_list[j].appl_vers_build = rxBuffer[i].getData()[4];
                    board_list[j].prot_vers_major = 0;
                    board_list[j].prot_vers_minor = 0;

                    j++;
                }
        }

    //if(_verbose) yDebug ("about to ask boardinfo \n");

    for (i=0; i<board_list_size; i++)
        {
            char board_info [32];
            get_board_info       (board_list[i].bus, board_list[i].pid, board_info);
            strcpy (board_list[i].add_info,  board_info);
            //pause
            drv_sleep(10);
        }


    //if(_verbose) yDebug ("about to ask serialno \n");

    for (i=0; i<board_list_size; i++)
    {
        char serial_no [32];
        get_serial_no       (board_list[i].bus, board_list[i].pid, serial_no);
        strcpy (board_list[i].serial,  serial_no);
        if(0 == strlen(board_list[i].serial))
        {
            snprintf(board_list[i].serial, sizeof(board_list[i].serial), "N/A");
        }
        //pause
        drv_sleep(10);
    }

    for (i=0; i<board_list_size; i++)
    {
        board_list[i].strainregsetinuse = board_list[i].strainregsetatboot = 0;
        if(board_list[i].type==icubCanProto_boardType__strain)
        {
            board_list[i].strainregsetinuse = board_list[i].strainregsetatboot = 1;
        }
        else if(board_list[i].type==icubCanProto_boardType__strain2)
        {
            this->strain_get_regulationset(board_list[i].bus, board_list[i].pid, board_list[i].strainregsetinuse, strain_regsetmode_temporary);
            this->strain_get_regulationset(board_list[i].bus, board_list[i].pid, board_list[i].strainregsetatboot, strain_regsetmode_permanent);
        }
        //pause
        drv_sleep(10);
    }

#define TEST_GET_FW_VERSION

#if defined(TEST_GET_FW_VERSION)

    if(_verbose) yDebug ("about to ask fw version \n");
    for(i=0; i<board_list_size; i++)
    {
        // marco.accame on 25 may 2016: i have added this code for demostration of how we can use the get_firmware_version() function.
        // this info is useful for the new fwUpdater. Moreover, it is a good way to further verify if we are in bootloader or not.
        // the bootloader does not reply to get-fw-version, whereas the applications replies.
        // The only known exception is the mtb application which replies o get-fw-version only after ... somewhere in early 2016.
        eObrd_info_t info = {0};
        memset(&info, 0, sizeof(info));
        bool noreply = true;
        int rr = get_firmware_version(board_list[i].bus, board_list[i].pid, (eObrd_cantype_t)board_list[i].type, &info, noreply);
        if(_verbose)
        {
            fprintf(stderr, "board %d: ret = %d, reply = %d, type = %d, f=(%d, %d, %d), pr=(%d, %d)\n", i, rr, !noreply,
                                        info.type,
                                        info.firmware.major, info.firmware.minor, info.firmware.build,
                                        info.protocol.major, info.protocol.minor);
        }
        board_list[i].prot_vers_major = info.protocol.major;
        board_list[i].prot_vers_minor = info.protocol.minor;
        drv_sleep(10);
    }
#endif


    if(_verbose) yInfo("CONNECTED: %d Boards",board_list_size);
    if(_verbose) yDebug("  BUS:id   type     version");
    for (int i = 0; i < board_list_size; i++)
    {
        if(_verbose) yDebug(" CAN%d:%d   %5d     %d.%d.%d", board_list[i].bus, board_list[i].pid, board_list[i].type , board_list[i].appl_vers_major , board_list[i].appl_vers_minor , board_list[i].appl_vers_build);
    }

    return 0;
}


//*****************************************************************/
int cDownloader::get_canbus_id()
{
    return canbus_id;
}

void cDownloader::set_canbus_id(int id)
{
    canbus_id = id;
}

//*****************************************************************/

int cDownloader::startscheda(int bus, int board_pid, bool board_eeprom, int board_type)
{
    // check if driver is running
    if (m_idriver == NULL)
        {
            if(_verbose) yError ("START_CMD: Driver not ready\n");
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
        txBuffer[0].setId(build_id(ID_MASTER, board_pid));
        txBuffer[0].setLen(1);
        txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BOARD;

        //makes the first jump
        set_bus(txBuffer[0], bus);
        m_idriver->send_message(txBuffer, 1);
        drv_sleep(250);
        }
        break;
    case icubCanProto_boardType__skin:
    case icubCanProto_boardType__strain:
    case icubCanProto_boardType__mais:
    case icubCanProto_boardType__2foc:
    case icubCanProto_boardType__6sg:
    case icubCanProto_boardType__jog:
    case icubCanProto_boardType__mtb4:
    case icubCanProto_boardType__strain2:
    case icubCanProto_boardType__rfe:
    case icubCanProto_boardType__sg3:
    case icubCanProto_boardType__psc:
    case icubCanProto_boardType__mtb4w:
    case icubCanProto_boardType__unknown:
    {
        // Send command
        txBuffer[0].setId(build_id(ID_MASTER,board_pid));
        txBuffer[0].setLen(2);
        txBuffer[0].getData()[0]= ICUBCANPROTO_BL_BOARD;
        txBuffer[0].getData()[1]= (int) board_eeprom;

        //makes the first jump
        set_bus(txBuffer[0], bus);
        m_idriver->send_message(txBuffer, 1);
        drv_sleep(1500);
        }
        break;
    }
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);

    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("START_CMD: Unable to send message\n");
            return -1;
        }

    //pause
     drv_sleep(500);

    // riceve la risposta
    int read_messages = m_idriver->receive_message(rxBuffer);


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
    if(_verbose) yError ("START_CMD: No ACK received from board %d\n", board_pid);
    return -1;

}

//*****************************************************************/

int cDownloader::stopscheda(int bus, int board_pid)
{
    // check if driver is running
    if (m_idriver == NULL)
        {
            if(_verbose) yError ("STOP_CMD: Driver not ready\n");
            return -1;
        }

    // Send command
    txBuffer[0].setId(build_id(ID_MASTER, board_pid));
    txBuffer[0].setLen(1);
    txBuffer[0].getData()[0]= ICUBCANPROTO_BL_END;
    set_bus(txBuffer[0], bus);
    int ret = m_idriver->send_message(txBuffer, 1);

    // check if send_message was successful
    if (ret==0)
        {
            if(_verbose) yError ("STOP_CMD: Unable to send message\n");
            return -1;
        }

    //pause
    drv_sleep(5);

    // riceve la risposta
    int read_messages = m_idriver->receive_message(rxBuffer);

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
    if(_verbose) yError ("TOP_CMD: No ACK received from board %d\n", board_pid);
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
int cDownloader::verify_ack(int command, int read_messages)
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
                                        if(board_list[i].pid == get_src_from_id(rxBuffer[k].getId()))
                                        {
                                            #if defined(DOWNLOADER_USE_IDRIVER2)
                                            if(board_list[i].bus == rxBuffer[k].getCanBus())
                                            #else
                                            if(1)
                                            #endif
                                            {
                                                board_list[i].status=BOARD_DOWNLOADING;
                                            }
                                        }
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

int cDownloader::download_motorola_line(char* line, int len, int bus, int board_pid)
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
            if(_verbose) yError ("Failed Checksum\n");
            return -1;
        }

    //state: WAIT
    if (!(line[0] == 'S'))
        {
            if(_verbose) yError ("start tag character not found\n");
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
            set_bus(txBuffer[0], bus);
            ret = m_idriver->send_message(txBuffer,1);

            // check if send_message was successful
            if (ret==0)
                {
                    if(_verbose) yError ("Unable to send message\n");
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
                    set_bus(txBuffer[0], bus);
                    ret = m_idriver->send_message(txBuffer,1);

                    // check if send_message was successful
                    if (ret==0)
                        {
                            if(_verbose) yError ("Unable to send message\n");
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
            read_messages = m_idriver->receive_message(rxBuffer, nSelectedBoards);
            // fprintf(stderr, "%u\n", read_messages);
            //   fprintf(stderr, "Skipping ack\n");
            //return verify_ack(ICUBCANPROTO_BL_DATA, read_messages);
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
            set_bus(txBuffer[0], bus);
            ret = m_idriver->send_message(txBuffer, 1);

            // check if send_message was successful
            if (ret==0)
                {
                    if(_verbose) yError ("Unable to send message\n");
                    return -1;
                }

            //pause
            drv_sleep(10+5);

            // riceve la risposta
            read_messages = m_idriver->receive_message(rxBuffer);
            verify_ack(ICUBCANPROTO_BL_START, read_messages);
            return 0;

            break;


        default:
            if(_verbose) yError ("wrong format tag character %c (hex:%X)\n", sprsRecordType, sprsRecordType);
            return -1;

            break;
        }

    if(_verbose) yError ("Can't reach here!\n");
    return -1;
}

//*****************************************************************/
// This function read one line of the hexintel file and send it to a board using the correct protocol
// Return values:
// 0  one line downloaded, continuing the download...
// 1  Current downloading, everything OK
// -1 Fatal error

int cDownloader::download_hexintel_line(char* line, int len, int bus, int board_pid, bool eeprom, int board_type)
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
      if(_verbose) yError ("Failed Checksum\n");
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
                if(_verbose) yError("start tag character not found in hex file\n");
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

                // marco.accame on 25may17:
                // here is extra safety to avoid an accidental loading of stm32 code (which starts at 0x0800) on dspic based boards.
                // the bootloader on dspic boards in such a case erases the first sector which tells to execute to the bootloader
                // at reset with teh result that the board become unreachable.
                // as we shall release strain2.hex and mtb4.hex which use stm32 mpus w/ code at 0x0800 and beyond, any accidental
                // attempt to program an old strain w/ strain2.hex becomes more probable. As the damage is high (removal of the the
                // FT sensor + disassembly + re-programming + recalibration), some sort of protection is mandatory.
                // instead, strain2/mtb4 are safe if any attempt is done to program them with old strain.hex/skin.hex code
                if(sprsPage >= 0x0800)
                {   // only mtb4, strain2, rfe, sg3, psc, mtb4w are allowed to use such a code space.
                    if((icubCanProto_boardType__mtb4 == board_type) || (icubCanProto_boardType__strain2 == board_type) ||
                       (icubCanProto_boardType__rfe == board_type) || (icubCanProto_boardType__sg3 == board_type) ||
                       (icubCanProto_boardType__psc == board_type) || (icubCanProto_boardType__mtb4w == board_type)
                      )
                    {   // it is ok
                    }
                    else
                    {   // be careful with that axe, eugene. ahhhhhhhhhhhhhhhh
                        //if(_verbose)
                        {
                            char msg[32] = {0};
                            snprintf(msg, sizeof(msg), "0x%04X", sprsPage);
                            yError() << "Upload of FW to board" << eoboards_type2string2((eObrd_type_t)board_type, eobool_true) << "is aborted because it was detected a wrong page number =" << msg << "in the .hex file";
                            yError() << "You must have loaded the .hex file of another board. Perform a new discovery, check the file name and retry.";
                        }
                        return -1;
                    }
                }


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
                    set_bus(txBuffer[0], bus);
                    ret = m_idriver->send_message(txBuffer,1);
                    // check if send_message was successful
                    if (ret==0)


                    {
                        if(_verbose) yError ("Unable to send message\n");
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
                        set_bus(txBuffer[0], bus);
                        ret = m_idriver->send_message(txBuffer,1);

                        // check if send_message was successful
                        if (ret==0)
                            {
                                if(_verbose) yError ("Unable to send message\n");
                                return -1;
                            }
                        //pause
                        drv_sleep(5);
                    }
                    //receive one ack for the whole line
                    read_messages = m_idriver->receive_message(rxBuffer,nSelectedBoards, 10);
                    ret=verify_ack(ICUBCANPROTO_BL_DATA, read_messages);
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
                    set_bus(txBuffer[0], bus);
                    ret = m_idriver->send_message(txBuffer,1);
                    // check if send_message was successful
                    if (ret==0)
                    {
                        if(_verbose) yError ("Unable to send message\n");
                        return -1;
                    }
                    //pause
                    drv_sleep(5);
                    //receive the ack from the board
                    read_messages = m_idriver->receive_message(rxBuffer);
                    ret=verify_ack(ICUBCANPROTO_BL_START, read_messages);
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
                if(_verbose) yError ("Checksum Error\n");
            }
        break;
        } //end switch
    }
    while(true);

    if(_verbose) yError ("Can't reach here!\n");
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
            if(_verbose) yError ("Error opening file!\n");
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
            if(_verbose) yError ("Error opening file!\n");
            return -1;
        }

    return 0;
}

//*****************************************************************/
// Return values:
// 0  Download terminated, everything OK
// 1  Current downloading, everything OK
// -1 Fatal error in sending one command
int cDownloader::download_file(int bus, int board_pid, int download_type, bool board_eeprom)
{

    if (!filestr.is_open())
        {
            if(_verbose) yError ("File not open!\n");
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
                             ret = download_motorola_line(buffer, strlen(buffer), bus, board_pid);
                        break;
                        case icubCanProto_boardType__pic:
                        case icubCanProto_boardType__skin:
                        case icubCanProto_boardType__strain:
                        case icubCanProto_boardType__mais:
                        case icubCanProto_boardType__2foc:
                        case icubCanProto_boardType__jog:
                        case icubCanProto_boardType__6sg:
                        case icubCanProto_boardType__mtb4:
                        case icubCanProto_boardType__strain2:
                        case icubCanProto_boardType__rfe:
                        case icubCanProto_boardType__sg3:
                        case icubCanProto_boardType__psc:
                        case icubCanProto_boardType__mtb4w:
                             ret = download_hexintel_line(buffer, strlen(buffer), bus, board_pid, board_eeprom, download_type);

                        break;
                        case icubCanProto_boardType__unknown:
                        default:
                             ret =-1;
                        break;
                    }
                    if (ret != 0)
                        {
                            if(_verbose) yError("fatal error during download: abort\n");
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

void cDownloader::clean_rx(void)
{
    m_idriver->receive_message(rxBuffer,64,0.001);
}

#if defined(DOWNLOADER_USE_IDRIVER2)

void cDownloader::set_bus(CanPacket &pkt, int bus)
{
    pkt.setCanBus(bus);
}

int cDownloader::get_bus(CanPacket &pkt)
{
    return pkt.getCanBus();
}

#else

void cDownloader::set_bus(yarp::dev::CanMessage &msg, int bus)
{
    // nothing
}

int cDownloader::get_bus(yarp::dev::CanMessage &msg)
{
    return get_canbus_id();
}

#endif



int cDownloader::strain_calibrate_offset2_strain1 (int bus, int target_id, int16_t t, string *errorstring)
{
#if 1
    return strain_calibrate_offset2_strain1safer(bus, target_id, t, 2, false, errorstring);
#else
    // check if driver is running
    if (m_idriver == NULL)
    {
       if(_verbose) yError ("Driver not ready\n");
       return -1;
    }

    // marco.accame: transform [-32K, +32K) into [0, +64K)
    unsigned int middle_val = 32768 + t;

    // in strain1 we dont have the concept of regulationset. however, if we use strain_regset_inuse which is = 0 we are ok.
    const int regset = 0; // strain_regset_inuse;

    int daclimit = 0x3ff;
    int dacstep = 1;
    long tolerance = 256;

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
        // yDebug() << "starting OFFSET of channel" << channel;
        // Send read channel command to strain board
        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(3);
        txBuffer[0].getData()[0]= 0x0C;
        txBuffer[0].getData()[1]= channel;
        txBuffer[0].getData()[2]= 0;
        set_bus(txBuffer[0], bus);
        ret = m_idriver->send_message(txBuffer, 1);
        // check if send_message was successful
        if (ret==0)
            {
                if(_verbose) yError ("Unable to send message\n");
                return -1;
            }
        //read adc
        read_messages = m_idriver->receive_message(rxBuffer,1);
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
        txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
        txBuffer[0].getData()[1]=  (channel & 0x0f);
        set_bus(txBuffer[0], bus);
        ret = m_idriver->send_message(txBuffer, 1);

        read_messages = m_idriver->receive_message(rxBuffer,1);
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

        while (abs(error)>tolerance && cycle<daclimit)
        {
            if (error>0) dac -= dacstep;
            else         dac += dacstep;

            if (dac>daclimit)       dac = daclimit;
            if (dac<0)              dac = 0;

            //yDebug() << "iter" << cycle << "err = " << error << "next dac =" << dac;

            // Send transmission command to strain board
            txBuffer[0].setId((2 << 8) + target_id);
            txBuffer[0].setLen(4);
            txBuffer[0].getData()[0]= 0x04;
            txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel & 0x0f);
            txBuffer[0].getData()[2]= dac >> 8;
            txBuffer[0].getData()[3]= dac & 0xFF;
            set_bus(txBuffer[0], bus);
            int ret = m_idriver->send_message(txBuffer, 1);

            //wait
            drv_sleep(3);

            // Send read channel command to strain board
            txBuffer[0].setId((2 << 8) + target_id);
            txBuffer[0].setLen(3);
            txBuffer[0].getData()[0]= 0x0C;
            txBuffer[0].getData()[1]= channel;
            txBuffer[0].getData()[2]= 0;
            set_bus(txBuffer[0], bus);
            ret = m_idriver->send_message(txBuffer, 1);
            // check if send_message was successful
            if (ret==0)
            {
                if(_verbose) yError ("Unable to send message\n");
                return -1;
            }
            //read adc
            read_messages = m_idriver->receive_message(rxBuffer, 1);
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
#endif
}


int cDownloader::strain_calibrate_offset2_strain2(int bus, int target_id, const std::vector<strain2_ampl_discretegain_t> &gains, const std::vector<int16_t> &targets, string *errorstring)
{
    // the calibration of the offset is meaningful only for the calibration set in use.
    const int regset = strain_regset_inuse;
    const unsigned int NUMofCHANNELS = 6;

    std::ostringstream ss;

    // check if driver is running
    if (m_idriver == NULL)
    {
       if(_verbose) yError ("Driver not ready\n");
       Log(std::string("strain_calibrate_offset2_strain2(): failure. driver no ready"));
       return -1;
    }


    ss.str("");
    ss.clear();
    ss << "performing offset autotuning for strain2";
    Log(ss.str());

    // step 1: apply the gains. the initial offset will be meaning less. however strain_set_amplifier_discretegain() assign offsets all equal to 32k-1
    for(int channel=0; channel<NUMofCHANNELS; channel++)
    {
        ss.str("");
        ss.clear();
        ss << "- on ch " << std::to_string(channel) << ": we impose g = " << std::to_string(static_cast<int>(strain_amplifier_discretegain2float(gains[channel])));

        if(0 != strain_set_amplifier_discretegain(bus, target_id, channel, gains[channel], regset, errorstring))
        {
            if(_verbose)
            {
                Log(ss.str());
                Log("strain_calibrate_offset2_strain2(): failure of strain_set_amplifier_discretegain()");
            }
            return -1;
        }
        // i wait some time
        yarp::os::Time::delay(1.0);

        float gaain = 0;
        uint16_t ooffset = 0;
        strain_get_amplifier_gain_offset(bus, target_id, channel, gaain, ooffset, regset, errorstring);

        ss << " and read (g, o) = (" << std::to_string(static_cast<int>(gaain)) << ", " << std::to_string(ooffset) << ")";
        Log(ss.str());
    }



    yarp::os::Time::delay(2.0);


    // step 3: eval if the targets are equal or not. if they are all equal we send a single command.
    //         if not we must send one command per channel.

    int16_t singletargetVALUE = targets[0];
    bool singletargetTHEREIS = true;
    for(int channel=1; channel<NUMofCHANNELS; channel++)
    {
        if(singletargetVALUE != targets[channel])
        {
            singletargetTHEREIS = false;
            break;
        }
    }


    long tolerance = 256;



    tolerance = 256;
    uint8_t samples2average = 8; // if zero, the board uses its default (= 4)



    // step3: autocalib
    const uint8_t everychannel = 0x0f;

    uint8_t channel2autocalib = everychannel;       // the channel(s) ...
    unsigned int middle_val = 32768;                // transform [-32K, +32K) into [0, +64K)
    uint8_t okmask = 0x3f; // all six channel

    if(true == singletargetTHEREIS)
    {

        channel2autocalib = everychannel;
        middle_val = 32768 + singletargetVALUE;
        okmask = 0x3f; // all six channel


        ss.str("");
        ss.clear();
        ss << "STEP-2. there is a single ADC target: performing parallel regularization";
        Log(ss.str());

        ss.str("");
        ss.clear();
        ss << " params: target = " << std::to_string(singletargetVALUE) << " tolerance = " << std::to_string(tolerance) << " samples2average = " << std::to_string(samples2average);

        // sending an autocalib message
        txBuffer[0].setId((2 << 8) + target_id);
        txBuffer[0].setLen(8);
        txBuffer[0].getData()[0]= 0x22;
        txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel2autocalib & 0x0f);
        txBuffer[0].getData()[2]= 0; // mode oneshot, the only possible so far
        txBuffer[0].getData()[3]= middle_val & 0x00ff;          // little endian
        txBuffer[0].getData()[4]= (middle_val >> 8) & 0x00ff;   // little endian
        txBuffer[0].getData()[5]= tolerance & 0x00ff;          // little endian
        txBuffer[0].getData()[6]= (tolerance >> 8) & 0x00ff;   // little endian
        txBuffer[0].getData()[7]= samples2average;
        set_bus(txBuffer[0], bus);
       // yDebug("strain2-amplifier-tuning: STEP-3. sent message = [%x, %x, %x, %x, %x, %x, %x, %x]", txBuffer[0].getData()[0], txBuffer[0].getData()[1], txBuffer[0].getData()[2], txBuffer[0].getData()[3], txBuffer[0].getData()[4], txBuffer[0].getData()[5], txBuffer[0].getData()[6], txBuffer[0].getData()[7]);
        int ret = m_idriver->send_message(txBuffer, 1);
        // check if send_message was successful
        if (ret==0)
        {
            if(_verbose) yError ("Unable to send message\n");
            return -1;
        }

        // now wait for a reply for most 3 seconds
        double TOUT = 3.0;

        ss.str("");
        ss.clear();
        ss << "STEP-3. results ...";
        Log(ss.str());

        int read_messages = m_idriver->receive_message(rxBuffer, 1, TOUT);
        for(int i=0; i<read_messages; i++)
        {
            if (rxBuffer[i].getData()[0]==0x22)
            {
                //yDebug("strain2-amplifier-tuning: STEP-3. received message = [%x, %x, %x, %x, %x, %x, %x, %x]", rxBuffer[0].getData()[0], rxBuffer[0].getData()[1], rxBuffer[0].getData()[2], rxBuffer[0].getData()[3], rxBuffer[0].getData()[4], rxBuffer[0].getData()[5], rxBuffer[0].getData()[6], rxBuffer[0].getData()[7]);

                //dac = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                uint8_t noisychannelmask = rxBuffer[i].getData()[2];
                uint8_t algorithmOKmask = rxBuffer[i].getData()[3];
                uint8_t finalmeasureOKmask = rxBuffer[i].getData()[4];
                uint16_t mae =  (static_cast<uint32_t>(rxBuffer[i].getData()[6]))       |
                                (static_cast<uint32_t>(rxBuffer[i].getData()[7]) << 8);

                if((okmask == algorithmOKmask) && (okmask == finalmeasureOKmask))
                {
                    ss.str("");
                    ss.clear();
                    ss << " OK w/ MAE = " << std::to_string(mae);
                    Log(ss.str());

                    if(0 != noisychannelmask)
                    {
                        ss.str("");
                        ss.clear();
                        ss << " BUT noisy acquisition of samples: ";
                        if((0x40 & noisychannelmask) == 0x40)
                        {
                            ss << " in computing average ADC before algorithm ";
                        }
                        if((0x80 & noisychannelmask) == 0x80)
                        {
                            ss << "after algorithm in computing MAE";
                        }
                        Log(ss.str());

                        ss.str("");
                        ss.clear();
                        char tmp[256] = {0};
                        snprintf(tmp, sizeof(tmp), "noisychannelmask = 0x%x, algorithmOKmask = 0x%x, finalmeasureOKmask = 0x%x, mae = %d", noisychannelmask, algorithmOKmask, finalmeasureOKmask, mae);
                        ss <<  "COMPLETE RES: " << tmp;
                        Log(ss.str());
                    }

                }
                else
                {
                    ss.str("");
                    ss.clear();
                    ss << " KO: ";
                    char tmp[256] = {0};
                    snprintf(tmp, sizeof(tmp), "noisychannelmask = 0x%x, algorithmOKmask = 0x%x, finalmeasureOKmask = 0x%x, mae = %d", noisychannelmask, algorithmOKmask, finalmeasureOKmask, mae);
                    ss << tmp;
                    Log(ss.str());

                    if(0 != noisychannelmask)
                    {
                        ss.str("");
                        ss.clear();
                        ss << " WITH noisy acquisition of samples: ";
                        if((0x40 & noisychannelmask) == 0x40)
                        {
                            ss << " in computing average ADC before algorithm ";
                        }
                        if((0x80 & noisychannelmask) == 0x80)
                        {
                            ss << "after algorithm in computing MAE";
                        }
                        Log(ss.str());
                    }

                    for(uint8_t channel=0; channel<NUMofCHANNELS; channel++)
                    {
                        ss.str("");
                        ss.clear();
                        bool problems = false;
                        ss << "- on ch " << std::to_string(channel) << ":";
                        if((algorithmOKmask & (0x01<<channel)) == 0)
                        {
                            problems =  true;
                            ss << " [algorithm fails]";
                        }
                        if((finalmeasureOKmask & (0x01<<channel)) == 0)
                        {
                            problems =  true;
                            ss << " [mae is high (does ADC work?)]";
                        }
                        if(((noisychannelmask) & (0x01<<channel)) == (0x01<<channel))
                        {
                            problems =  true;
                            ss << " [noisy acquition]";
                        }
                        if(!problems)
                        {
                            ss << " [no detected problem]";
                        }
                        Log(ss.str());
                    }
                }
                break;
            }
        }
    }
    else
    {
        // for all six channels
        ss.str("");
        ss.clear();
        ss << "STEP-2. there are multiple ADC targets: performing regularization channel by channel";
        Log(ss.str());

        ss.str("");
        ss.clear();
        ss << " common params: tolerance = " << std::to_string(tolerance) << " samples2average = " << std::to_string(samples2average);
        Log(ss.str());


        for(int channel=0; channel<NUMofCHANNELS; channel++)
        {
            channel2autocalib = channel;
            middle_val = 32768 + targets[channel];
            okmask = 0x01 << channel;

            ss.str("");
            ss.clear();
            ss << "- on ch " << std::to_string(channel) << ": ADC target = " << std::to_string(targets[channel]);
            Log(ss.str());


            // send message, check results ...

            // sending an autocalib message
            txBuffer[0].setId((2 << 8) + target_id);
            txBuffer[0].setLen(8);
            txBuffer[0].getData()[0]= 0x22;
            txBuffer[0].getData()[1]= ((regset << 4) & 0xf0) | (channel2autocalib & 0x0f);
            txBuffer[0].getData()[2]= 0; // mode oneshot, the only possible so far
            txBuffer[0].getData()[3]= middle_val & 0x00ff;          // little endian
            txBuffer[0].getData()[4]= (middle_val >> 8) & 0x00ff;   // little endian
            txBuffer[0].getData()[5]= tolerance & 0x00ff;          // little endian
            txBuffer[0].getData()[6]= (tolerance >> 8) & 0x00ff;   // little endian
            txBuffer[0].getData()[7]= samples2average;
            set_bus(txBuffer[0], bus);
            int ret = m_idriver->send_message(txBuffer, 1);
            // check if send_message was successful
            if (ret==0)
            {
                if(_verbose) yError ("Unable to send message\n");
                return -1;
            }

            // now wait for a reply for most 3 seconds
            double TOUT = 3.0;


            int read_messages = m_idriver->receive_message(rxBuffer, 1, TOUT);
            for(int i=0; i<read_messages; i++)
            {
                if (rxBuffer[i].getData()[0]==0x22)
                {
                    //yDebug("strain2-amplifier-tuning: STEP-3. received message = [%x, %x, %x, %x, %x, %x, %x, %x]", rxBuffer[0].getData()[0], rxBuffer[0].getData()[1], rxBuffer[0].getData()[2], rxBuffer[0].getData()[3], rxBuffer[0].getData()[4], rxBuffer[0].getData()[5], rxBuffer[0].getData()[6], rxBuffer[0].getData()[7]);

                    //dac = rxBuffer[i].getData()[2]<<8 | rxBuffer[i].getData()[3];
                    uint8_t noisychannelmask = rxBuffer[i].getData()[2];
                    uint8_t algorithmOKmask = rxBuffer[i].getData()[3];
                    uint8_t finalmeasureOKmask = rxBuffer[i].getData()[4];
                    uint16_t mae =  (static_cast<uint32_t>(rxBuffer[i].getData()[6]))       |
                                    (static_cast<uint32_t>(rxBuffer[i].getData()[7]) << 8);

                    if((okmask == algorithmOKmask) && (okmask == finalmeasureOKmask))
                    {
                        ss.str("");
                        ss.clear();
                        ss << "  OK w/ MAE = " << std::to_string(mae);
                        Log(ss.str());

                        if(0 != (noisychannelmask & okmask))
                        {
                            ss.str("");
                            ss.clear();
                            ss << "  BUT noisy acquisition of samples: ";
                            Log(ss.str());

                            if((0x40 & noisychannelmask) == 0x40)
                            {
                                ss.str("");
                                ss.clear();
                                ss << "  - in computing average ADC before algorithm ";
                                Log(ss.str());
                            }
                            if((0x80 & noisychannelmask) == 0x80)
                            {
                                ss.str("");
                                ss.clear();
                                ss << "  - after algorithm in computing MAE";
                                Log(ss.str());
                            }

                            ss.str("");
                            ss.clear();
                            char tmp[256] = {0};
                            snprintf(tmp, sizeof(tmp), "noisychannelmask = 0x%x, algorithmOKmask = 0x%x, finalmeasureOKmask = 0x%x, mae = %d", noisychannelmask, algorithmOKmask, finalmeasureOKmask, mae);
                            ss <<  "COMPLETE RES: " << tmp;
                            Log(ss.str());
                        }

                    }
                    else
                    {

                        ss.str("");
                        ss.clear();
                        ss << "  KO /w MAE = " << std::to_string(mae) << " because: ";
                        Log(ss.str());

                        ss.str("");
                        ss.clear();
                        char tmp[256] = {0};
                        snprintf(tmp, sizeof(tmp), "  KO details: noisychannelmask = 0x%x, algorithmOKmask = 0x%x, finalmeasureOKmask = 0x%x, mae = %d", noisychannelmask, algorithmOKmask, finalmeasureOKmask, mae);
                        ss << tmp;
                        Log(ss.str());

                        bool problems = false;
                        if((algorithmOKmask & okmask) == 0)
                        {
                            problems =  true;
                            ss.str("");
                            ss.clear();
                            ss << "  - algorithm fails";
                            Log(ss.str());
                        }
                        if((finalmeasureOKmask & okmask) == 0)
                        {
                            problems =  true;
                            ss.str("");
                            ss.clear();
                            ss << "  - mae is high (does ADC work?)";
                            Log(ss.str());
                        }
                        if(!problems)
                        {
                            ss.str("");
                            ss.clear();
                            ss << " .. strange: no detected problem";
                            Log(ss.str());
                        }

                    }
                    break;
                }
            }
        }
    }

    return 0;
}

int cDownloader::readADC(int bus, int target_id, int channel, int nmeasures)
{
    const int type = 0; // raw

    txBuffer[0].setId((2 << 8) + target_id);
    txBuffer[0].setLen(3);
    txBuffer[0].getData()[0]= 0x0C;
    txBuffer[0].getData()[1]= channel;
    txBuffer[0].getData()[2]= type;

    int measure = 0;

    for(int n=0; n<nmeasures; n++)
    {
        int tmp = 0;
        set_bus(txBuffer[0], bus);
        int ret = m_idriver->send_message(txBuffer, 1);
        // check if send_message was successful
        if (ret==0)
        {
            yError ("Unable to send message\n");
            return 0;
        }
        //read adc
        int read_messages = m_idriver->receive_message(rxBuffer,1);
        for (int i=0; i<read_messages; i++)
        {
            if (rxBuffer[i].getData()[0]==0x0C)
            {
                tmp = (rxBuffer[i].getData()[3]<<8 | rxBuffer[i].getData()[4]);
                break;
            }
            else
            {
                printf("cDownloader::strain_calibrate_offset2_strain1(): fails in reading reply for a measure\n");
            }
        }

        measure += tmp;
    }

    measure /= nmeasures;

    return measure;
}

int cDownloader::strain_calibrate_offset2_strain1safer (int bus, int target_id, int16_t t, uint8_t nmeasures, bool fullsearch, string *errorstring)
{
    // check if driver is running
    if (m_idriver == NULL)
    {
       if(_verbose) yError ("Driver not ready\n");
       return -1;
    }

    std::ostringstream ss;

    // marco.accame: transform [-32K, +32K) into [0, +64K)
    unsigned int middle_val = 32768 + t;


    if(fullsearch)
    {
        //yDebug() << "performing full search in dac space to find best value to match adc ="<< t << " using" << nmeasures << "adc acquisions for better averaging";

        ss.str("");
        ss.clear();
        ss << "performing offset autotuning for strain1 in full search mode";
        Log(ss.str());

        ss.str("");
        ss.clear();
        ss << "params: " << "adc target =" << std::to_string(t) << " using" << std::to_string(nmeasures) << "adc acquisions for better averaging";
        Log(ss.str());

        for(int channel=0; channel<6; channel++)
        {

            long minABSerror = 128*1024;
            unsigned int minDAC = 0;

            // loop over all possible dac values
            for(unsigned int testdac=0; testdac<1024; testdac++)
            {
                 // send new dac
                strain_set_offset(bus, target_id, channel, testdac);
                //wait
                drv_sleep(3);
                // verify it
                unsigned int tmp = 0;
                strain_get_offset(bus, target_id, channel, tmp);
                if(tmp != testdac)
                {
                    yError() << "failed to impose DAC = " << testdac << "read:" << tmp;
                }

                // read value
                int measure = readADC(bus, target_id, channel, nmeasures);
                // compute error vs target
                long error = long(measure) - long(middle_val);

                if(fabs(error) < fabs(minABSerror))
                {
                    minABSerror = fabs(error);
                    minDAC = testdac;
                    //yDebug() << "PROGESS: channel =" << channel << "minerror = " << minABSerror << "mindac =" << minDAC;
                }
            }

            // apply the best dac
            strain_set_offset(bus, target_id, channel, minDAC);
            // wait
            drv_sleep(3);
            // verify it
            unsigned int tmp = 0;
            strain_get_offset(bus, target_id, channel, tmp);
            if(tmp != minDAC)
            {
                yError() << "failed to impose DAC = " << minDAC << "read:" << tmp;
            }

            ss.str("");
            ss.clear();
            ss << "RESULT of FULL SEARCH w/ nsamples average =" << std::to_string(nmeasures) << "-> channel =" << std::to_string(channel) << "minerror = " << std::to_string(minABSerror) << "mindac =" << std::to_string(minDAC);
            Log(ss.str());

            //yDebug() << "RESULT of FULL SEARCH w/ nsamples average =" << nmeasures << "-> channel =" << channel << "minerror = " << minABSerror << "mindac =" << minDAC;

        } // channel

    }
    else
    {
        const int daclimit = 0x3ff;
        const int dacstep = 1;
        const long tolerance = 128;
        const int maxiterations = 1024;

        ss.str("");
        ss.clear();
        ss << "performing gradient descend in dac space to find best value to match adc ="<< std::to_string(t) << " using" << std::to_string(nmeasures) << "adc acquisions for better averaging";
        Log(ss.str());

        ss.str("");
        ss.clear();
        ss << "exit conditions: max number of iterations =" << std::to_string(maxiterations) << "error tolerance =" << std::to_string(tolerance);
        Log(ss.str());

        for(int channel=0; channel<6; channel++)
        {

            long minABSerror = 128*1024;
            unsigned int minDAC = 0;
            unsigned int dac = 0;


            int measure = readADC(bus, target_id, channel, nmeasures);

            // read dac
            strain_get_offset(bus, target_id, channel, dac);

            long error = long(measure) - long(middle_val);
            int cycle =0;

            minABSerror = fabs(error);
            minDAC = dac;

            // now i perform a sort of gradient descend
            while ((abs(error)>tolerance) && (cycle<daclimit) && (cycle<maxiterations))
            {
                if (error>0) dac -= dacstep;
                else         dac += dacstep;

                if (dac>daclimit)       dac = daclimit;
                if (dac<0)              dac = 0;

                //yDebug() << "channel =" << channel << "iter =" << cycle << "err = " << error << "next dac =" << dac << "minerror = " << minABSerror << "mindac =" << minDAC;

                // send new dac
                strain_set_offset(bus, target_id, channel, dac);
                //wait
                drv_sleep(3);
                // verify it
                unsigned int tmp = 0;
                strain_get_offset(bus, target_id, channel, tmp);
                if(tmp != dac)
                {
                    yError() << "failed to impose DAC = " << dac << "read:" << tmp;
                }

                // read value
                measure = readADC(bus, target_id, channel, nmeasures);

                error = long(measure) - long(middle_val);
                cycle++;

                if(fabs(error) < fabs(minABSerror))
                {
                    minABSerror = fabs(error);
                    minDAC = dac;
                }
            }

            ss.str("");
            ss.clear();
            ss << "RESULT of gradient descend  w/ nsamples average =" << std::to_string(nmeasures) << " -> channel =" << std::to_string(channel) << " num iters =" << std::to_string(cycle) << " err = " << std::to_string(error) << " applied dac =" << std::to_string(dac) << " minerror = " << std::to_string(minABSerror) << " mindac =" << std::to_string(minDAC);
            Log(ss.str());

        }   // channel

    }


    return 0;
}





// eof



