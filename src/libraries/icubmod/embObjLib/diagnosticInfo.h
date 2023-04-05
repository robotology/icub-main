/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */


#ifndef __diagnosticInfo_h__
#define __diagnosticInfo_h__

#include <string>


namespace Diagnostic {
    enum class SeverityOfError {info=0, debug =1, warning =2, error=3, fatal=4}; //TBD
    class EmbeddedInfo;
    class TimeOfInfo;
}

class Diagnostic::TimeOfInfo
{
public:
    uint32_t    sec;
    uint32_t    msec;
    uint32_t    usec;

    void toString(std::string &str_toi);

};


class Diagnostic::EmbeddedInfo
{
public:
    std::string                   sourceBoardIpAddrStr; // is the ipv4 address, in string, of the board that sends the diagnostic information 
    std::string                   sourceBoardName; // is the name of  the board that sends the diagnostic information
    std::string                   sourceCANPortStr; // if the diagnostic info si sent by a board on CAN, this field contains the CAN port expressed in string
    uint8_t                       sourceCANBoardAddr; // if the diagnostic info si sent by a board on CAN, this field contains the CAN address of the board
    std::string                   finalMessage; // contains the final diagostic message after the parsering
    Diagnostic::SeverityOfError   severity; // is the severity of the message
    Diagnostic::TimeOfInfo        timeOfInfo; //it is the time of the board when it sent the message
    std::string                   axisName; //if the error contains the joint number, then axisName contain its name.

public:
    void printMessage();
};



#endif //__diagnosticInfo_h__
