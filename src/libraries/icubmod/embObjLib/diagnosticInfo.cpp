/*
 * Copyright (C) Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include "diagnosticInfo.h"
#include <yarp/os/LogStream.h>


using namespace Diagnostic;


/**************************************************************************************************************************/
/******************************************        TimeOfInfo       ***************************************************/
/**************************************************************************************************************************/

void TimeOfInfo::toString(std::string &str_toi)
{
    char str[50];
    snprintf(str, sizeof(str), "%ds %dm %du", sec, msec, usec);
    str_toi.clear();
    str_toi.append(str);
}



/**************************************************************************************************************************/
/******************************************        Info       ***************************************************/
/**************************************************************************************************************************/

void EmbeddedInfo::printMessage()
{
    if (finalMessage.size() == 0)
        return;
   
   std::string str_toi;
   timeOfInfo.toString(str_toi);

    std::string final_str = "from BOARD " + sourceBoardIpAddrStr + " (" + sourceBoardName +") time=" + str_toi + " : " + finalMessage;
    switch(severity)
    {
        case SeverityOfError::info:
        {
            yInfo() << final_str;
        } break;

        case SeverityOfError::debug:
        {
            yDebug() << final_str;
        } break;

        case SeverityOfError::warning:
        {
            yWarning() << final_str;
        } break;

        case SeverityOfError::error:
        {
            yError() << final_str;
        } break;

        case SeverityOfError::fatal:
        {
            yError() << "EMS received the following FATAL error: " <<final_str;
        } break;

        default:
        {
            yError() << final_str;
        } break;
    }


}

