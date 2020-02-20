/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author:  Valentina Gaggero
 * email:   valentina.gaggero@iit.it
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


// - include guard ----------------------------------------------------------------------------------------------------
#ifndef _WRAPPER_PCAP_H_
#define _WRAPPER_PCAP_H_




/** @file       wrappePcap_linux.h
    @brief
    @author     valentina.gaggero@iit.it
    @date       03/19/2013
**/

/** @defgroup

    @{
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "EoCommon.h"
#include <pcap.h>



// - public #define  --------------------------------------------------------------------------------------------------



// - declaration of public user-defined types -------------------------------------------------------------------------


// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------

extern uint8_t wrapperPcap_init(char* dev, char*filter_expr);

extern uint8_t wrapperPcap_loop(int32_t cnt, pcap_handler callback, uint8_t *user);

extern void wrapperPcap_close(void);
/** @}
    end of group
 **/



#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

