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
#ifndef _TEMPLATE_BUTTONS_H_
#define _TEMPLATE_BUTTONS_H_



/** @file       template_buttons.hpp
    @brief
    @author     valentina.gaggero@iit.it
    @date       03/22/2013
**/

/** @defgroup

    @{
 **/


// - external dependencies --------------------------------------------------------------------------------------------

#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include <string>
#include <signal.h>
#include <iostream>

using namespace std;

// Ace stuff
#include <ace/ACE.h>
#include "ace/SOCK_Dgram.h"
#include "ace/Addr.h"
#include "ace/Thread.h"
#include "ace/Logging_Strategy.h"


// - public #define  --------------------------------------------------------------------------------------------------



// - declaration of public user-defined types -------------------------------------------------------------------------


// - declaration of extern public variables, ... but better using use _get/_set instead -------------------------------
// empty-section


// - declaration of extern public functions ---------------------------------------------------------------------------
void commands(void);

uint32_t callback_button_0(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_1(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_2(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_3(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_4(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_5(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_6(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_7(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_8(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_9(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
uint32_t callback_button_10(uint8_t *payload_ptr, uint32_t payload_size, ACE_INET_Addr *addr);
/** @}
    end of group
 **/



#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------

