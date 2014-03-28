/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __iCubIDLClients_H__
#define __iCubIDLClients_H__

// whenever a new service (e.g. new_service_IDL) becomes available,
// you have to add up two more lines to this file:
// 1. #include "new_service_IDL.h"
// 2. @ref new_service_IDL

#include "dataSetPlayer_IDL.h"
#include "depth2kin_IDL.h"
#include "fingersTuner_IDL.h"

/**
*
* @defgroup iCubIDLClients IDL Client Interfaces
* @ingroup icub_libraries 
*  
* This library contains all the definitions for a client to 
* interface to the IDL services made available by iCub 
* software. \n 
* Available services are listed below: \n 
* - @ref dataSetPlayer_IDL 
* - @ref depth2kin_IDL 
* - @ref fingersTuner_IDL 
*/

#endif

