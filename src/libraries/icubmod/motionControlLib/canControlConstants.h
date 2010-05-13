// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2008 The RobotCub Consortium
 * Author: Lorenzo Natale
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

#ifndef __CANCONTROLCONSTANTS__
#define __CANCONTROLCONSTANTS__

#define BUF_SIZE 2047
const int debug_mask = 0x20;

/**
 * Max number of addressable cards in this implementation.
 */
const int CAN_MAX_CARDS= 16;
const int ESD_MAX_CARDS= 16;

const int CANCONTROL_MAX_THREADS=500;

/**
 * Max number of addressable cards in this implementation.
 */
const int PLXCAN_MAX_CARDS= 16;
const int MAX_THREADS=200;


#endif
