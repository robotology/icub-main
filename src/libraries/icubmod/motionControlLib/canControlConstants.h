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

// this constant determines the number of requests (messages)
// that can be queued by a thread before waiting on synch().
// the worst case is when a request is sent to all joints of a 
// netowork, we use 50 to be conservative.
const int BUF_SIZE=50;
const int debug_mask = 0x20;

/**
 * Max number of addressable cards in this implementation.
 */
const int CAN_MAX_CARDS= 16;
const int ESD_MAX_CARDS= 16;

// Max number of threads allowed to communicate with the
// CanBusMotionControl object at the same time.
// Since the thread table is allocated statically 
// this value is a constant. 
const int CANCONTROL_MAX_THREADS=500;

/**
 * Max number of addressable cards in this implementation.
 */
const int PLXCAN_MAX_CARDS= 16;

#endif
