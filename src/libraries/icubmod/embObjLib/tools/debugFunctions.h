// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) <year>  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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

#ifndef __debugfunctions__
#define __debugfunctions__

// usual includes
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include <string>
#include <signal.h>
#include <iostream>

// ACE stuff
#include <ace/ACE.h>
#include "ace/SOCK_Dgram.h"
#include "ace/Addr.h"
#include "ace/Thread.h"

// EO includes
//#include "EOnv_hid.h"
//#include "EOropframe_hid.h"
//#include "embObjLibInterface.h"

#define _LINUX_UDP_SOCKET_
//#undef _LINUX_UDP_SOCKET_


#define _DEEP_DEBUG_

#define eb1_ip "10.0.0.1"
#define eb2_ip "10.0.1.2"
#define eb3_ip "10.255.72.101"
#define eb4_ip "10.0.1.4"
#define eb5_ip "192.168.202.1"
#define eb6_ip "10.0.0.100"


#define	BOARD_NUM					7
#define SOGLIA						1700
#define MAX_ACQUISITION 			100*1000


bool check_received_pkt(ACE_INET_Addr *sender_addr, void * pkt, ACE_UINT16 pkt_size);
bool check_received_pkt(sockaddr *sender_addr, void * pkt, ACE_UINT16 pkt_size);
bool do_real_check(int board, void * pkt, ACE_UINT16 pkt_size);

void print_data(void);

int gettimeofday(struct timeval *tv, struct timezone *tz);

#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64

struct timezone
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};

#else
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif

static int timeval_subtract(struct timeval *_result, struct timeval *_x, struct timeval *_y)
{
	/* Perform the carry for the later subtraction by updating y. */

	if(_x->tv_usec < _y->tv_usec)
	{
		int nsec    = (_y->tv_usec - _x->tv_usec) / 1000000 + 1;

		_y->tv_usec -= 1000000 * nsec;
		_y->tv_sec  += nsec;
	}

	if(_x->tv_usec - _y->tv_usec > 1000000)
	{
		int nsec    = (_x->tv_usec - _y->tv_usec) / 1000000;

		_y->tv_usec += 1000000 * nsec;
		_y->tv_sec  -= nsec;
	}

	/* Compute the time remaining to wait. tv_usec is certainly positive. */

	_result->tv_sec  = _x->tv_sec  - _y->tv_sec;
	_result->tv_usec = _x->tv_usec - _y->tv_usec;

	/* Return 1 if result is negative. */

	return _x->tv_sec < _y->tv_sec;
}



#endif  // __debugfunctions__
