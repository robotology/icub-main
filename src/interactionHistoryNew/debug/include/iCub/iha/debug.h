// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Assif Mirza
 * email:   assif.mirza@robotcub.org
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

#ifndef PMESG_DEBUG_H
#define PMESG_DEBUG_H
#include <stdarg.h>

// Define levels of debug printing

// No Debug
#define DBGL_NONE 0
// Information - e.g. startup
#define	DBGL_INFO 10
// pure status line (with \r)
#define DBGL_STATUSLINE 15
// Running status info
#define	DBGL_STATUS1 20
// More detailed running status info
#define	DBGL_STATUS2 30
// 1st level debug
#define	DBGL_DEBUG1 40
// Detailed debug
#define	DBGL_DEBUG2 50
// Too much detail
#define	DBGL_DEBUG3 60


namespace iCub {
	namespace iha {
		class IhaDebug;
	}
}

class iCub::iha::IhaDebug 
{
public:

    static int msglevel; /* the higher, the more messages... */

	IhaDebug();
	//IhaDebug(int ml);
	~IhaDebug();

	/**
	 * pmesg
	 * print a message, if it is considered significant enough.
	 *    Adapted from [K&R2], p. 174 
	 */
	static void pmesg(int level, const char *format, ...);

	static void setLevel(int level);
	static int getLevel();

};

#endif /* DEBUG_H */
