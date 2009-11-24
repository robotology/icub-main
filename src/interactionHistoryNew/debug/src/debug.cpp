// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
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

#include <iCub/iha/debug.h>
#include <ace/OS.h>
#include <stdio.h>

iCub::iha::IhaDebug::IhaDebug() {
    IhaDebug::msglevel=DBGL_INFO; // default
}
//iCub::iha::IhaDebug::IhaDebug(int ml) : msglevel(ml) {
//}
iCub::iha::IhaDebug::~IhaDebug() {
}

void iCub::iha::IhaDebug::pmesg(int level, const char* format, ...) {
#ifdef NDEBUG
	/* Empty body, so a good compiler will optimise calls
	   to pmesg away */
#else
        va_list args;

        if (level>IhaDebug::msglevel)
                return;

		// don't print a status line message in any other
		// level than status line - to stop overwriting when using \r
		if (level==DBGL_STATUSLINE && IhaDebug::msglevel!=DBGL_STATUSLINE) 
				return;

        va_start(args, format);
        vfprintf(stderr, format, args);
        va_end(args);
        // may need this to get messages from a segfault process
        // but it also may slow things down :(
        ACE_OS::fflush(stderr);
#endif /* NDEBUG */
}

void iCub::iha::IhaDebug::setLevel(int lvl) {
    IhaDebug::msglevel=lvl;
}

int iCub::iha::IhaDebug::getLevel() {
    return IhaDebug::msglevel;
}

int iCub::iha::IhaDebug::msglevel = DBGL_NONE;

