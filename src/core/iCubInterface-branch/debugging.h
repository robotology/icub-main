/*
 * Copyright (C) <year>  iCub Facility, Istituto Italiano di Tecnologia
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

#ifndef __DEBUGGING_H__
#define __DEBUGGING_H__

#ifndef _AC_
//	#define _AC_
#endif

#undef _AC_


#ifdef _AC_
#define AC_YARP_INFO(args...)  YARP_INFO(args)
#else
#define AC_YARP_INFO
#endif



#endif  // __DEBUGGING_H__
