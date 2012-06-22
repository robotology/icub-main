// Copyright: (C) 2010 RobotCub Consortium
// Authors: Alberto Cardellino
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __DEBUGGING_H__
#define __DEBUGGING_H__

#ifndef _AC_
//	#define _AC_
#endif


#ifdef _AC_
#define AC_YARP_INFO(args...)  YARP_INFO(args)
#else
#define AC_YARP_INFO
#endif



#endif  // __DEBUGGING_H__
