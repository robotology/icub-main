// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2013  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Marco Accame
 * email: marco.accame@iit.it
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

//
// $Id: embObjLibConfig.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __embObjLibConfig_h_
#define __embObjLibConfig_h_

// always use that. as an alternative one could use ETHMANAGER_LOCK_WITH_RECURSIVE_MUTEX
#define     ETHMANAGER_LOCK_WITH_SEMAPHORE_MUTEX

// one could undef this and use ... HOSTTRANSCEIVER_LOCK_WITH_SEMAPHORE_MUTEX
#define     HOSTTRANSCEIVER_LOCK_ALL_EXTERNALLY_WITH_SEMAPHORE_MUTEX
#undef     HOSTTRANSCEIVER_LOCK_EMBOBJTRANSCEIVER_INTERNALLY
#undef     HOSTTRANSCEIVER_LOCK_EMBOBJNETVARS_INTERNALLY_BY_DEVICE
#undef     HOSTTRANSCEIVER_LOCK_EMBOBJNETVARS_INTERNALLY_BY_ENDPOINT
#undef     HOSTTRANSCEIVER_LOCK_EMBOBJNETVARS_INTERNALLY_BY_NETVAR

#endif
