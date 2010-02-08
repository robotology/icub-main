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

#include "ThreadPool2.h"

ThreadPool2::ThreadPool2(yarp::dev::ICanBufferFactory *ic)
{
    index=0;
    pool=new ThreadTable2[CANCONTROL_MAX_THREADS];
    for(int k=0;k<CANCONTROL_MAX_THREADS;k++)
        pool[k].init(ic);
}

ThreadPool2::~ThreadPool2()
{
    delete [] pool;
}

