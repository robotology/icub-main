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

#include "ThreadTable2.h"
#include "canControlConstants.h"

ThreadTable2::ThreadTable2():_synch(0)
{
    ic=0;
    clear();
}

ThreadTable2::~ThreadTable2()
{
    if(ic!=0)
        ic->destroyBuffer(_replies);
    ic=0;
}

void ThreadTable2::clear()
{
    lock();
    _pending=BUF_SIZE;
    _replied=0;
    unlock();
}

void ThreadTable2::init(yarp::dev::ICanBufferFactory *i)
{
    ic=i;
    _replies=ic->createBuffer(BUF_SIZE);
}

