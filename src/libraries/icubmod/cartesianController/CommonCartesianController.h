/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Developed by Ugo Pattacini

#ifndef __COMMONCARTESIANCONTROLLER_H__
#define __COMMONCARTESIANCONTROLLER_H__

#include <yarp/os/Vocab.h>

#define IKINCARTCTRL_VOCAB_CMD_GET          VOCAB3('g','e','t')
#define IKINCARTCTRL_VOCAB_CMD_SET          VOCAB3('s','e','t')
#define IKINCARTCTRL_VOCAB_CMD_ASK          VOCAB3('a','s','k')
#define IKINCARTCTRL_VOCAB_CMD_STORE        VOCAB4('s','t','o','r')
#define IKINCARTCTRL_VOCAB_CMD_RESTORE      VOCAB4('r','e','s','t')
#define IKINCARTCTRL_VOCAB_CMD_DELETE       VOCAB3('d','e','l')
#define IKINCARTCTRL_VOCAB_CMD_GO           VOCAB2('g','o')
#define IKINCARTCTRL_VOCAB_CMD_TASKVEL      VOCAB4('t','v','e','l')
#define IKINCARTCTRL_VOCAB_CMD_STOP         VOCAB4('s','t','o','p')
#define IKINCARTCTRL_VOCAB_OPT_MODE         VOCAB4('m','o','d','e')
#define IKINCARTCTRL_VOCAB_OPT_TIME         VOCAB4('t','i','m','e')
#define IKINCARTCTRL_VOCAB_OPT_TOL          VOCAB3('t','o','l')
#define IKINCARTCTRL_VOCAB_OPT_DOF          VOCAB3('d','o','f')
#define IKINCARTCTRL_VOCAB_OPT_REST_POS     VOCAB4('r','e','s','p')
#define IKINCARTCTRL_VOCAB_OPT_REST_WEIGHTS VOCAB4('r','e','s','w')
#define IKINCARTCTRL_VOCAB_OPT_DES          VOCAB3('d','e','s')
#define IKINCARTCTRL_VOCAB_OPT_LIM          VOCAB3('l','i','m')
#define IKINCARTCTRL_VOCAB_OPT_XD           VOCAB2('x','d')
#define IKINCARTCTRL_VOCAB_OPT_X            VOCAB1('x')
#define IKINCARTCTRL_VOCAB_OPT_Q            VOCAB1('q')
#define IKINCARTCTRL_VOCAB_OPT_XDOT         VOCAB4('x','d','o','t')
#define IKINCARTCTRL_VOCAB_OPT_QDOT         VOCAB4('q','d','o','t')
#define IKINCARTCTRL_VOCAB_OPT_MOTIONDONE   VOCAB4('d','o','n','e')
#define IKINCARTCTRL_VOCAB_OPT_ISSOLVERON   VOCAB4('i','s','o','n')
#define IKINCARTCTRL_VOCAB_OPT_POSE         VOCAB4('p','o','s','e')
#define IKINCARTCTRL_VOCAB_VAL_POSE_FULL    VOCAB4('f','u','l','l')
#define IKINCARTCTRL_VOCAB_VAL_POSE_XYZ     VOCAB3('x','y','z')
#define IKINCARTCTRL_VOCAB_VAL_MODE_TRACK   VOCAB4('c','o','n','t')
#define IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE  VOCAB4('s','h','o','t')
#define IKINCARTCTRL_VOCAB_VAL_TRUE         VOCAB4('t','r','u','e')
#define IKINCARTCTRL_VOCAB_VAL_FALSE        VOCAB4('f','a','l','s')
#define IKINCARTCTRL_VOCAB_REP_ACK          VOCAB3('a','c','k')
#define IKINCARTCTRL_VOCAB_REP_NACK         VOCAB4('n','a','c','k')

#endif

