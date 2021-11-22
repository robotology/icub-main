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

#ifndef __COMMONCARTESIANCONTROLLER_H__
#define __COMMONCARTESIANCONTROLLER_H__

#include <yarp/os/Vocab.h>

#define IKINCARTCTRL_VOCAB_CMD_GET              yarp::os::createVocab32('g','e','t')
#define IKINCARTCTRL_VOCAB_CMD_SET              yarp::os::createVocab32('s','e','t')
#define IKINCARTCTRL_VOCAB_CMD_ASK              yarp::os::createVocab32('a','s','k')
#define IKINCARTCTRL_VOCAB_CMD_STORE            yarp::os::createVocab32('s','t','o','r')
#define IKINCARTCTRL_VOCAB_CMD_RESTORE          yarp::os::createVocab32('r','e','s','t')
#define IKINCARTCTRL_VOCAB_CMD_DELETE           yarp::os::createVocab32('d','e','l')
#define IKINCARTCTRL_VOCAB_CMD_GO               yarp::os::createVocab32('g','o')
#define IKINCARTCTRL_VOCAB_CMD_TASKVEL          yarp::os::createVocab32('t','v','e','l')
#define IKINCARTCTRL_VOCAB_CMD_STOP             yarp::os::createVocab32('s','t','o','p')
#define IKINCARTCTRL_VOCAB_CMD_EVENT            yarp::os::createVocab32('e','v','e','n')
#define IKINCARTCTRL_VOCAB_OPT_MODE             yarp::os::createVocab32('m','o','d','e')
#define IKINCARTCTRL_VOCAB_OPT_REFERENCE        yarp::os::createVocab32('r','e','f','e')
#define IKINCARTCTRL_VOCAB_OPT_PRIO             yarp::os::createVocab32('p','r','i','o')
#define IKINCARTCTRL_VOCAB_OPT_TIME             yarp::os::createVocab32('t','i','m','e')
#define IKINCARTCTRL_VOCAB_OPT_TOL              yarp::os::createVocab32('t','o','l')
#define IKINCARTCTRL_VOCAB_OPT_DOF              yarp::os::createVocab32('d','o','f')
#define IKINCARTCTRL_VOCAB_OPT_REST_POS         yarp::os::createVocab32('r','e','s','p')
#define IKINCARTCTRL_VOCAB_OPT_REST_WEIGHTS     yarp::os::createVocab32('r','e','s','w')
#define IKINCARTCTRL_VOCAB_OPT_DES              yarp::os::createVocab32('d','e','s')
#define IKINCARTCTRL_VOCAB_OPT_LIM              yarp::os::createVocab32('l','i','m')
#define IKINCARTCTRL_VOCAB_OPT_XD               yarp::os::createVocab32('x','d')
#define IKINCARTCTRL_VOCAB_OPT_X                yarp::os::createVocab32('x')
#define IKINCARTCTRL_VOCAB_OPT_Q                yarp::os::createVocab32('q')
#define IKINCARTCTRL_VOCAB_OPT_XDOT             yarp::os::createVocab32('x','d','o','t')
#define IKINCARTCTRL_VOCAB_OPT_QDOT             yarp::os::createVocab32('q','d','o','t')
#define IKINCARTCTRL_VOCAB_OPT_TIP_FRAME        yarp::os::createVocab32('t','i','p')
#define IKINCARTCTRL_VOCAB_OPT_MOTIONDONE       yarp::os::createVocab32('d','o','n','e')
#define IKINCARTCTRL_VOCAB_OPT_ISSOLVERON       yarp::os::createVocab32('i','s','o','n')
#define IKINCARTCTRL_VOCAB_OPT_POSE             yarp::os::createVocab32('p','o','s','e')
#define IKINCARTCTRL_VOCAB_OPT_INFO             yarp::os::createVocab32('i','n','f','o')
#define IKINCARTCTRL_VOCAB_OPT_TWEAK            yarp::os::createVocab32('t','w','e','a')
#define IKINCARTCTRL_VOCAB_OPT_REGISTER         yarp::os::createVocab32('r','e','g','i')
#define IKINCARTCTRL_VOCAB_OPT_UNREGISTER       yarp::os::createVocab32('u','n','r','e')
#define IKINCARTCTRL_VOCAB_OPT_LIST             yarp::os::createVocab32('l','i','s','t')
#define IKINCARTCTRL_VOCAB_VAL_POSE_FULL        yarp::os::createVocab32('f','u','l','l')
#define IKINCARTCTRL_VOCAB_VAL_POSE_XYZ         yarp::os::createVocab32('x','y','z')
#define IKINCARTCTRL_VOCAB_VAL_MODE_TRACK       yarp::os::createVocab32('c','o','n','t')
#define IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE      yarp::os::createVocab32('s','h','o','t')
#define IKINCARTCTRL_VOCAB_VAL_TRUE             yarp::os::createVocab32('t','r','u','e')
#define IKINCARTCTRL_VOCAB_VAL_FALSE            yarp::os::createVocab32('f','a','l','s')
#define IKINCARTCTRL_VOCAB_VAL_EVENT_ONGOING    yarp::os::createVocab32('o','g','o','i')
#define IKINCARTCTRL_VOCAB_REP_ACK              yarp::os::createVocab32('a','c','k')
#define IKINCARTCTRL_VOCAB_REP_NACK             yarp::os::createVocab32('n','a','c','k')

#endif

