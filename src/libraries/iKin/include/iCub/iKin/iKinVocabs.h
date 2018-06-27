/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

#ifndef __IKINVOCABS_H__
#define __IKINVOCABS_H__

#include <yarp/os/Vocab.h>

#define IKINSLV_VOCAB_CMD_GET           yarp::os::createVocab('g','e','t')
#define IKINSLV_VOCAB_CMD_SET           yarp::os::createVocab('s','e','t')
#define IKINSLV_VOCAB_CMD_ASK           yarp::os::createVocab('a','s','k')
#define IKINSLV_VOCAB_CMD_SUSP          yarp::os::createVocab('s','u','s','p')
#define IKINSLV_VOCAB_CMD_RUN           yarp::os::createVocab('r','u','n')
#define IKINSLV_VOCAB_CMD_STATUS        yarp::os::createVocab('s','t','a','t')
#define IKINSLV_VOCAB_CMD_HELP          yarp::os::createVocab('h','e','l','p')
#define IKINSLV_VOCAB_CMD_CFG           yarp::os::createVocab('c','f','g')
#define IKINSLV_VOCAB_CMD_QUIT          yarp::os::createVocab('q','u','i','t')
#define IKINSLV_VOCAB_OPT_MODE          yarp::os::createVocab('m','o','d','e')
#define IKINSLV_VOCAB_OPT_POSE          yarp::os::createVocab('p','o','s','e')
#define IKINSLV_VOCAB_OPT_PRIO          yarp::os::createVocab('p','r','i','o')
#define IKINSLV_VOCAB_OPT_DOF           yarp::os::createVocab('d','o','f')
#define IKINSLV_VOCAB_OPT_LIM           yarp::os::createVocab('l','i','m')
#define IKINSLV_VOCAB_OPT_XD            yarp::os::createVocab('x','d')
#define IKINSLV_VOCAB_OPT_X             yarp::os::createVocab('x')
#define IKINSLV_VOCAB_OPT_Q             yarp::os::createVocab('q')
#define IKINSLV_VOCAB_OPT_TOKEN         yarp::os::createVocab('t','o','k')
#define IKINSLV_VOCAB_OPT_VERB          yarp::os::createVocab('v','e','r','b')
#define IKINSLV_VOCAB_OPT_REST_POS      yarp::os::createVocab('r','e','s','p')
#define IKINSLV_VOCAB_OPT_REST_WEIGHTS  yarp::os::createVocab('r','e','s','w')
#define IKINSLV_VOCAB_OPT_TIP_FRAME     yarp::os::createVocab('t','i','p')
#define IKINSLV_VOCAB_OPT_TASK2         yarp::os::createVocab('t','s','k','2')
#define IKINSLV_VOCAB_OPT_CONVERGENCE   yarp::os::createVocab('c','o','n','v')
#define IKINSLV_VOCAB_VAL_POSE_FULL     yarp::os::createVocab('f','u','l','l')
#define IKINSLV_VOCAB_VAL_POSE_XYZ      yarp::os::createVocab('x','y','z')
#define IKINSLV_VOCAB_VAL_PRIO_XYZ      yarp::os::createVocab('x','y','z')
#define IKINSLV_VOCAB_VAL_PRIO_ANG      yarp::os::createVocab('a','n','g')
#define IKINSLV_VOCAB_VAL_MODE_TRACK    yarp::os::createVocab('c','o','n','t')
#define IKINSLV_VOCAB_VAL_MODE_SINGLE   yarp::os::createVocab('s','h','o','t')
#define IKINSLV_VOCAB_VAL_ON            yarp::os::createVocab('o','n')
#define IKINSLV_VOCAB_VAL_OFF           yarp::os::createVocab('o','f','f')
#define IKINSLV_VOCAB_REP_ACK           yarp::os::createVocab('a','c','k')
#define IKINSLV_VOCAB_REP_NACK          yarp::os::createVocab('n','a','c','k')

#endif
