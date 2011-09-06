/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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

#ifndef __PMP_HELPERS_H__
#define __PMP_HELPERS_H__

#include <string>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#define PMP_VOCAB_CMD_PING                 VOCAB4('p','i','n','g')
#define PMP_VOCAB_CMD_ADD                  VOCAB3('a','d','d')
#define PMP_VOCAB_CMD_DEL                  VOCAB3('d','e','l')
#define PMP_VOCAB_CMD_SET                  VOCAB3('s','e','t')
#define PMP_VOCAB_CMD_GET                  VOCAB3('g','e','t')
#define PMP_VOCAB_CMD_LIST                 VOCAB4('l','i','s','t')
#define PMP_VOCAB_CMD_CLEAR                VOCAB4('c','l','e','a')
#define PMP_VOCAB_CMD_ENFIELD              VOCAB4('f','e','n','a')
#define PMP_VOCAB_CMD_DISFIELD             VOCAB4('f','d','i','s')
#define PMP_VOCAB_CMD_STATFIELD            VOCAB4('f','s','t','a')
#define PMP_VOCAB_CMD_ENCTRL               VOCAB4('c','e','n','a')
#define PMP_VOCAB_CMD_DISCTRL              VOCAB4('c','d','i','s')
#define PMP_VOCAB_CMD_STATCTRL             VOCAB4('c','s','t','a')
#define PMP_VOCAB_CMD_ENSIM                VOCAB4('m','e','n','a')
#define PMP_VOCAB_CMD_DISSIM               VOCAB4('m','d','i','s')
#define PMP_VOCAB_CMD_STATSIM              VOCAB4('m','s','t','a')
#define PMP_VOCAB_CMD_SETPER               VOCAB4('s','p','e','r')
#define PMP_VOCAB_CMD_GETPER               VOCAB4('g','p','e','r')
#define PMP_VOCAB_CMD_SETSTATETOTOOL       VOCAB4('s','s','t','t')
#define PMP_VOCAB_CMD_SETSTATE             VOCAB4('s','s','t','a')
#define PMP_VOCAB_CMD_GETSTATE             VOCAB4('g','s','t','a')
#define PMP_VOCAB_CMD_ATTACHTOOLFRAME      VOCAB4('t','a','t','f')
#define PMP_VOCAB_CMD_GETTOOLFRAME         VOCAB4('t','g','e','f')
#define PMP_VOCAB_CMD_REMOVETOOLFRAME      VOCAB4('t','r','e','f')
#define PMP_VOCAB_CMD_GETTOOL              VOCAB4('t','g','e','t')
#define PMP_VOCAB_CMD_ACK                  VOCAB3('a','c','k')
#define PMP_VOCAB_CMD_NACK                 VOCAB4('n','a','c','k')

#define extractProperty(value)             (Property(value.asList()->toString().c_str()))


inline yarp::sig::Vector getVectorPos(const yarp::sig::Vector &x)
{
    return x.subVector(0,2);
}

inline yarp::sig::Vector getVectorOrien(const yarp::sig::Vector &x)
{
    return x.subVector(3,6);
}

bool copyVectorData(const yarp::sig::Vector &src, yarp::sig::Vector &dest);

bool extractVector(yarp::os::Property &prop, const std::string &option,
                   yarp::sig::Vector &res);

#endif


