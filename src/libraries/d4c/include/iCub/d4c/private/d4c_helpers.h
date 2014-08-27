/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef __D4C_HELPERS_H__
#define __D4C_HELPERS_H__

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#define D4C_VOCAB_CMD_PING                 VOCAB4('p','i','n','g')
#define D4C_VOCAB_CMD_ADD                  VOCAB3('a','d','d')
#define D4C_VOCAB_CMD_DEL                  VOCAB3('d','e','l')
#define D4C_VOCAB_CMD_SET                  VOCAB3('s','e','t')
#define D4C_VOCAB_CMD_GET                  VOCAB3('g','e','t')
#define D4C_VOCAB_CMD_LIST                 VOCAB4('l','i','s','t')
#define D4C_VOCAB_CMD_CLEAR                VOCAB4('c','l','e','a')
#define D4C_VOCAB_CMD_ENFIELD              VOCAB4('f','e','n','a')
#define D4C_VOCAB_CMD_DISFIELD             VOCAB4('f','d','i','s')
#define D4C_VOCAB_CMD_STATFIELD            VOCAB4('f','s','t','a')
#define D4C_VOCAB_CMD_ENCTRL               VOCAB4('c','e','n','a')
#define D4C_VOCAB_CMD_DISCTRL              VOCAB4('c','d','i','s')
#define D4C_VOCAB_CMD_STATCTRL             VOCAB4('c','s','t','a')
#define D4C_VOCAB_CMD_ENSIM                VOCAB4('m','e','n','a')
#define D4C_VOCAB_CMD_DISSIM               VOCAB4('m','d','i','s')
#define D4C_VOCAB_CMD_STATSIM              VOCAB4('m','s','t','a')
#define D4C_VOCAB_CMD_SETPER               VOCAB4('s','p','e','r')
#define D4C_VOCAB_CMD_GETPER               VOCAB4('g','p','e','r')
#define D4C_VOCAB_CMD_SETACTIF             VOCAB4('s','a','i','f')
#define D4C_VOCAB_CMD_GETACTIF             VOCAB4('g','a','i','f')
#define D4C_VOCAB_CMD_GETTRAJ              VOCAB4('g','t','r','j')
#define D4C_VOCAB_CMD_EXECTRAJ             VOCAB4('e','t','r','j')
#define D4C_VOCAB_CMD_SETSTATETOTOOL       VOCAB4('s','s','t','t')
#define D4C_VOCAB_CMD_SETSTATE             VOCAB4('s','s','t','a')
#define D4C_VOCAB_CMD_SETORIEN             VOCAB4('s','o','r','i')
#define D4C_VOCAB_CMD_GETSTATE             VOCAB4('g','s','t','a')
#define D4C_VOCAB_CMD_ATTACHTOOLFRAME      VOCAB4('t','a','t','f')
#define D4C_VOCAB_CMD_GETTOOLFRAME         VOCAB4('t','g','e','f')
#define D4C_VOCAB_CMD_REMOVETOOLFRAME      VOCAB4('t','r','e','f')
#define D4C_VOCAB_CMD_GETTOOL              VOCAB4('t','g','e','t')
#define D4C_VOCAB_CMD_ACK                  VOCAB3('a','c','k')
#define D4C_VOCAB_CMD_NACK                 VOCAB4('n','a','c','k')

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


