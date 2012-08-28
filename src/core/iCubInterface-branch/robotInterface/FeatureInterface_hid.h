/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef FEATUREINTERFACE_HID_H_
#define FEATUREINTERFACE_HID_H_

#include <ace/ACE.h>
#include <yarp/sig/Vector.h>
#include "FeatureInterface.h"

using namespace yarp::sig;


class IiCubFeature
{
    public:
        virtual ~IiCubFeature() {};
        virtual bool fillData(char *data ) =0;

        virtual void setId(FEAT_ID &id) { };
};

#endif /* FEATUREINTERFACE_HID_H_ */

