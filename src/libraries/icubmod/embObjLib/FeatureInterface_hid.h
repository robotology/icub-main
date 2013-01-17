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
#include "embObjMotionControl.h"

using namespace yarp::sig;

class IiCubFeature
{
    public:
        virtual ~IiCubFeature() {};
//         virtual void setId(FEAT_ID &id) { };
// None of the following is a "MUST" because each module need just one of them
        virtual bool fillData(void *data )  =0;         //skin data
//         virtual bool fill_AS_data(void *as_array) =0;   // analogSensor data
};

#endif /* FEATUREINTERFACE_HID_H_ */

