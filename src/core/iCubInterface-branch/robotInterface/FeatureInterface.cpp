/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include "FeatureInterface.h"
#include "FeatureInterface_hid.h"
#include "IRobotInterface.h"
#include <ethManager.h>

bool findAndFill(FEAT_ID *id, char *sk_array)
{
    // new with table, data stored in eoSkin;
    IiCubFeature * skin =  (IiCubFeature*) (ethResCreator::instance()->getHandleFromEP(id->ep));
    if(NULL == skin)
    {
//        printf( "/************************************\\\n"
//                "            Parte non trovata!!!\n"
//                "\\***********************************/\n");
        return false;
    }
    else
        skin->fillData(sk_array);

    return true;
}
