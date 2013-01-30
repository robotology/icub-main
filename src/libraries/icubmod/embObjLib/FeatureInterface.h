/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef FEATUREINTERFACE_H_
#define FEATUREINTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

#include <string.h>

#ifdef WIN32
	#pragma warning(disable:4355)
#endif

#include "EOnv_hid.h"
#include "EoMotionControl.h"

#define SIZE_INFO		128

#ifdef __cplusplus

extern "C"
{
#endif

#include "EOconstvector_hid.h"


typedef enum
{
    Management    = 0x00,
    AnalogMais    = 0x01,
    AnalogStrain  = 0x02,
    MotionControl = 0x03,
    Skin          = 0x04
} FeatureType;


typedef struct
{
    uint8_t       boardNum;
    eOnvEP_t      ep;
    void         *handle;

    // eoStuff
    const EOconstvector  *EPvector;
    eOuint16_fp_uint16_t  EPhash_function;

    // Following are additional and optional info for debug, DO NOT COUNT ON THEM as identifiers for searches!!
    // They may be removed very soon!
    FeatureType           type;
    char                  name[SIZE_INFO];
}FEAT_ID;

bool addEncoderTimeStamp(FEAT_ID *id, int jointNum);
bool findAndFill(FEAT_ID *id, void *sk_array);
bool handle_AS_data(FEAT_ID *id, void *as_array);
void * get_MChandler_fromEP(eOnvEP_t ep);
bool MCmutex_post(void * p, uint16_t epindex, uint16_t nvindex);

bool EP_NV_2_index(eOnvEP_t ep, eOnvID_t nvid, uint16_t *epindex, uint16_t *nvindex);

void transceiver_wait(eOnvEP_t ep);
void transceiver_post(eOnvEP_t ep);

#ifdef __cplusplus
}
#endif

#endif /* FEATUREINTERFACE_H_ */

