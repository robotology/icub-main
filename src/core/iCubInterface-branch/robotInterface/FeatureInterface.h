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

#ifdef __cplusplus

extern "C"
{
#endif

typedef enum
{
	MotionControl		= 0x00,
	Skin				= 0x01,
	Analog				= 0x02
}FeatureType;



typedef struct
{
	FeatureType		type;
	uint8_t 		ep;
	char 			name[64];
	void			*handle;
}FEAT_ID;

bool findAndFill(FEAT_ID *id, char *sk_array);

#ifdef __cplusplus
}
#endif

#endif /* FEATUREINTERFACE_H_ */

