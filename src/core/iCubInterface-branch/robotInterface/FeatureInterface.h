/*
 * FeatureInterface.h
 *
 *  Created on: May 30, 2012
 *      Author: Alberto Cardellino
 */

#ifndef FEATUREINTERFACE_H_
#define FEATUREINTERFACE_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	MotionControl		= 0x00,
	Skin				= 0x01
}FeatureType;

typedef struct
{
	FeatureType		type;
	char 			name[64];
}FEAT_ID;

extern void *getRobotFeatureList_C(FEAT_ID *);
bool findAndFill(FEAT_ID *id, char *sk_array);



#ifdef __cplusplus
}
#endif

#endif /* FEATUREINTERFACE_H_ */

