/*
 * FeatureInterface.h
 *
 *  Created on: May 30, 2012
 *      Author: Alberto Cardellino
 */

#ifndef FEATUREINTERFACE_HID_H_
#define FEATUREINTERFACE_HID_H_

#include <ace/ACE.h>




class IiCubFeature
{
	public:
		virtual	~IiCubFeature() {};
		virtual bool fillData(uint8_t &data ) =0;
};

class IiCubFeatureList
{
	public:

		virtual	~IiCubFeatureList() {};

		// initialize the transceiver
		virtual IiCubFeature * findus(FEAT_ID *id ) =0;
};

//IiCubFeatureList *getRobotFeatureList_C();

#endif /* FEATUREINTERFACE_HID_H_ */

