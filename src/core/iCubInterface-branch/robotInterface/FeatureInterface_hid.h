/*
 * FeatureInterface.h
 *
 *  Created on: May 30, 2012
 *      Author: Alberto Cardellino
 */

#ifndef FEATUREINTERFACE_HID_H_
#define FEATUREINTERFACE_HID_H_

#include <ace/ACE.h>
#include <yarp/sig/Vector.h>
using namespace yarp::sig;


class IiCubFeature
{
	public:
		virtual	~IiCubFeature() {};
		virtual bool fillData(char *data ) =0;
		virtual Vector * getData() =0;
		virtual bool pushData(yarp::sig::Vector &in) =0;
};

class IiCubFeatureList
{
	public:

		virtual	~IiCubFeatureList() {};

		// initialize the transceiver
		virtual IiCubFeature * findus(FEAT_ID *id ) =0;
	    IiCubFeature * findus2(FEAT_ID *id );
};

//IiCubFeatureList *getRobotFeatureList_C();

#endif /* FEATUREINTERFACE_HID_H_ */

